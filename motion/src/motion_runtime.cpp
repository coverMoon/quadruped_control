/**
 * @file motion_runtime.cpp
 * @brief 实现 MotionRuntime 的周期更新主流程、会话跟踪和命令提交。
 */

#include "quadruped/motion/motion_runtime.hpp"

#include "quadruped/core/validation.hpp"

#include <cstdint>
#include <string>
#include <utility>

namespace quadruped::motion
{
namespace
{

// 面向错误信息的 RobotIO 结果描述；只用于日志和界面显示。
const char* read_failure_message(core::RobotIOCode code)
{
    switch (code)
    {
    case core::RobotIOCode::NoData:
        return "no state available";
    case core::RobotIOCode::Disconnected:
        return "robot io disconnected";
    case core::RobotIOCode::InvalidFrame:
        return "robot io reported invalid state frame";
    case core::RobotIOCode::Rejected:
        return "robot io rejected state read";
    case core::RobotIOCode::Fault:
        return "robot io fault";
    case core::RobotIOCode::Ok:
        break;
    }
    return "unknown robot io error";
}

}  // 匿名命名空间

MotionRuntime::MotionRuntime(core::RobotModel model, core::ControllerConfig config)
    : model_(std::move(model)), config_(config)
{
}

MotionRuntime::CreateResult MotionRuntime::create(
    core::RobotModel model,
    core::ControllerConfig config)
{
    CreateResult result;
    if (const auto validation = core::validate(model); !validation)
    {
        result.error_message = "invalid RobotModel: " + validation.message;
        return result;
    }
    if (const auto validation = core::validate(config, model); !validation)
    {
        result.error_message = "invalid ControllerConfig: " + validation.message;
        return result;
    }
    result.runtime.reset(new MotionRuntime(std::move(model), config));
    return result;
}

bool MotionRuntime::attach_policy(
    RlConfig config,
    Policy& policy,
    std::string& error_message)
{
    auto created = RlController::create(model_, config);
    if (!created.ok())
    {
        error_message = created.error_message;
        return false;
    }
    policy_name_ = config.name;
    rl_controller_ = std::move(created.controller);
    policy_ = &policy;
    rl_control_cycle_ = 0;
    rl_command_ = {};
    status_.policy_name = policy_name_;
    status_.policy_ready = true;
    error_message.clear();
    return true;
}

bool MotionRuntime::track_session(const core::StateFrame& state)
{
    if (state.header.startup_id == startup_id_ && state.header.session_id == session_id_)
    {
        return false;
    }

    // 从已建立会话切换时先给旧活动请求一个 Failed 终态，避免结果静默丢失。
    const bool switched = session_id_ != 0;
    if (has_active_request_)
    {
        abort_active_request("session changed");
    }

    // 新会话：回到 Passive，清空未完成请求、rest_pose，命令序号从 1 重新开始。
    startup_id_ = state.header.startup_id;
    session_id_ = state.header.session_id;
    command_sequence_ = 0;
    latest_request_id_ = 0;
    active_request_id_ = 0;
    active_result_ = {};
    has_active_request_ = false;
    terminal_count_ = 0;
    terminal_next_ = 0;
    has_rest_pose_ = false;
    getup_second_phase_ = false;
    rl_control_cycle_ = 0;
    rl_command_ = {};
    if (rl_controller_ != nullptr)
    {
        rl_controller_->reset();
    }
    mode_ = core::MotionMode::Passive;
    status_.active_source = core::CommandSource::None;
    status_.behavior_name.clear();
    status_.behavior_phase.clear();
    status_.error_message.clear();
    return switched;
}

bool MotionRuntime::check_active_preconditions(std::string& reason) const
{
    if (!joints_ready_)
    {
        reason = "joints are not online and valid";
        return false;
    }
    if (latest_safety_ != core::SafetyState::ControlEnabled)
    {
        reason = "executor safety state is not ControlEnabled";
        return false;
    }
    return true;
}

void MotionRuntime::fail_active_motion(const std::string& reason)
{
    abort_active_request(reason);
    mode_ = core::MotionMode::Passive;
    status_.error_message = reason;
}

core::RobotIOCode MotionRuntime::submit_command(
    core::RobotIO& io,
    const CommandSubmission& submission)
{
    core::CommandFrame command;
    command.header.schema_version = core::kFrameSchemaVersion;
    command.header.startup_id = startup_id_;
    command.header.session_id = session_id_;
    command.header.sequence = ++command_sequence_;
    command.header.timestamp_ns = submission.now_ns;
    // 单调时间加有效期在极端输入下可能溢出；溢出时按最长可表示时间处理。
    command.expires_at_ns = (submission.now_ns > INT64_MAX - config_.command_validity_ns)
        ? INT64_MAX
        : submission.now_ns + config_.command_validity_ns;
    command.joint_count = model_.joint_count;
    command.motion_mode = mode_;
    command.source = submission.source;

    for (std::size_t i = 0; i < model_.joint_count; ++i)
    {
        auto& joint = command.joints[i];
        if (!submission.use_impedance)
        {
            // Passive 使用显式 Disabled，让机器人不受主动保持地自然运动。
            joint.mode = core::ControlMode::Disabled;
            continue;
        }
        joint.mode = core::ControlMode::JointImpedance;
        joint.target_position = submission.positions[i];
        joint.target_velocity = submission.velocities == nullptr
            ? 0.0
            : (*submission.velocities)[i];
        joint.kp = submission.kp == nullptr ? config_.fixed_kp[i] : (*submission.kp)[i];
        joint.kd = submission.kd == nullptr ? config_.fixed_kd[i] : (*submission.kd)[i];
        joint.feedforward_effort = 0.0;
    }
    return io.submit(command);
}

MotionRuntime::StateRead MotionRuntime::read_state(
    core::RobotIO& io,
    const core::Nanoseconds now_ns)
{
    StateRead result;
    result.code = io.read_latest(result.frame);
    result.failure_reason = read_failure_message(result.code);
    if (result.code != core::RobotIOCode::Ok)
    {
        return result;
    }

    // 只有通过核心校验的状态才能用于会话跟踪、请求接受和命令生成。
    if (const auto validation = core::validate(result.frame, model_, now_ns); !validation)
    {
        result.failure_reason = "state validation failed: " + validation.message;
        status_.error_message = result.failure_reason;
        return result;
    }

    result.usable = true;
    result.now_ns = now_ns;
    for (std::size_t i = 0; i < model_.joint_count; ++i)
    {
        current_positions_[i] = result.frame.joints[i].position;
    }
    joints_ready_ = true;
    for (std::size_t i = 0; i < model_.joint_count; ++i)
    {
        if (!result.frame.joints[i].online || !result.frame.joints[i].valid)
        {
            joints_ready_ = false;
            break;
        }
    }
    latest_safety_ = result.frame.safety_state;
    result.session_changed = track_session(result.frame);
    return result;
}

bool MotionRuntime::run_active_mode(
    core::RobotIO& io,
    const StateRead& state,
    MotionUpdateOutput& output)
{
    if (!state.usable)
    {
        // 没有可用状态就无法构造合法命令，依赖旧命令过期进入执行侧安全退路。
        fail_active_motion(state.failure_reason);
        return true;
    }
    if (std::string reason; !check_active_preconditions(reason))
    {
        fail_active_motion(reason);
        output.submit_code = submit_command(io, {current_positions_, false, state.now_ns});
        output.submitted = true;
        return true;
    }

    if (has_active_request_ && active_result_.state == core::ModeResultState::Accepted)
    {
        active_result_.state = core::ModeResultState::Running;
    }
    std::array<double, core::kMaxJoints> positions{};
    const bool use_impedance = advance_active_motion(positions);
    output.submit_code = submit_command(io, {positions, use_impedance, state.now_ns});
    output.submitted = true;
    if (output.submit_code != core::RobotIOCode::Ok &&
        mode_ != core::MotionMode::Passive)
    {
        fail_active_motion("command submit failed");
        return true;
    }
    return false;
}

bool MotionRuntime::run_rl_mode(
    core::RobotIO& io,
    const StateRead& state,
    const core::BaseCommand* const base_command,
    MotionUpdateOutput& output)
{
    const auto fail = [&](const std::string& reason) {
        fail_active_motion(reason);
        status_.active_source = core::CommandSource::None;
        status_.behavior_name.clear();
        status_.behavior_phase.clear();
        if (state.usable)
        {
            output.submit_code = submit_command(io, {current_positions_, false, state.now_ns});
            output.submitted = true;
        }
        return true;
    };

    if (!state.usable)
    {
        return fail(state.failure_reason);
    }
    if (std::string reason; !check_active_preconditions(reason))
    {
        return fail(reason);
    }
    if (rl_controller_ == nullptr || policy_ == nullptr)
    {
        return fail("RL policy is not attached");
    }

    rl_controller_->update_command(base_command, state.now_ns);
    const bool inference_due = !rl_command_.ok || rl_control_cycle_ % kRlDecimation == 0;
    if (inference_due)
    {
        const auto observation = rl_controller_->build_observation(state.frame, state.now_ns);
        if (!observation.ok)
        {
            return fail("RL observation failed: " + observation.error_message);
        }
        rl_controller_->insert_observation(observation.observation);
        const auto inference = policy_->forward(rl_controller_->inference_input());
        if (!inference.ok)
        {
            return fail("RL inference failed: " + inference.error_message);
        }
        if (inference.elapsed_ns > kRlInferenceDeadlineNs)
        {
            return fail("RL inference exceeded deadline: " +
                std::to_string(inference.elapsed_ns / 1'000'000) + " ms");
        }
        rl_command_ = rl_controller_->convert_actions(inference.actions, current_positions_);
        if (!rl_command_.ok)
        {
            return fail("RL action conversion failed: " + rl_command_.error_message);
        }
        if (has_active_request_)
        {
            complete_active_request("behavior started");
        }
    }
    ++rl_control_cycle_;

    status_.active_source = rl_controller_->active_command_source();
    status_.behavior_phase = "driving";
    output.submit_code = submit_command(io,
        {rl_command_.target_positions,
            true,
            state.now_ns,
            &rl_command_.target_velocities,
            &rl_command_.kp,
            &rl_command_.kd,
            status_.active_source});
    output.submitted = true;
    if (output.submit_code != core::RobotIOCode::Ok)
    {
        return fail("RL command submit failed");
    }
    return false;
}

MotionUpdateOutput MotionRuntime::update(core::RobotIO& io, const MotionUpdateInput& input)
{
    MotionUpdateOutput output;

    // 终态事件缓冲每周期从零开始，只收集本周期新产生的终态。
    pending_result_event_count_ = 0;

    const StateRead state = read_state(io, input.now_ns);
    output.read_code = state.code;
    handle_input_request(input, state, output);

    // 本周期是否发生了主动动作失败；失败周期的错误说明不能被随后的成功提交冲掉。
    bool failed_this_cycle = false;
    if (mode_ == core::MotionMode::Passive)
    {
        if (state.usable)
        {
            output.submit_code = submit_command(io, {current_positions_, false, input.now_ns});
            output.submitted = true;
        }
    }
    else if (mode_ == core::MotionMode::Running)
    {
        failed_this_cycle = run_rl_mode(io, state, input.base_command, output);
    }
    else
    {
        failed_this_cycle = run_active_mode(io, state, output);
    }

    if (!failed_this_cycle && output.submitted &&
        output.submit_code == core::RobotIOCode::Ok)
    {
        status_.error_message.clear();
    }

    // 本周期产生的全部终态事件随输出交付。
    output.result_event_count = pending_result_event_count_;
    for (std::size_t i = 0; i < pending_result_event_count_; ++i)
    {
        output.result_events[i] = pending_result_events_[i];
    }

    status_.mode = mode_;
    output.status = status_;
    return output;
}

const char* motion_mode_name(const core::MotionMode mode) noexcept
{
    switch (mode)
    {
    case core::MotionMode::Passive:
        return "Passive";
    case core::MotionMode::GetUp:
        return "GetUp";
    case core::MotionMode::Stand:
        return "Stand";
    case core::MotionMode::Running:
        return "Running";
    case core::MotionMode::GetDown:
        return "GetDown";
    }
    return "Unknown";
}

}  // 命名空间 quadruped::motion
