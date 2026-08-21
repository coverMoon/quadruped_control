/**
 * @file motion_requests.cpp
 * @brief 实现 MotionRuntime 的请求分派、接受逻辑和主动模式插值推进。
 */

#include "quadruped/motion/motion_runtime.hpp"

#include "quadruped/core/validation.hpp"

#include <cstddef>

namespace quadruped::motion
{
namespace
{

core::ModeResult make_result(
    const std::uint64_t request_id,
    const core::ModeResultState state,
    const char* message)
{
    core::ModeResult result;
    result.request_id = request_id;
    result.state = state;
    result.message = message;
    return result;
}

// 线性插值：percent 从 0 到 1，输出 (1-percent)*start + percent*target。
double interpolate(const double start, const double target, const double percent)
{
    return (1.0 - percent) * start + percent * target;
}

}  // 匿名命名空间

void MotionRuntime::handle_input_request(
    const MotionUpdateInput& input,
    const StateRead& state,
    const bool base_command_usable,
    MotionUpdateOutput& output)
{
    if (input.request == nullptr)
    {
        return;
    }
    output.has_result = true;
    if (state.session_changed)
    {
        // 会话切换周期不处理输入请求（旧终态已交付、编号已清零，
        // 同周期分派会把旧请求重启到新会话）；调用方在后续周期重新提交。
        output.result.request_id = input.request->request_id;
        output.result.state = core::ModeResultState::Rejected;
        output.result.message = "session changed; resubmit request";
        return;
    }

    // 重试必须先查当前请求或终态历史，不能因为原请求已经过期而变成新的拒绝。
    // 这样外围 Action 可以使用同一个 request_id 查询 Accepted、Running 或终态。
    if (has_active_request_ && input.request->request_id == active_request_id_)
    {
        output.result = active_result_;
        return;
    }
    core::ModeResult stored{};
    if (find_terminal_result(input.request->request_id, stored))
    {
        output.result = stored;
        return;
    }
    if (input.request->request_id != 0 &&
        input.request->request_id <= latest_request_id_)
    {
        output.result = handle_request(*input.request, state.usable, base_command_usable);
        return;
    }

    if (const auto validation = core::validate(*input.request, input.now_ns); !validation)
    {
        // 字段非法的请求不参与编号去重，直接拒绝且不影响当前运动。
        output.result.request_id = input.request->request_id;
        output.result.state = core::ModeResultState::Rejected;
        output.result.message = "invalid request: " + validation.message;
    }
    else
    {
        output.result = handle_request(*input.request, state.usable, base_command_usable);
    }
}

core::ModeResult MotionRuntime::handle_request(
    const core::ModeRequest& request,
    const bool state_usable,
    const bool base_command_usable)
{
    // 正在执行的活动请求：相同编号视为重试，返回实时结果，不重复执行。
    if (has_active_request_ && request.request_id == active_request_id_)
    {
        return active_result_;
    }
    // 已终结的请求：相同编号重试或旧编号查询都返回保存的终态。
    core::ModeResult stored{};
    if (find_terminal_result(request.request_id, stored))
    {
        return stored;
    }
    if (request.request_id <= latest_request_id_)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "stale request id");
    }

    // 编号更大的新请求：被拒绝的请求只记录自身终态，不覆盖活动请求的结果。
    const core::ModeResult result = dispatch_request(request, state_usable, base_command_usable);
    if (request.request_id > latest_request_id_)
    {
        latest_request_id_ = request.request_id;
    }
    if (result.state != core::ModeResultState::Accepted)
    {
        store_terminal_result(result);
        return result;
    }
    active_request_id_ = request.request_id;
    active_request_type_ = request.type;
    active_result_ = result;
    has_active_request_ = true;
    return result;
}

core::ModeResult MotionRuntime::dispatch_request(
    const core::ModeRequest& request,
    const bool state_usable,
    const bool base_command_usable)
{
    switch (request.type)
    {
    case core::ModeRequestType::EnterPassive:
        return dispatch_enter_passive(request);
    case core::ModeRequestType::GetUp:
        return dispatch_getup(request, state_usable);
    case core::ModeRequestType::Stand:
        return dispatch_stand(request);
    case core::ModeRequestType::StartBehavior:
        return dispatch_start_behavior(request, state_usable, base_command_usable);
    case core::ModeRequestType::GetDown:
        return dispatch_getdown(request, state_usable);
    case core::ModeRequestType::SwitchPolicy:
        return dispatch_switch_policy(request, state_usable);
    case core::ModeRequestType::ResetFault:
        return dispatch_reset_fault(request);
    }
    return make_result(request.request_id, core::ModeResultState::Rejected,
        "unknown request type");
}

core::ModeResult MotionRuntime::dispatch_enter_passive(const core::ModeRequest& request)
{
    // 可从任意模式立即中止主动动作；被中止的活动请求先获得 Failed 终态。
    if (has_active_request_)
    {
        abort_active_request("interrupted by enter passive");
    }
    mode_ = core::MotionMode::Passive;
    status_.active_source = core::CommandSource::None;
    status_.behavior_name.clear();
    status_.behavior_phase.clear();
    rl_control_cycle_ = 0;
    rl_command_ = {};
    pending_policy_index_ = kInvalidPolicyIndex;
    policy_transition_active_ = false;
    if (rl_controller_ != nullptr)
    {
        rl_controller_->reset();
    }
    return make_result(request.request_id, core::ModeResultState::Completed,
        "entered passive");
}

core::ModeResult MotionRuntime::dispatch_getup(
    const core::ModeRequest& request,
    const bool state_usable)
{
    if (mode_ == core::MotionMode::GetUp)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "getup already in progress");
    }
    if (mode_ == core::MotionMode::Stand)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "already standing");
    }
    if (mode_ == core::MotionMode::Running)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "getup is not valid while a behavior is running");
    }
    if (!state_usable)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "no valid state");
    }
    if (std::string reason; !check_active_preconditions(reason))
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            reason.c_str());
    }
    return accept_getup(request.request_id);
}

core::ModeResult MotionRuntime::dispatch_stand(const core::ModeRequest& request)
{
    // M2 不允许跳过起立进入 Stand，已在 Stand 时也不重新执行动作。
    if (mode_ == core::MotionMode::Stand)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "already standing");
    }
    if (mode_ == core::MotionMode::Passive)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "stand requires completing getup first");
    }
    return make_result(request.request_id, core::ModeResultState::Rejected,
        "another motion is in progress");
}

core::ModeResult MotionRuntime::dispatch_getdown(
    const core::ModeRequest& request,
    const bool state_usable)
{
    if (mode_ == core::MotionMode::GetDown)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "getdown already in progress");
    }
    if (!has_rest_pose_)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "no recorded rest pose");
    }
    if (!state_usable)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "no valid state");
    }
    if (std::string reason; !check_active_preconditions(reason))
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            reason.c_str());
    }
    return accept_getdown(request.request_id);
}

core::ModeResult MotionRuntime::dispatch_start_behavior(
    const core::ModeRequest& request,
    const bool state_usable,
    const bool base_command_usable)
{
    if (request.behavior_name != "rl_locomotion")
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "unknown behavior");
    }
    if (rl_controller_ == nullptr || policy_ == nullptr)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "RL policy is not attached");
    }
    if (mode_ != core::MotionMode::Stand)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "RL behavior requires Stand mode");
    }
    if (!state_usable)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "no valid state");
    }
    if (!base_command_usable)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "rl_locomotion requires a valid BaseCommand");
    }
    if (std::string reason; !check_active_preconditions(reason))
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            reason.c_str());
    }

    rl_controller_->reset();
    rl_control_cycle_ = 0;
    rl_command_ = {};
    mode_ = core::MotionMode::Running;
    status_.behavior_name = request.behavior_name;
    status_.behavior_phase = "starting";
    status_.policy_name = policy_name_;
    status_.policy_ready = true;
    return make_result(request.request_id, core::ModeResultState::Accepted,
        "RL behavior accepted");
}

core::ModeResult MotionRuntime::dispatch_switch_policy(
    const core::ModeRequest& request,
    const bool state_usable)
{
    if (mode_ != core::MotionMode::Running ||
        status_.behavior_name != "rl_locomotion")
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "policy switching requires running rl_locomotion");
    }
    if (!state_usable)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "no valid state");
    }
    if (std::string reason; !check_active_preconditions(reason))
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            reason.c_str());
    }

    const std::size_t target_index = find_policy(request.policy_name);
    if (target_index == kInvalidPolicyIndex)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "policy is not registered");
    }

    auto& target = policies_[target_index];
    if (target.policy == nullptr || target.controller == nullptr)
    {
        return make_result(request.request_id, core::ModeResultState::Rejected,
            "policy is not loaded");
    }
    if (has_active_request_)
    {
        abort_active_request("interrupted by policy switch");
    }

    rl_control_cycle_ = 0;
    rl_command_ = {};
    if (rl_controller_ != nullptr)
    {
        rl_controller_->reset();
    }
    status_.active_source = core::CommandSource::None;

    if (pose_close_to_policy(target.config, current_positions_))
    {
        activate_policy(target_index);
        status_.behavior_phase = "starting";
        return make_result(request.request_id, core::ModeResultState::Accepted,
            "policy reload accepted");
    }

    pending_policy_index_ = target_index;
    policy_transition_active_ = true;
    interp_start_ = current_positions_;
    interp_target_ = current_positions_;
    for (std::size_t i = 0; i < kRlJointCount; ++i)
    {
        interp_target_[i] = target.config.default_joint_positions[i];
    }
    interp_total_cycles_ = kPolicyTransitionCycles;
    interp_elapsed_cycles_ = 0;
    status_.behavior_phase = "policy_transition";
    return make_result(request.request_id, core::ModeResultState::Accepted,
        "policy transition accepted");
}

core::ModeResult MotionRuntime::dispatch_reset_fault(const core::ModeRequest& request)
{
    // RobotIO 当前没有 reset_fault() 边界，不能在 MotionRuntime 内伪造故障复位。
    return make_result(request.request_id, core::ModeResultState::Rejected,
        "fault reset is not supported by RobotIO");
}

core::ModeResult MotionRuntime::accept_getup(const std::uint64_t request_id)
{
    // 首次从 Passive 起立时完整记录当前落地姿态；GetDown 中断再起立时保留原姿态。
    if (!has_rest_pose_)
    {
        rest_pose_ = current_positions_;
        has_rest_pose_ = true;
    }
    // GetDown 被 GetUp 中断时，被中止的趴下请求获得明确终态。
    if (has_active_request_)
    {
        abort_active_request("interrupted by getup");
    }

    interp_start_ = current_positions_;
    interp_target_ = config_.pre_getup_position;
    interp_total_cycles_ = config_.getup_pre_cycles;
    interp_elapsed_cycles_ = 0;
    getup_second_phase_ = false;
    mode_ = core::MotionMode::GetUp;
    status_.active_source = core::CommandSource::None;
    status_.behavior_name.clear();
    status_.behavior_phase = "interpolating";
    return make_result(request_id, core::ModeResultState::Accepted, "getup accepted");
}

core::ModeResult MotionRuntime::accept_getdown(const std::uint64_t request_id)
{
    if (has_active_request_)
    {
        abort_active_request("interrupted by getdown");
    }
    status_.active_source = core::CommandSource::None;
    status_.behavior_name.clear();
    status_.behavior_phase = "interpolating";
    interp_start_ = current_positions_;
    interp_target_ = rest_pose_;
    interp_total_cycles_ = config_.getdown_cycles;
    interp_elapsed_cycles_ = 0;
    mode_ = core::MotionMode::GetDown;
    return make_result(request_id, core::ModeResultState::Accepted, "getdown accepted");
}

bool MotionRuntime::advance_active_motion(
    std::array<double, core::kMaxJoints>& positions)
{
    if (mode_ == core::MotionMode::Stand)
    {
        positions = config_.stand_position;
        return true;
    }

    if (interp_elapsed_cycles_ < interp_total_cycles_)
    {
        // 只有状态读取、命令生成条件满足的周期才推进插值周期。
        ++interp_elapsed_cycles_;
        const double percent = static_cast<double>(interp_elapsed_cycles_) /
            static_cast<double>(interp_total_cycles_);
        for (std::size_t i = 0; i < model_.joint_count; ++i)
        {
            positions[i] = interpolate(interp_start_[i], interp_target_[i], percent);
        }
        return true;
    }

    if (mode_ == core::MotionMode::GetUp && !getup_second_phase_)
    {
        // 第一段完成后从预起立姿态开始第二段，保证命令连续。
        interp_start_ = config_.pre_getup_position;
        interp_target_ = config_.stand_position;
        interp_total_cycles_ = config_.getup_cycles;
        interp_elapsed_cycles_ = 0;
        getup_second_phase_ = true;
        return advance_active_motion(positions);
    }

    if (mode_ == core::MotionMode::GetUp)
    {
        mode_ = core::MotionMode::Stand;
        status_.behavior_phase.clear();
        complete_active_request("getup completed");
        positions = config_.stand_position;
        return true;
    }

    // GetDown 完成：回到记录的落地姿态后进入 Passive，本周期起发送 Disabled。
    mode_ = core::MotionMode::Passive;
    status_.active_source = core::CommandSource::None;
    status_.behavior_name.clear();
    status_.behavior_phase.clear();
    complete_active_request("getdown completed");
    return false;
}

}  // 命名空间 quadruped::motion
