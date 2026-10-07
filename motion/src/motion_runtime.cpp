/**
 * @file motion_runtime.cpp
 * @brief 实现 MotionRuntime 的周期更新主流程、会话跟踪和命令提交。
 */

#include "quadruped/motion/motion_runtime.hpp"

#include "quadruped/core/validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

namespace quadruped::motion
{
namespace
{

constexpr std::uint32_t kAllowedMissedPolicyUpdates = 1;

core::Nanoseconds saturating_add(
    const core::Nanoseconds time_ns,
    const core::Nanoseconds duration_ns) noexcept
{
    return time_ns > INT64_MAX - duration_ns ? INT64_MAX : time_ns + duration_ns;
}

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

bool MotionRuntime::configure_retry(RetryConfig config, std::string& error_message)
{
    if (config.robot_name != model_.name || config.joint_count != model_.joint_count)
    {
        error_message = "Retry config does not match RobotModel";
        return false;
    }
    if (config.prepare_cycles == 0)
    {
        error_message = "Retry prepare_cycles must be positive";
        return false;
    }
    for (std::size_t i = 0; i < model_.joint_count; ++i)
    {
        const auto& joint = model_.joints[i];
        if (config.joint_names[i] != joint.name)
        {
            error_message = "Retry joint order mismatch at index " + std::to_string(i);
            return false;
        }
        const double position = config.target_positions[i];
        if (!std::isfinite(position) || !std::isfinite(config.kp[i]) ||
            !std::isfinite(config.kd[i]) || config.kp[i] < 0.0 || config.kd[i] < 0.0 ||
            config.kp[i] > joint.limits.max_kp || config.kd[i] > joint.limits.max_kd)
        {
            error_message = "Retry joint values are invalid at index " + std::to_string(i);
            return false;
        }
        if (joint.limits.position_limited &&
            (position < joint.limits.min_position || position > joint.limits.max_position))
        {
            error_message = "Retry target exceeds position limits at index " +
                std::to_string(i);
            return false;
        }
        if (joint.role == core::JointRole::Wheel && config.kp[i] != 0.0)
        {
            error_message = "Retry wheel KP must be zero at index " + std::to_string(i);
            return false;
        }
    }
    retry_config_ = std::move(config);
    retry_configured_ = true;
    error_message.clear();
    return true;
}

bool MotionRuntime::configure_fixed_drive(
    FixedDriveConfig config,
    std::string& error_message)
{
    if (config.behavior_name != "car_drive" &&
        config.behavior_name != "bridge_drive" &&
        config.behavior_name != "low_bar_drive")
    {
        error_message = "Fixed drive behavior name is invalid";
        return false;
    }
    if (config.robot_name != model_.name || config.joint_count != model_.joint_count ||
        config.prepare_cycles == 0 || config.exit_to_rl_cycles == 0)
    {
        error_message = "Fixed drive config does not match RobotModel";
        return false;
    }
    if (find_fixed_drive(config.behavior_name) != kInvalidFixedDriveIndex)
    {
        error_message = "Fixed drive behavior is already configured";
        return false;
    }
    if (!std::isfinite(config.max_x) || config.max_x <= 0.0 ||
        !std::isfinite(config.max_yaw) || config.max_yaw <= 0.0 ||
        !std::isfinite(config.wheel_velocity_scale) ||
        config.wheel_velocity_scale <= 0.0 ||
        !std::isfinite(config.yaw_to_wheel_velocity) ||
        config.yaw_to_wheel_velocity <= 0.0)
    {
        error_message = "Fixed drive limits and velocity scales are invalid";
        return false;
    }

    std::size_t wheel_count = 0;
    bool has_left = false;
    bool has_right = false;
    for (std::size_t i = 0; i < model_.joint_count; ++i)
    {
        const auto& joint = model_.joints[i];
        if (config.joint_names[i] != joint.name ||
            !std::isfinite(config.target_positions[i]) ||
            !std::isfinite(config.kp[i]) || !std::isfinite(config.kd[i]) ||
            config.kp[i] < 0.0 || config.kd[i] < 0.0 ||
            config.kp[i] > joint.limits.max_kp || config.kd[i] > joint.limits.max_kd)
        {
            error_message = "Fixed drive joint values are invalid at index " +
                std::to_string(i);
            return false;
        }
        if (joint.limits.position_limited &&
            (config.target_positions[i] < joint.limits.min_position ||
                config.target_positions[i] > joint.limits.max_position))
        {
            error_message = "Fixed drive pose exceeds limits at index " +
                std::to_string(i);
            return false;
        }
        if (joint.role == core::JointRole::Wheel)
        {
            if (config.kp[i] != 0.0 || wheel_count >= config.wheel_count ||
                !std::isfinite(config.wheel_velocity_sign[wheel_count]) ||
                config.wheel_velocity_sign[wheel_count] == 0.0 ||
                (config.wheel_sides[wheel_count] != WheelSide::Left &&
                    config.wheel_sides[wheel_count] != WheelSide::Right))
            {
                error_message = "Fixed drive Wheel mapping is invalid";
                return false;
            }
            has_left = has_left ||
                config.wheel_sides[wheel_count] == WheelSide::Left;
            has_right = has_right ||
                config.wheel_sides[wheel_count] == WheelSide::Right;
            ++wheel_count;
        }
    }
    if (wheel_count == 0 || config.wheel_count != wheel_count || !has_left || !has_right)
    {
        error_message = "Fixed drive Wheel count or side mapping is invalid";
        return false;
    }
    const double worst_velocity = config.wheel_velocity_scale * config.max_x +
        config.yaw_to_wheel_velocity * config.max_yaw;
    for (std::size_t i = 0; i < model_.joint_count; ++i)
    {
        if (model_.joints[i].role == core::JointRole::Wheel &&
            worst_velocity > model_.joints[i].limits.max_velocity)
        {
            error_message = "Fixed drive maximum Wheel velocity exceeds limits";
            return false;
        }
    }

    for (std::size_t i = 0; i < fixed_drive_configs_.size(); ++i)
    {
        if (!fixed_drive_configured_[i])
        {
            fixed_drive_configs_[i] = std::move(config);
            fixed_drive_configured_[i] = true;
            error_message.clear();
            return true;
        }
    }
    error_message = "Fixed drive registry is full";
    return false;
}

bool MotionRuntime::configure_event_chain(
    EventChainConfig config,
    std::string& error_message)
{
    if (config.robot_name != model_.name || config.joint_count != model_.joint_count)
    {
        error_message = "Event chain config does not match RobotModel";
        return false;
    }
    if (config.exit_to_rl_cycles == 0 ||
        (config.interpolation != "linear" && config.interpolation != "smoothstep"))
    {
        error_message = "Event chain timing or interpolation is invalid";
        return false;
    }
    if (config.event_count == 0 || config.event_count > config.events.size())
    {
        error_message = "Event chain must contain at least one event";
        return false;
    }
    std::size_t model_wheel_count = 0;
    for (std::size_t i = 0; i < model_.joint_count; ++i)
    {
        const auto& joint = model_.joints[i];
        if (config.joint_names[i] != joint.name)
        {
            error_message = "Event chain joint order mismatch at index " +
                std::to_string(i);
            return false;
        }
        if (!std::isfinite(config.kp[i]) || !std::isfinite(config.kd[i]) ||
            config.kp[i] < 0.0 || config.kd[i] < 0.0 ||
            config.kp[i] > joint.limits.max_kp || config.kd[i] > joint.limits.max_kd ||
            (joint.role == core::JointRole::Wheel && config.kp[i] != 0.0))
        {
            error_message = "Event chain gains are invalid at index " + std::to_string(i);
            return false;
        }
        if (joint.role == core::JointRole::Wheel)
        {
            ++model_wheel_count;
        }
    }
    if (config.wheel_count != model_wheel_count ||
        (model_wheel_count > 0 &&
            (!std::isfinite(config.wheel_radius) || config.wheel_radius <= 0.0)))
    {
        error_message = "Event chain wheel parameters do not match RobotModel";
        return false;
    }
    for (std::size_t i = 0; i < config.wheel_count; ++i)
    {
        if (!std::isfinite(config.wheel_velocity_sign[i]) ||
            config.wheel_velocity_sign[i] == 0.0)
        {
            error_message = "Event chain wheel velocity sign is invalid";
            return false;
        }
    }
    for (std::size_t i = 0; i < config.event_count; ++i)
    {
        const auto& event = config.events[i];
        if (event.type != EventType::Pose && event.type != EventType::Drive &&
            event.type != EventType::PoseDrive)
        {
            error_message = "Event chain event type is unknown";
            return false;
        }
        const bool uses_pose = event.type == EventType::Pose ||
            event.type == EventType::PoseDrive;
        const bool uses_wheel = event.type == EventType::Drive ||
            event.type == EventType::PoseDrive;
        if (event.name.empty())
        {
            error_message = "Event chain event name is empty";
            return false;
        }
        if (uses_wheel && model_wheel_count == 0)
        {
            error_message = "Event chain wheel event is not supported by this RobotModel";
            return false;
        }
        if (uses_wheel &&
            ((event.wheel_group != WheelGroup::Front &&
                 event.wheel_group != WheelGroup::Rear &&
                 event.wheel_group != WheelGroup::All) ||
                !std::isfinite(event.distance_m) || event.distance_m == 0.0 ||
                !std::isfinite(event.speed_mps) || event.speed_mps <= 0.0 ||
                event.timeout_cycles == 0))
        {
            error_message = "Event chain wheel event parameters are invalid";
            return false;
        }
        if (uses_pose)
        {
            if (event.transition_cycles == 0)
            {
                error_message = "Event chain pose transition must be positive";
                return false;
            }
            for (std::size_t joint_index = 0; joint_index < model_.joint_count; ++joint_index)
            {
                const double position = event.dof_positions[joint_index];
                const auto& limits = model_.joints[joint_index].limits;
                if (!std::isfinite(position) ||
                    (limits.position_limited &&
                        (position < limits.min_position || position > limits.max_position)))
                {
                    error_message = "Event chain pose is invalid at joint index " +
                        std::to_string(joint_index);
                    return false;
                }
            }
        }
        if (event.type == EventType::PoseDrive &&
            event.timeout_cycles < event.transition_cycles)
        {
            error_message = "Event chain pose-drive timeout is shorter than transition";
            return false;
        }
    }
    event_chain_config_ = std::move(config);
    event_chain_configured_ = true;
    error_message.clear();
    return true;
}

bool MotionRuntime::attach_policy(
    RlConfig config,
    Policy& policy,
    std::string& error_message)
{
    const std::string policy_name = config.name;
    if (!register_policy(std::move(config), policy, error_message))
    {
        return false;
    }
    current_policy_index_ = find_policy(policy_name);
    if (current_policy_index_ == kInvalidPolicyIndex)
    {
        error_message = "registered policy cannot be selected";
        return false;
    }
    activate_policy(current_policy_index_);
    return true;
}

bool MotionRuntime::set_policy_cycle(
    const std::vector<std::string>& policy_names,
    const std::uint32_t posture_transition_cycles,
    std::string& error_message)
{
    if (policy_names.empty())
    {
        error_message = "policy cycle must not be empty";
        return false;
    }
    if (policy_names.size() > kMaxPolicies)
    {
        error_message = "policy cycle exceeds runtime capacity";
        return false;
    }
    if (posture_transition_cycles == 0)
    {
        error_message = "policy transition cycles must be positive";
        return false;
    }

    std::array<std::string, kMaxPolicies> cycle{};
    for (std::size_t i = 0; i < policy_names.size(); ++i)
    {
        if (policy_names[i].empty() || find_policy(policy_names[i]) == kInvalidPolicyIndex)
        {
            error_message = "policy cycle contains an unregistered policy: " + policy_names[i];
            return false;
        }
        for (std::size_t previous = 0; previous < i; ++previous)
        {
            if (cycle[previous] == policy_names[i])
            {
                error_message = "policy cycle contains a duplicate policy: " + policy_names[i];
                return false;
            }
        }
        cycle[i] = policy_names[i];
    }

    policy_cycle_ = std::move(cycle);
    policy_cycle_size_ = policy_names.size();
    policy_transition_cycles_ = posture_transition_cycles;
    error_message.clear();
    return true;
}

std::string MotionRuntime::next_policy_name() const
{
    if (policy_cycle_size_ == 0)
    {
        return {};
    }
    std::size_t next_index = 0;
    for (std::size_t i = 0; i < policy_cycle_size_; ++i)
    {
        if (policy_cycle_[i] == policy_name_)
        {
            next_index = (i + 1) % policy_cycle_size_;
            break;
        }
    }
    return policy_cycle_[next_index];
}

bool MotionRuntime::register_policy(
    RlConfig config,
    Policy& policy,
    std::string& error_message)
{
    if (config.name.empty())
    {
        error_message = "policy name is empty";
        return false;
    }
    if (find_policy(config.name) != kInvalidPolicyIndex)
    {
        error_message = "policy is already registered: " + config.name;
        return false;
    }

    std::size_t free_index = kInvalidPolicyIndex;
    for (std::size_t i = 0; i < policies_.size(); ++i)
    {
        if (!policies_[i].registered)
        {
            free_index = i;
            break;
        }
    }
    if (free_index == kInvalidPolicyIndex)
    {
        error_message = "policy registry is full";
        return false;
    }

    auto created = RlController::create(model_, config);
    if (!created.ok())
    {
        error_message = created.error_message;
        return false;
    }

    auto& entry = policies_[free_index];
    entry.config = std::move(config);
    entry.policy = &policy;
    entry.controller = std::move(created.controller);
    entry.registered = true;
    error_message.clear();
    return true;
}

void MotionRuntime::activate_policy(const std::size_t policy_index)
{
    auto& entry = policies_[policy_index];
    current_policy_index_ = policy_index;
    pending_policy_index_ = kInvalidPolicyIndex;
    policy_transition_active_ = false;
    event_to_rl_transition_ = false;
    fixed_drive_to_rl_transition_ = false;
    active_fixed_drive_index_ = kInvalidFixedDriveIndex;
    retry_locked_ = false;
    rl_controller_ = entry.controller.get();
    policy_ = entry.policy;
    policy_name_ = entry.config.name;
    rl_control_cycle_ = 0;
    rl_command_ = {};
    rl_target_generated_at_ns_ = 0;
    rl_target_expires_at_ns_ = 0;
    latest_inference_elapsed_ns_ = 0;
    rl_controller_->reset();
    status_.policy_name = policy_name_;
    status_.policy_ready = true;
    status_.command_limits = entry.config.command_limits;
}

std::size_t MotionRuntime::find_policy(const std::string& name) const
{
    for (std::size_t i = 0; i < policies_.size(); ++i)
    {
        if (policies_[i].registered && policies_[i].config.name == name)
        {
            return i;
        }
    }
    return kInvalidPolicyIndex;
}

std::size_t MotionRuntime::find_fixed_drive(const std::string& name) const
{
    for (std::size_t i = 0; i < fixed_drive_configs_.size(); ++i)
    {
        if (fixed_drive_configured_[i] &&
            fixed_drive_configs_[i].behavior_name == name)
        {
            return i;
        }
    }
    return kInvalidFixedDriveIndex;
}

bool MotionRuntime::pose_close_to_policy(
    const RlConfig& config,
    const std::array<double, core::kMaxJoints>& positions) const
{
    for (std::size_t action_index = 0;
         action_index < config.action_dimension;
         ++action_index)
    {
        const std::size_t joint_index = config.policy_dof_indices[action_index];
        if (model_.joints[joint_index].role == core::JointRole::Wheel)
        {
            continue;
        }
        if (std::abs(positions[joint_index] -
                config.default_joint_positions[action_index]) >
            kPolicyPoseTolerance)
        {
            return false;
        }
    }
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
    active_request_type_ = core::ModeRequestType::EnterPassive;
    active_result_ = {};
    has_active_request_ = false;
    terminal_count_ = 0;
    terminal_next_ = 0;
    has_rest_pose_ = false;
    getup_second_phase_ = false;
    rl_control_cycle_ = 0;
    rl_command_ = {};
    rl_target_generated_at_ns_ = 0;
    rl_target_expires_at_ns_ = 0;
    latest_inference_elapsed_ns_ = 0;
    pending_policy_index_ = kInvalidPolicyIndex;
    policy_transition_active_ = false;
    event_to_rl_transition_ = false;
    fixed_drive_to_rl_transition_ = false;
    active_fixed_drive_index_ = kInvalidFixedDriveIndex;
    event_initialized_ = false;
    event_motion_complete_ = false;
    event_chain_complete_ = false;
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
    pending_policy_index_ = kInvalidPolicyIndex;
    policy_transition_active_ = false;
    event_to_rl_transition_ = false;
    fixed_drive_to_rl_transition_ = false;
    active_fixed_drive_index_ = kInvalidFixedDriveIndex;
    retry_locked_ = false;
    rl_control_cycle_ = 0;
    rl_command_ = {};
    rl_target_generated_at_ns_ = 0;
    rl_target_expires_at_ns_ = 0;
    if (rl_controller_ != nullptr)
    {
        rl_controller_->reset();
    }
    mode_ = core::MotionMode::Passive;
    status_.active_source = core::CommandSource::None;
    status_.behavior_name.clear();
    status_.behavior_phase.clear();
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
    const core::Nanoseconds frame_now = io.clock_now_ns();
    if (frame_now < 0)
    {
        return core::RobotIOCode::InvalidFrame;
    }
    const core::Nanoseconds nominal_expiry =
        saturating_add(frame_now, config_.command_validity_ns);
    const bool has_rl_target = submission.target_expires_at_ns > 0;
    if (has_rl_target &&
        (submission.target_generated_at_ns < 0 ||
            submission.target_generated_at_ns > frame_now ||
            submission.target_expires_at_ns <= frame_now))
    {
        return core::RobotIOCode::InvalidFrame;
    }
    command.header.sequence = ++command_sequence_;
    command.header.timestamp_ns = frame_now;
    // 单调时间加有效期在极端输入下可能溢出；溢出时按最长可表示时间处理。
    command.expires_at_ns = has_rl_target
        ? std::min(nominal_expiry, submission.target_expires_at_ns)
        : nominal_expiry;
    command.target_generated_at_ns =
        has_rl_target ? submission.target_generated_at_ns : frame_now;
    command.target_expires_at_ns =
        has_rl_target ? submission.target_expires_at_ns : command.expires_at_ns;
    if (command.expires_at_ns <= frame_now)
    {
        return core::RobotIOCode::InvalidFrame;
    }
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
        const bool is_wheel = model_.joints[i].role == core::JointRole::Wheel;
        const auto& limits = model_.joints[i].limits;
        joint.target_position = is_wheel
            ? current_positions_[i]
            : (limits.position_limited
                      ? std::clamp(
                            submission.positions[i], limits.min_position, limits.max_position)
                      : submission.positions[i]);
        joint.target_velocity = submission.velocities == nullptr ? 0.0
                                                                  : (*submission.velocities)[i];
        joint.kp = is_wheel ? 0.0
                            : (submission.kp == nullptr ? config_.fixed_kp[i]
                                                        : (*submission.kp)[i]);
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
        status_.error_message = result.failure_reason;
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
        output.submit_code = submit_command(io, {current_positions_, false});
        output.submitted = true;
        return true;
    }

    if (has_active_request_ && active_result_.state == core::ModeResultState::Accepted)
    {
        active_result_.state = core::ModeResultState::Running;
    }
    std::array<double, core::kMaxJoints> positions{};
    const bool use_impedance = advance_active_motion(positions);
    output.submit_code = submit_command(io, {positions, use_impedance});
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
            output.submit_code = submit_command(io, {current_positions_, false});
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
    if (policy_transition_active_)
    {
        return run_policy_transition(io, state, output);
    }
    if (rl_controller_ == nullptr || policy_ == nullptr)
    {
        return fail("RL policy is not attached");
    }

    if (has_active_request_ && active_result_.state == core::ModeResultState::Accepted)
    {
        active_result_.state = core::ModeResultState::Running;
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
        if (!rl_controller_->insert_observation(observation))
        {
            return fail("RL observation history insertion failed");
        }
        const auto inference = policy_->forward(rl_controller_->inference_input());
        latest_inference_elapsed_ns_ = inference.elapsed_ns;
        if (!inference.ok)
        {
            return fail("RL inference failed: " + inference.error_message);
        }
        if (inference.elapsed_ns > kRlInferenceDeadlineNs)
        {
            return fail("RL inference exceeded deadline: " +
                std::to_string(inference.elapsed_ns / 1'000'000) + " ms");
        }
        rl_command_ = rl_controller_->convert_actions(inference, current_positions_);
        if (!rl_command_.ok)
        {
            return fail("RL action conversion failed: " + rl_command_.error_message);
        }
        const core::Nanoseconds target_now = io.clock_now_ns();
        constexpr core::Nanoseconds kLifetimePeriods =
            static_cast<core::Nanoseconds>(kRlDecimation) *
            (1 + kAllowedMissedPolicyUpdates);
        const core::Nanoseconds lifetime_ns =
            config_.control_period_ns > INT64_MAX / kLifetimePeriods
            ? INT64_MAX
            : config_.control_period_ns * kLifetimePeriods;
        if (target_now < 0 || lifetime_ns <= 0 ||
            saturating_add(target_now, lifetime_ns) <= target_now)
        {
            return fail("RL target timing is invalid");
        }
        rl_target_generated_at_ns_ = target_now;
        rl_target_expires_at_ns_ = saturating_add(target_now, lifetime_ns);
        if (has_active_request_)
        {
            const char* message = active_request_type_ == core::ModeRequestType::SwitchPolicy
                ? "policy switched"
                : "behavior started";
            complete_active_request(message);
        }
    }
    ++rl_control_cycle_;

    if (io.clock_now_ns() >= rl_target_expires_at_ns_)
    {
        return fail("RL target hard expiry reached");
    }
    status_.active_source = rl_controller_->active_command_source();
    status_.behavior_phase = "driving";
    output.submit_code = submit_command(io,
        {rl_command_.target_positions,
            true,
            &rl_command_.target_velocities,
            &rl_command_.kp,
            &rl_command_.kd,
            status_.active_source,
            rl_target_generated_at_ns_,
            rl_target_expires_at_ns_});
    output.submitted = true;
    if (output.submit_code != core::RobotIOCode::Ok)
    {
        return fail("RL command submit failed");
    }
    return false;
}

bool MotionRuntime::run_retry_mode(
    core::RobotIO& io,
    const StateRead& state,
    MotionUpdateOutput& output)
{
    if (!state.usable)
    {
        fail_active_motion(state.failure_reason);
        return true;
    }
    if (std::string reason; !check_active_preconditions(reason))
    {
        fail_active_motion(reason);
        output.submit_code = submit_command(io, {current_positions_, false});
        output.submitted = true;
        return true;
    }

    if (has_active_request_ && active_result_.state == core::ModeResultState::Accepted)
    {
        active_result_.state = core::ModeResultState::Running;
    }
    std::array<double, core::kMaxJoints> positions = retry_config_.target_positions;
    if (!retry_locked_)
    {
        ++interp_elapsed_cycles_;
        const double percent = static_cast<double>(interp_elapsed_cycles_) /
            static_cast<double>(interp_total_cycles_);
        for (std::size_t i = 0; i < model_.joint_count; ++i)
        {
            positions[i] = (1.0 - percent) * interp_start_[i] +
                percent * retry_config_.target_positions[i];
        }
        if (interp_elapsed_cycles_ >= interp_total_cycles_)
        {
            retry_locked_ = true;
            status_.behavior_phase = "locked";
            complete_active_request("retry locked");
        }
    }

    std::array<double, core::kMaxJoints> velocities{};
    status_.active_source = core::CommandSource::None;
    output.submit_code = submit_command(io,
        {positions,
            true,
            &velocities,
            &retry_config_.kp,
            &retry_config_.kd,
            core::CommandSource::None});
    output.submitted = true;
    if (output.submit_code != core::RobotIOCode::Ok)
    {
        fail_active_motion("Retry command submit failed");
        return true;
    }
    return false;
}

bool MotionRuntime::run_fixed_drive_mode(
    core::RobotIO& io,
    const StateRead& state,
    const core::BaseCommand* const base_command,
    MotionUpdateOutput& output)
{
    const auto fail = [&](const std::string& reason) {
        fail_active_motion(reason);
        if (state.usable)
        {
            output.submit_code = submit_command(io, {current_positions_, false});
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
    if (active_fixed_drive_index_ == kInvalidFixedDriveIndex ||
        !fixed_drive_configured_[active_fixed_drive_index_])
    {
        return fail("Fixed drive runtime state is invalid");
    }
    if (has_active_request_ && active_result_.state == core::ModeResultState::Accepted)
    {
        active_result_.state = core::ModeResultState::Running;
    }

    const auto& fixed = fixed_drive_configs_[active_fixed_drive_index_];
    std::array<double, core::kMaxJoints> positions = fixed.target_positions;
    if (interp_elapsed_cycles_ < interp_total_cycles_)
    {
        ++interp_elapsed_cycles_;
        const double percent = static_cast<double>(interp_elapsed_cycles_) /
            static_cast<double>(interp_total_cycles_);
        for (std::size_t i = 0; i < model_.joint_count; ++i)
        {
            if (model_.joints[i].role != core::JointRole::Wheel)
            {
                positions[i] = (1.0 - percent) * interp_start_[i] +
                    percent * fixed.target_positions[i];
            }
        }
        status_.behavior_phase = interp_elapsed_cycles_ >= interp_total_cycles_
            ? "driving"
            : "preparing";
        if (interp_elapsed_cycles_ >= interp_total_cycles_)
        {
            complete_active_request("fixed drive pose ready");
        }
    }
    else
    {
        status_.behavior_phase = "driving";
    }

    const bool command_usable = base_command != nullptr &&
        core::validate(*base_command, state.now_ns).ok();
    const double x = command_usable
        ? std::clamp(base_command->vx, -fixed.max_x, fixed.max_x)
        : 0.0;
    const double yaw = command_usable
        ? std::clamp(base_command->wz, -fixed.max_yaw, fixed.max_yaw)
        : 0.0;
    const double left_velocity =
        fixed.wheel_velocity_scale * x - fixed.yaw_to_wheel_velocity * yaw;
    const double right_velocity =
        fixed.wheel_velocity_scale * x + fixed.yaw_to_wheel_velocity * yaw;
    std::array<double, core::kMaxJoints> velocities{};
    std::size_t wheel_id = 0;
    for (std::size_t i = 0; i < model_.joint_count; ++i)
    {
        if (model_.joints[i].role != core::JointRole::Wheel)
        {
            continue;
        }
        positions[i] = current_positions_[i];
        const double side_velocity = fixed.wheel_sides[wheel_id] == WheelSide::Left
            ? left_velocity
            : right_velocity;
        velocities[i] = fixed.wheel_velocity_sign[wheel_id] * side_velocity;
        ++wheel_id;
    }

    status_.active_source = command_usable
        ? base_command->source
        : core::CommandSource::None;
    output.submit_code = submit_command(io,
        {positions,
            true,
            &velocities,
            &fixed.kp,
            &fixed.kd,
            status_.active_source});
    output.submitted = true;
    if (output.submit_code != core::RobotIOCode::Ok)
    {
        return fail("Fixed drive command submit failed");
    }
    return false;
}

bool MotionRuntime::run_event_chain_mode(
    core::RobotIO& io,
    const StateRead& state,
    MotionUpdateOutput& output)
{
    const auto fail = [&](const std::string& reason) {
        fail_active_motion(reason);
        if (state.usable)
        {
            output.submit_code = submit_command(io, {current_positions_, false});
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
    if (!event_chain_configured_ || event_index_ >= event_chain_config_.event_count)
    {
        return fail("Event chain runtime state is invalid");
    }
    if (has_active_request_ && active_result_.state == core::ModeResultState::Accepted)
    {
        active_result_.state = core::ModeResultState::Running;
    }

    std::array<double, core::kMaxJoints> positions = event_active_pose_;
    std::array<double, core::kMaxJoints> velocities{};
    for (std::size_t i = 0; i < model_.joint_count; ++i)
    {
        if (model_.joints[i].role == core::JointRole::Wheel)
        {
            positions[i] = current_positions_[i];
        }
    }

    if (event_chain_complete_)
    {
        status_.behavior_phase = "completed";
    }
    else
    {
        const auto& event = event_chain_config_.events[event_index_];
        status_.behavior_phase = event.name;
        if (!event_initialized_)
        {
            event_initialized_ = true;
            event_motion_complete_ = false;
            event_cycle_ = 0;
            event_hold_cycle_ = 0;
            event_segment_start_ = event_active_pose_;
            std::size_t wheel_id = 0;
            for (std::size_t i = 0; i < model_.joint_count; ++i)
            {
                if (model_.joints[i].role == core::JointRole::Wheel)
                {
                    event_drive_start_[wheel_id] = current_positions_[i];
                    ++wheel_id;
                }
            }
        }

        const auto wheel_selected = [&](const std::size_t wheel_id) {
            const std::size_t front_count = event_chain_config_.wheel_count / 2;
            return event.wheel_group == WheelGroup::All ||
                (event.wheel_group == WheelGroup::Front && wheel_id < front_count) ||
                (event.wheel_group == WheelGroup::Rear && wheel_id >= front_count);
        };
        const auto drive_distance = [&]() {
            double distance_sum = 0.0;
            std::size_t selected_count = 0;
            std::size_t wheel_id = 0;
            for (std::size_t i = 0; i < model_.joint_count; ++i)
            {
                if (model_.joints[i].role != core::JointRole::Wheel)
                {
                    continue;
                }
                if (wheel_selected(wheel_id))
                {
                    const double direction =
                        event_chain_config_.wheel_velocity_sign[wheel_id] > 0.0 ? 1.0 : -1.0;
                    distance_sum += direction *
                        (current_positions_[i] - event_drive_start_[wheel_id]) *
                        event_chain_config_.wheel_radius;
                    ++selected_count;
                }
                ++wheel_id;
            }
            return selected_count == 0
                ? 0.0
                : distance_sum / static_cast<double>(selected_count);
        };
        const auto apply_wheel_drive = [&]() {
            const double travel_direction = event.distance_m > 0.0 ? 1.0 : -1.0;
            const double wheel_speed = travel_direction * event.speed_mps /
                event_chain_config_.wheel_radius;
            std::size_t wheel_id = 0;
            for (std::size_t i = 0; i < model_.joint_count; ++i)
            {
                if (model_.joints[i].role != core::JointRole::Wheel)
                {
                    continue;
                }
                if (wheel_selected(wheel_id))
                {
                    const double direction =
                        event_chain_config_.wheel_velocity_sign[wheel_id] > 0.0 ? 1.0 : -1.0;
                    velocities[i] = direction * wheel_speed;
                }
                ++wheel_id;
            }
        };
        const auto apply_pose = [&](const double alpha) {
            for (std::size_t i = 0; i < model_.joint_count; ++i)
            {
                if (model_.joints[i].role != core::JointRole::Wheel)
                {
                    positions[i] = (1.0 - alpha) * event_segment_start_[i] +
                        alpha * event.dof_positions[i];
                }
            }
        };
        const auto target_reached = [&](const double distance) {
            return event.distance_m > 0.0
                ? distance >= event.distance_m
                : distance <= event.distance_m;
        };

        if (event.type == EventType::Pose && !event_motion_complete_)
        {
            ++event_cycle_;
            double alpha = static_cast<double>(event_cycle_) /
                static_cast<double>(event.transition_cycles);
            alpha = std::min(alpha, 1.0);
            if (event_chain_config_.interpolation == "smoothstep")
            {
                alpha = alpha * alpha * (3.0 - 2.0 * alpha);
            }
            apply_pose(alpha);
            if (event_cycle_ >= event.transition_cycles)
            {
                event_active_pose_ = event.dof_positions;
                event_motion_complete_ = true;
            }
        }
        else if (event.type == EventType::Drive && !event_motion_complete_)
        {
            const double distance = drive_distance();
            if (target_reached(distance))
            {
                event_motion_complete_ = true;
            }
            else if (event_cycle_ >= event.timeout_cycles)
            {
                return fail("Event chain drive timed out: " + event.name);
            }
            else
            {
                apply_wheel_drive();
                ++event_cycle_;
            }
        }
        else if (event.type == EventType::PoseDrive && !event_motion_complete_)
        {
            const double distance = drive_distance();
            const bool drive_complete = target_reached(distance);
            const bool pose_complete_before_cycle =
                event_cycle_ >= event.transition_cycles;
            if ((!drive_complete || !pose_complete_before_cycle) &&
                event_cycle_ >= event.timeout_cycles)
            {
                return fail("Event chain pose-drive timed out: " + event.name);
            }

            ++event_cycle_;
            double alpha = std::min(
                static_cast<double>(event_cycle_) /
                    static_cast<double>(event.transition_cycles),
                1.0);
            if (event_chain_config_.interpolation == "smoothstep")
            {
                alpha = alpha * alpha * (3.0 - 2.0 * alpha);
            }
            apply_pose(alpha);
            if (event_cycle_ >= event.transition_cycles)
            {
                event_active_pose_ = event.dof_positions;
            }
            if (!drive_complete)
            {
                apply_wheel_drive();
            }
            event_motion_complete_ =
                event_cycle_ >= event.transition_cycles && drive_complete;
        }

        if (event_motion_complete_)
        {
            positions = event_active_pose_;
            for (std::size_t i = 0; i < model_.joint_count; ++i)
            {
                if (model_.joints[i].role == core::JointRole::Wheel)
                {
                    positions[i] = current_positions_[i];
                }
            }
            if (event_hold_cycle_ < event.hold_cycles)
            {
                ++event_hold_cycle_;
            }
            else if (event_index_ + 1 >= event_chain_config_.event_count)
            {
                event_chain_complete_ = true;
                status_.behavior_phase = "completed";
                complete_active_request("Event chain completed");
            }
            else
            {
                ++event_index_;
                event_initialized_ = false;
                event_motion_complete_ = false;
                event_cycle_ = 0;
                event_hold_cycle_ = 0;
            }
        }
    }

    status_.active_source = core::CommandSource::None;
    output.submit_code = submit_command(io,
        {positions,
            true,
            &velocities,
            &event_chain_config_.kp,
            &event_chain_config_.kd,
            core::CommandSource::None});
    output.submitted = true;
    if (output.submit_code != core::RobotIOCode::Ok)
    {
        return fail("Event chain command submit failed");
    }
    return false;
}

bool MotionRuntime::run_policy_transition(
    core::RobotIO& io,
    const StateRead&,
    MotionUpdateOutput& output)
{
    if (pending_policy_index_ == kInvalidPolicyIndex ||
        !policies_[pending_policy_index_].registered)
    {
        fail_active_motion("policy transition target is unavailable");
        output.submit_code = submit_command(io, {current_positions_, false});
        output.submitted = true;
        return true;
    }

    if (has_active_request_ && active_result_.state == core::ModeResultState::Accepted)
    {
        active_result_.state = core::ModeResultState::Running;
    }

    std::array<double, core::kMaxJoints> positions = interp_target_;
    if (interp_elapsed_cycles_ < interp_total_cycles_)
    {
        ++interp_elapsed_cycles_;
        double percent = static_cast<double>(interp_elapsed_cycles_) /
            static_cast<double>(interp_total_cycles_);
        if (event_to_rl_transition_ && event_chain_config_.interpolation == "smoothstep")
        {
            percent = percent * percent * (3.0 - 2.0 * percent);
        }
        for (std::size_t i = 0; i < model_.joint_count; ++i)
        {
            positions[i] = (1.0 - percent) * interp_start_[i] +
                percent * interp_target_[i];
        }
    }

    status_.active_source = core::CommandSource::None;
    status_.behavior_phase = "policy_transition";
    const std::array<double, core::kMaxJoints>* kp = nullptr;
    const std::array<double, core::kMaxJoints>* kd = nullptr;
    if (event_to_rl_transition_)
    {
        kp = &event_chain_config_.kp;
        kd = &event_chain_config_.kd;
    }
    else if (fixed_drive_to_rl_transition_ &&
        active_fixed_drive_index_ != kInvalidFixedDriveIndex)
    {
        kp = &fixed_drive_configs_[active_fixed_drive_index_].kp;
        kd = &fixed_drive_configs_[active_fixed_drive_index_].kd;
    }
    output.submit_code = submit_command(io,
        {positions, true, nullptr, kp, kd, core::CommandSource::None});
    output.submitted = true;
    if (output.submit_code != core::RobotIOCode::Ok)
    {
        fail_active_motion("policy transition command submit failed");
        return true;
    }

    if (interp_elapsed_cycles_ >= interp_total_cycles_)
    {
        const std::size_t target_index = pending_policy_index_;
        activate_policy(target_index);
        status_.behavior_phase = "starting";
        const char* message = active_request_type_ == core::ModeRequestType::SwitchPolicy
            ? "policy switched"
            : "behavior started";
        complete_active_request(message);
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
    const bool base_command_usable = input.base_command != nullptr &&
        core::validate(*input.base_command, input.now_ns).ok();
    handle_input_request(input, state, base_command_usable, output);

    // 本周期是否发生了主动动作失败；失败周期的错误说明不能被随后的成功提交冲掉。
    bool failed_this_cycle = false;
    if (mode_ == core::MotionMode::Passive)
    {
        if (state.usable)
        {
            output.submit_code = submit_command(io, {current_positions_, false});
            output.submitted = true;
        }
    }
    else if (mode_ == core::MotionMode::Running)
    {
        if (status_.behavior_name == "retry")
        {
            failed_this_cycle = run_retry_mode(io, state, output);
        }
        else if (status_.behavior_name == "event_chain")
        {
            failed_this_cycle = run_event_chain_mode(io, state, output);
        }
        else if (find_fixed_drive(status_.behavior_name) != kInvalidFixedDriveIndex)
        {
            failed_this_cycle =
                run_fixed_drive_mode(io, state, input.base_command, output);
        }
        else
        {
            failed_this_cycle = run_rl_mode(io, state, input.base_command, output);
        }
    }
    else
    {
        failed_this_cycle = run_active_mode(io, state, output);
    }

    if (mode_ != core::MotionMode::Passive && !failed_this_cycle && output.submitted &&
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
    const core::RobotIOStatus io_status = io.status();
    output.diagnostics.startup_id = startup_id_;
    output.diagnostics.session_id = session_id_;
    output.diagnostics.command_sequence = command_sequence_;
    output.diagnostics.dropped_state_frames = io_status.dropped_state_frames;
    output.diagnostics.rejected_command_frames = io_status.rejected_command_frames;
    output.diagnostics.inference_elapsed_ns = latest_inference_elapsed_ns_;
    if (state.usable)
    {
        output.diagnostics.state_sequence = state.frame.header.sequence;
        output.diagnostics.effective_command_sequence =
            state.frame.effective_command_sequence;
        output.diagnostics.state_age_ns = input.now_ns - state.frame.header.timestamp_ns;
    }
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
