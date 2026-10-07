/**
 * @file validation.cpp
 * @brief 实现机器人模型、状态帧和控制命令的合法性检查。
 */

#include "quadruped/core/validation.hpp"

#include <cmath>
#include <string>
#include <unordered_set>
#include <utility>

namespace quadruped::core
{
namespace
{

ValidationResult failure(
    ValidationError error,
    std::string message,
    std::size_t joint_index = kMaxJoints)
{
    return ValidationResult{error, joint_index, std::move(message)};
}

bool is_valid(JointRole role)
{
    switch (role)
    {
    case JointRole::Leg:
    case JointRole::Wheel:
    case JointRole::Auxiliary:
        return true;
    }
    return false;
}

bool is_valid(ControlMode mode)
{
    switch (mode)
    {
    case ControlMode::Disabled:
    case ControlMode::Damping:
    case ControlMode::JointImpedance:
    case ControlMode::Velocity:
    case ControlMode::Torque:
        return true;
    }
    return false;
}

bool is_valid(CommandSource source)
{
    switch (source)
    {
    case CommandSource::None:
    case CommandSource::Gamepad:
    case CommandSource::Navigation:
    case CommandSource::Remote:
    case CommandSource::Test:
        return true;
    }
    return false;
}

bool is_valid(MotionMode mode)
{
    switch (mode)
    {
    case MotionMode::Passive:
    case MotionMode::GetUp:
    case MotionMode::Stand:
    case MotionMode::Running:
    case MotionMode::GetDown:
        return true;
    }
    return false;
}

bool is_valid(ModeRequestType type)
{
    switch (type)
    {
    case ModeRequestType::EnterPassive:
    case ModeRequestType::GetUp:
    case ModeRequestType::Stand:
    case ModeRequestType::StartBehavior:
    case ModeRequestType::GetDown:
    case ModeRequestType::SwitchPolicy:
    case ModeRequestType::ResetFault:
        return true;
    }
    return false;
}

bool is_finite(double value)
{
    return std::isfinite(value);
}

ValidationResult validate_header(const FrameHeader& header, Nanoseconds now_ns)
{
    if (header.schema_version != kFrameSchemaVersion)
    {
        return failure(ValidationError::SchemaMismatch, "frame schema version does not match");
    }
    if (header.timestamp_ns < 0 || now_ns < 0 || header.timestamp_ns > now_ns)
    {
        return failure(
            ValidationError::InvalidTimestamp,
            "frame timestamp is invalid or in the future");
    }
    return {};
}

ValidationResult validate_joint_count(std::size_t actual, const RobotModel& model)
{
    if (actual == 0 || actual > kMaxJoints || actual != model.joint_count)
    {
        return failure(
            ValidationError::InvalidJointCount,
            "frame joint count does not match RobotModel");
    }
    return {};
}

}  // 匿名命名空间

ValidationResult validate(const RobotModel& model)
{
    if (model.name.empty())
    {
        return failure(ValidationError::InvalidName, "robot name must not be empty");
    }
    if (model.joint_count == 0 || model.joint_count > kMaxJoints)
    {
        return failure(ValidationError::InvalidJointCount, "joint count must be in [1, 16]");
    }

    // 只在启动期校验模型，允许使用集合换取清晰可靠的重复名称检查。
    std::unordered_set<std::string> names;
    names.reserve(model.joint_count);

    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        const auto& joint = model.joints[i];
        if (joint.name.empty())
        {
            return failure(ValidationError::InvalidName, "joint name must not be empty", i);
        }
        if (!names.insert(joint.name).second)
        {
            return failure(
                ValidationError::DuplicateJointName,
                "joint name is duplicated",
                i);
        }
        if (!is_valid(joint.role))
        {
            return failure(ValidationError::InvalidJointRole, "joint role is invalid", i);
        }

        const auto& limits = joint.limits;
        if (!is_finite(limits.min_position) || !is_finite(limits.max_position) ||
            !is_finite(limits.max_velocity) || !is_finite(limits.max_effort) ||
            !is_finite(limits.max_kp) || !is_finite(limits.max_kd))
        {
            return failure(ValidationError::NonFiniteValue, "joint limit is not finite", i);
        }
        if (limits.position_limited && limits.min_position >= limits.max_position)
        {
            return failure(
                ValidationError::InvalidJointLimits,
                "position-limited joint min_position must be less than max_position",
                i);
        }
        if (limits.max_velocity <= 0.0 || limits.max_effort <= 0.0 || limits.max_kp < 0.0 ||
            limits.max_kd < 0.0)
        {
            return failure(
                ValidationError::InvalidJointLimits,
                "joint velocity/effort must be positive and gains must be non-negative",
                i);
        }
    }
    return {};
}

ValidationResult validate(const ControllerConfig& config, const RobotModel& model)
{
    if (config.control_period_ns <= 0)
    {
        return failure(ValidationError::InvalidConfiguration, "control period must be positive");
    }
    if (config.command_validity_ns <= config.control_period_ns)
    {
        return failure(
            ValidationError::InvalidConfiguration,
            "command validity must be longer than one control period");
    }
    if (config.getup_pre_cycles == 0 || config.getup_cycles == 0 ||
        config.getdown_cycles == 0)
    {
        return failure(
            ValidationError::InvalidConfiguration,
            "interpolation cycle counts must be positive");
    }

    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        const double pre_position = config.pre_getup_position[i];
        const double stand_position = config.stand_position[i];
        const double kp = config.fixed_kp[i];
        const double kd = config.fixed_kd[i];
        if (!is_finite(pre_position) || !is_finite(stand_position) || !is_finite(kp) ||
            !is_finite(kd))
        {
            return failure(
                ValidationError::NonFiniteValue,
                "controller pose or gain is not finite",
                i);
        }
        if (kp < 0.0 || kd < 0.0)
        {
            return failure(ValidationError::NegativeValue, "controller gain is negative", i);
        }

        const auto& limits = model.joints[i].limits;
        if (limits.position_limited &&
            (pre_position < limits.min_position || pre_position > limits.max_position ||
             stand_position < limits.min_position || stand_position > limits.max_position))
        {
            return failure(
                ValidationError::InvalidJointLimits,
                "controller pose exceeds joint position limits",
                i);
        }
        if (kp > limits.max_kp || kd > limits.max_kd)
        {
            return failure(
                ValidationError::InvalidJointLimits,
                "controller gain exceeds RobotModel limits",
                i);
        }
    }
    return {};
}

ValidationResult validate(const StateFrame& frame, const RobotModel& model, Nanoseconds now_ns)
{
    if (const auto result = validate_header(frame.header, now_ns); !result)
    {
        return result;
    }
    if (const auto result = validate_joint_count(frame.joint_count, model); !result)
    {
        return result;
    }

    // 固定容量数组中只有 [0, joint_count) 区间属于当前机器人。
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        const auto& joint = frame.joints[i];
        if (!is_finite(joint.position) || !is_finite(joint.velocity) ||
            !is_finite(joint.effort) || !is_finite(joint.temperature_c))
        {
            return failure(ValidationError::NonFiniteValue, "joint state is not finite", i);
        }
        if (joint.age_ns < 0)
        {
            return failure(ValidationError::NegativeValue, "joint state age is negative", i);
        }
    }

    // IMU 的三个数组分别检查，便于错误信息指出具体数据类别。
    for (const double value : frame.imu.orientation)
    {
        if (!is_finite(value))
        {
            return failure(ValidationError::NonFiniteValue, "IMU orientation is not finite");
        }
    }
    for (const double value : frame.imu.angular_velocity)
    {
        if (!is_finite(value))
        {
            return failure(ValidationError::NonFiniteValue, "IMU angular velocity is not finite");
        }
    }
    for (const double value : frame.imu.linear_acceleration)
    {
        if (!is_finite(value))
        {
            return failure(ValidationError::NonFiniteValue, "IMU acceleration is not finite");
        }
    }
    if (frame.imu.age_ns < 0)
    {
        return failure(ValidationError::NegativeValue, "IMU state age is negative");
    }
    return {};
}

ValidationResult validate(const CommandFrame& frame, const RobotModel& model, Nanoseconds now_ns)
{
    if (const auto result = validate_header(frame.header, now_ns); !result)
    {
        return result;
    }
    if (const auto result = validate_joint_count(frame.joint_count, model); !result)
    {
        return result;
    }
    if (frame.expires_at_ns <= frame.header.timestamp_ns)
    {
        return failure(
            ValidationError::InvalidTimestamp,
            "command expiry must be after its timestamp");
    }
    if (frame.target_generated_at_ns < 0 ||
        frame.target_generated_at_ns > frame.header.timestamp_ns ||
        frame.target_expires_at_ns <= frame.target_generated_at_ns ||
        frame.expires_at_ns > frame.target_expires_at_ns)
    {
        return failure(ValidationError::InvalidTimestamp, "command target timing is invalid");
    }
    // 到达 expires_at_ns 的这一时刻仍视为有效，下一个时刻开始过期。
    if (now_ns > frame.expires_at_ns)
    {
        return failure(ValidationError::Expired, "command frame has expired");
    }
    if (now_ns > frame.target_expires_at_ns)
    {
        return failure(ValidationError::Expired, "command target has expired");
    }
    if (!is_valid(frame.motion_mode))
    {
        return failure(ValidationError::InvalidMotionMode, "motion mode is invalid");
    }
    if (!is_valid(frame.source))
    {
        return failure(ValidationError::InvalidCommandSource, "command source is invalid");
    }

    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        const auto& joint = frame.joints[i];
        if (!is_valid(joint.mode))
        {
            return failure(
                ValidationError::InvalidControlMode,
                "joint control mode is invalid",
                i);
        }
        if (!is_finite(joint.target_position) || !is_finite(joint.target_velocity) ||
            !is_finite(joint.kp) || !is_finite(joint.kd) ||
            !is_finite(joint.feedforward_effort))
        {
            return failure(ValidationError::NonFiniteValue, "joint command is not finite", i);
        }
        if (joint.kp < 0.0 || joint.kd < 0.0)
        {
            return failure(ValidationError::NegativeValue, "joint command gain is negative", i);
        }

        const auto& limits = model.joints[i].limits;
        if (joint.kp > limits.max_kp || joint.kd > limits.max_kd ||
            std::abs(joint.target_velocity) > limits.max_velocity ||
            std::abs(joint.feedforward_effort) > limits.max_effort)
        {
            return failure(
                ValidationError::InvalidJointLimits,
                "joint command exceeds RobotModel limits",
                i);
        }
        // 只有当位置项实际参与关节阻抗控制时，才校验目标位置。
        if (joint.mode == ControlMode::JointImpedance &&
            limits.position_limited && joint.kp > 0.0 &&
            (joint.target_position < limits.min_position ||
             joint.target_position > limits.max_position))
        {
            return failure(
                ValidationError::InvalidJointLimits,
                "joint position target exceeds limits",
                i);
        }
    }
    return {};
}

ValidationResult validate(const BaseCommand& command, Nanoseconds now_ns)
{
    if (command.timestamp_ns < 0 || now_ns < 0 || command.timestamp_ns > now_ns ||
        command.expires_at_ns <= command.timestamp_ns)
    {
        return failure(
            ValidationError::InvalidTimestamp,
            "base command timestamp or expiry is invalid");
    }
    if (now_ns > command.expires_at_ns)
    {
        return failure(ValidationError::Expired, "base command has expired");
    }
    if (!is_valid(command.source) || command.source == CommandSource::None)
    {
        return failure(ValidationError::InvalidCommandSource, "base command source is invalid");
    }
    if (!is_finite(command.vx) || !is_finite(command.vy) || !is_finite(command.wz))
    {
        return failure(ValidationError::NonFiniteValue, "base command velocity is not finite");
    }
    return {};
}

ValidationResult validate(const ModeRequest& request, Nanoseconds now_ns)
{
    if (request.request_id == 0)
    {
        return failure(ValidationError::InvalidIdentifier, "mode request id must not be zero");
    }
    if (request.timestamp_ns < 0 || now_ns < 0 || request.timestamp_ns > now_ns)
    {
        return failure(ValidationError::InvalidTimestamp, "mode request timestamp is invalid");
    }
    if (!is_valid(request.type))
    {
        return failure(ValidationError::InvalidModeRequest, "mode request type is invalid");
    }
    if (request.type == ModeRequestType::StartBehavior && request.behavior_name.empty())
    {
        return failure(
            ValidationError::InvalidName,
            "behavior name is required for StartBehavior");
    }
    if (request.type == ModeRequestType::SwitchPolicy && request.policy_name.empty())
    {
        return failure(ValidationError::InvalidName, "policy name is required for SwitchPolicy");
    }
    return {};
}

}  // 命名空间 quadruped::core
