/**
 * @file conversions.cpp
 * @brief 实现 core 与共享内存 wire schema 的字段级转换和枚举检查。
 */

#include "quadruped/ipc/conversions.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace quadruped::ipc
{
namespace
{

template<std::size_t Capacity>
void copy_string(const std::string& input, std::array<char, Capacity>& output)
{
    output.fill('\0');
    const std::size_t length = std::min(input.size(), Capacity - 1);
    std::memcpy(output.data(), input.data(), length);
}

template<std::size_t Capacity>
std::string read_string(const std::array<char, Capacity>& input)
{
    const auto end = std::find(input.begin(), input.end(), '\0');
    return std::string(input.begin(), end);
}

bool valid_control_mode(const std::uint8_t value)
{
    return value <= static_cast<std::uint8_t>(core::ControlMode::Torque);
}

bool valid_motion_mode(const std::uint8_t value)
{
    return value <= static_cast<std::uint8_t>(core::MotionMode::GetDown);
}

bool valid_command_source(const std::uint8_t value)
{
    return value <= static_cast<std::uint8_t>(core::CommandSource::Test);
}

bool valid_safety_state(const std::uint8_t value)
{
    return value <= static_cast<std::uint8_t>(core::SafetyState::EmergencyStop);
}

bool valid_robot_io_state(const std::uint8_t value)
{
    return value <= static_cast<std::uint8_t>(core::RobotIOState::Fault);
}

bool valid_request_type(const std::uint8_t value)
{
    return value <= static_cast<std::uint8_t>(core::ModeRequestType::ResetFault);
}

bool valid_result_state(const std::uint8_t value)
{
    return value <= static_cast<std::uint8_t>(core::ModeResultState::Failed);
}

}  // 匿名命名空间

WireIdentity make_identity(const core::RobotModel& model)
{
    WireIdentity identity;
    identity.magic = kWireMagic;
    identity.wire_schema_version = kWireSchemaVersion;
    identity.frame_schema_version = core::kFrameSchemaVersion;
    identity.joint_count = static_cast<std::uint32_t>(model.joint_count);
    copy_string(model.name, identity.robot_name);
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        copy_string(model.joints[i].name, identity.joint_names[i]);
    }
    return identity;
}

bool identity_matches(
    const WireIdentity& identity,
    const core::RobotModel& model,
    std::string& error_message)
{
    if (identity.magic != kWireMagic ||
        identity.wire_schema_version != kWireSchemaVersion ||
        identity.frame_schema_version != core::kFrameSchemaVersion)
    {
        error_message = "wire or frame schema version mismatch";
        return false;
    }
    if (identity.joint_count != model.joint_count || read_string(identity.robot_name) != model.name)
    {
        error_message = "robot identity mismatch";
        return false;
    }
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        if (read_string(identity.joint_names[i]) != model.joints[i].name)
        {
            error_message = "joint order mismatch at index " + std::to_string(i);
            return false;
        }
    }
    return true;
}

WireStateFrame to_wire(const core::StateFrame& frame)
{
    WireStateFrame wire;
    wire.schema_version = frame.header.schema_version;
    wire.startup_id = frame.header.startup_id;
    wire.session_id = frame.header.session_id;
    wire.sequence = frame.header.sequence;
    wire.timestamp_ns = frame.header.timestamp_ns;
    wire.joint_count = static_cast<std::uint32_t>(frame.joint_count);
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        const auto& source = frame.joints[i];
        auto& target = wire.joints[i];
        target.position = source.position;
        target.velocity = source.velocity;
        target.effort = source.effort;
        target.temperature_c = source.temperature_c;
        target.error_code = source.error_code;
        target.age_ns = source.age_ns;
        target.online = source.online ? 1U : 0U;
        target.valid = source.valid ? 1U : 0U;
    }
    wire.imu.orientation = frame.imu.orientation;
    wire.imu.angular_velocity = frame.imu.angular_velocity;
    wire.imu.linear_acceleration = frame.imu.linear_acceleration;
    wire.imu.age_ns = frame.imu.age_ns;
    wire.imu.valid = frame.imu.valid ? 1U : 0U;
    wire.safety_state = static_cast<std::uint8_t>(frame.safety_state);
    wire.last_accepted_command_sequence = frame.last_accepted_command_sequence;
    wire.effective_command_sequence = frame.effective_command_sequence;
    return wire;
}

bool from_wire(const WireStateFrame& wire, core::StateFrame& frame)
{
    if (wire.joint_count > core::kMaxJoints || !valid_safety_state(wire.safety_state))
    {
        return false;
    }
    frame = {};
    frame.header.schema_version = wire.schema_version;
    frame.header.startup_id = wire.startup_id;
    frame.header.session_id = wire.session_id;
    frame.header.sequence = wire.sequence;
    frame.header.timestamp_ns = wire.timestamp_ns;
    frame.joint_count = wire.joint_count;
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        const auto& source = wire.joints[i];
        auto& target = frame.joints[i];
        target.position = source.position;
        target.velocity = source.velocity;
        target.effort = source.effort;
        target.temperature_c = source.temperature_c;
        target.error_code = source.error_code;
        target.age_ns = source.age_ns;
        target.online = source.online != 0;
        target.valid = source.valid != 0;
    }
    frame.imu.orientation = wire.imu.orientation;
    frame.imu.angular_velocity = wire.imu.angular_velocity;
    frame.imu.linear_acceleration = wire.imu.linear_acceleration;
    frame.imu.age_ns = wire.imu.age_ns;
    frame.imu.valid = wire.imu.valid != 0;
    frame.safety_state = static_cast<core::SafetyState>(wire.safety_state);
    frame.last_accepted_command_sequence = wire.last_accepted_command_sequence;
    frame.effective_command_sequence = wire.effective_command_sequence;
    return true;
}

WireCommandFrame to_wire(const core::CommandFrame& frame)
{
    WireCommandFrame wire;
    wire.schema_version = frame.header.schema_version;
    wire.startup_id = frame.header.startup_id;
    wire.session_id = frame.header.session_id;
    wire.sequence = frame.header.sequence;
    wire.timestamp_ns = frame.header.timestamp_ns;
    wire.expires_at_ns = frame.expires_at_ns;
    wire.target_generated_at_ns = frame.target_generated_at_ns;
    wire.target_expires_at_ns = frame.target_expires_at_ns;
    wire.joint_count = static_cast<std::uint32_t>(frame.joint_count);
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        const auto& source = frame.joints[i];
        auto& target = wire.joints[i];
        target.mode = static_cast<std::uint8_t>(source.mode);
        target.target_position = source.target_position;
        target.target_velocity = source.target_velocity;
        target.kp = source.kp;
        target.kd = source.kd;
        target.feedforward_effort = source.feedforward_effort;
    }
    wire.motion_mode = static_cast<std::uint8_t>(frame.motion_mode);
    wire.source = static_cast<std::uint8_t>(frame.source);
    return wire;
}

bool from_wire(const WireCommandFrame& wire, core::CommandFrame& frame)
{
    if (wire.schema_version != core::kFrameSchemaVersion ||
        wire.joint_count > core::kMaxJoints || !valid_motion_mode(wire.motion_mode) ||
        !valid_command_source(wire.source))
    {
        return false;
    }
    frame = {};
    frame.header.schema_version = wire.schema_version;
    frame.header.startup_id = wire.startup_id;
    frame.header.session_id = wire.session_id;
    frame.header.sequence = wire.sequence;
    frame.header.timestamp_ns = wire.timestamp_ns;
    frame.expires_at_ns = wire.expires_at_ns;
    frame.target_generated_at_ns = wire.target_generated_at_ns;
    frame.target_expires_at_ns = wire.target_expires_at_ns;
    frame.joint_count = wire.joint_count;
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        const auto& source = wire.joints[i];
        if (!valid_control_mode(source.mode))
        {
            return false;
        }
        auto& target = frame.joints[i];
        target.mode = static_cast<core::ControlMode>(source.mode);
        target.target_position = source.target_position;
        target.target_velocity = source.target_velocity;
        target.kp = source.kp;
        target.kd = source.kd;
        target.feedforward_effort = source.feedforward_effort;
    }
    frame.motion_mode = static_cast<core::MotionMode>(wire.motion_mode);
    frame.source = static_cast<core::CommandSource>(wire.source);
    return true;
}

WireBaseCommand to_wire(const core::BaseCommand& command)
{
    WireBaseCommand wire;
    wire.schema_version = core::kFrameSchemaVersion;
    wire.sequence = command.sequence;
    wire.timestamp_ns = command.timestamp_ns;
    wire.expires_at_ns = command.expires_at_ns;
    wire.source = static_cast<std::uint8_t>(command.source);
    wire.priority = command.priority;
    wire.vx = command.vx;
    wire.vy = command.vy;
    wire.wz = command.wz;
    return wire;
}

bool from_wire(const WireBaseCommand& wire, core::BaseCommand& command)
{
    if (wire.schema_version != core::kFrameSchemaVersion ||
        !valid_command_source(wire.source))
    {
        return false;
    }
    command = {};
    command.sequence = wire.sequence;
    command.timestamp_ns = wire.timestamp_ns;
    command.expires_at_ns = wire.expires_at_ns;
    command.source = static_cast<core::CommandSource>(wire.source);
    command.priority = wire.priority;
    command.vx = wire.vx;
    command.vy = wire.vy;
    command.wz = wire.wz;
    return true;
}

WireModeRequest to_wire(const core::ModeRequest& request)
{
    WireModeRequest wire;
    wire.schema_version = core::kFrameSchemaVersion;
    wire.request_id = request.request_id;
    wire.timestamp_ns = request.timestamp_ns;
    wire.type = static_cast<std::uint8_t>(request.type);
    copy_string(request.behavior_name, wire.behavior_name);
    copy_string(request.policy_name, wire.policy_name);
    return wire;
}

bool from_wire(const WireModeRequest& wire, core::ModeRequest& request)
{
    if (wire.schema_version != core::kFrameSchemaVersion || !valid_request_type(wire.type))
    {
        return false;
    }
    request = {};
    request.request_id = wire.request_id;
    request.timestamp_ns = wire.timestamp_ns;
    request.type = static_cast<core::ModeRequestType>(wire.type);
    request.behavior_name = read_string(wire.behavior_name);
    request.policy_name = read_string(wire.policy_name);
    return true;
}

WireModeResult to_wire(const core::ModeResult& result)
{
    WireModeResult wire;
    wire.schema_version = core::kFrameSchemaVersion;
    wire.request_id = result.request_id;
    wire.state = static_cast<std::uint8_t>(result.state);
    copy_string(result.message, wire.message);
    return wire;
}

bool from_wire(const WireModeResult& wire, core::ModeResult& result)
{
    if (wire.schema_version != core::kFrameSchemaVersion || !valid_result_state(wire.state))
    {
        return false;
    }
    result = {};
    result.request_id = wire.request_id;
    result.state = static_cast<core::ModeResultState>(wire.state);
    result.message = read_string(wire.message);
    return true;
}

WireMotionStatus to_wire(const core::MotionStatus& status)
{
    WireMotionStatus wire;
    wire.schema_version = core::kFrameSchemaVersion;
    wire.mode = static_cast<std::uint8_t>(status.mode);
    wire.active_source = static_cast<std::uint8_t>(status.active_source);
    copy_string(status.behavior_name, wire.behavior_name);
    copy_string(status.behavior_phase, wire.behavior_phase);
    copy_string(status.policy_name, wire.policy_name);
    copy_string(status.error_message, wire.error_message);
    wire.policy_ready = status.policy_ready ? 1U : 0U;
    wire.command_limits = status.command_limits;
    return wire;
}


bool from_wire(const WireMotionStatus& wire, core::MotionStatus& status)
{
    if (wire.schema_version != core::kFrameSchemaVersion ||
        !valid_motion_mode(wire.mode) || !valid_command_source(wire.active_source))
    {
        return false;
    }
    status = {};
    status.mode = static_cast<core::MotionMode>(wire.mode);
    status.active_source = static_cast<core::CommandSource>(wire.active_source);
    status.behavior_name = read_string(wire.behavior_name);
    status.behavior_phase = read_string(wire.behavior_phase);
    status.policy_name = read_string(wire.policy_name);
    status.error_message = read_string(wire.error_message);
    status.policy_ready = wire.policy_ready != 0;
    status.command_limits = wire.command_limits;
    if (status.policy_ready &&
        std::any_of(
            status.command_limits.begin(),
            status.command_limits.end(),
            [](const double value) { return !std::isfinite(value) || value <= 0.0; }))
    {
        return false;
    }
    return true;
}

WireRobotIOStatus to_wire(const core::RobotIOStatus& status)
{
    WireRobotIOStatus wire;
    wire.schema_version = core::kFrameSchemaVersion;
    wire.state = static_cast<std::uint8_t>(status.state);
    wire.latest_state_sequence = status.latest_state_sequence;
    wire.latest_command_sequence = status.latest_command_sequence;
    wire.dropped_state_frames = status.dropped_state_frames;
    wire.rejected_command_frames = status.rejected_command_frames;
    return wire;
}


bool from_wire(const WireRobotIOStatus& wire, core::RobotIOStatus& status)
{
    if (wire.schema_version != core::kFrameSchemaVersion || !valid_robot_io_state(wire.state))
    {
        return false;
    }
    status = {};
    status.state = static_cast<core::RobotIOState>(wire.state);
    status.latest_state_sequence = wire.latest_state_sequence;
    status.latest_command_sequence = wire.latest_command_sequence;
    status.dropped_state_frames = wire.dropped_state_frames;
    status.rejected_command_frames = wire.rejected_command_frames;
    return true;
}

}  // 命名空间 quadruped::ipc
