/**
 * @file command_validation.cpp
 * @brief 实现 MujocoRobotIO 对 CommandFrame 的后端校验。
 */

#include "command_validation.hpp"

#include "sim_time.hpp"

#include "quadruped/core/validation.hpp"

#include <cstdint>
#include <string>

namespace quadruped::backends::mujoco
{

CommandValidationResult validate_mujoco_command(
    const core::CommandFrame& frame,
    const core::RobotModel& robot_model,
    const std::uint64_t session_id,
    const std::uint64_t startup_id,
    const std::uint64_t latest_command_sequence,
    const mjData* data)
{
    if (session_id == 0)
    {
        return {core::RobotIOCode::Rejected, "no active session"};
    }
    if (frame.header.session_id != session_id)
    {
        return {core::RobotIOCode::Rejected, "session_id mismatch"};
    }
    if (frame.header.startup_id != startup_id)
    {
        return {core::RobotIOCode::Rejected, "startup_id mismatch"};
    }
    if (frame.header.sequence == 0 || frame.header.sequence <= latest_command_sequence)
    {
        return {core::RobotIOCode::Rejected, "command sequence is not strictly increasing"};
    }

    // 本阶段只支持三种必要控制模式；Velocity/Torque 明确拒绝。
    for (std::size_t i = 0; i < robot_model.joint_count; ++i)
    {
        const auto mode = frame.joints[i].mode;
        if (mode != core::ControlMode::Disabled && mode != core::ControlMode::Damping &&
            mode != core::ControlMode::JointImpedance)
        {
            return {core::RobotIOCode::Rejected, "unsupported control mode in this phase"};
        }
    }

    if (data == nullptr)
    {
        return {core::RobotIOCode::Fault, "mjData is not available"};
    }

    core::Nanoseconds now_ns{0};
    if (const SimTimeError error = seconds_to_nanoseconds(data->time, now_ns);
        error != SimTimeError::None)
    {
        return {core::RobotIOCode::Fault, "simulation time conversion failed"};
    }

    if (const auto validation = core::validate(frame, robot_model, now_ns); !validation)
    {
        return {core::RobotIOCode::InvalidFrame, validation.message};
    }

    return {core::RobotIOCode::Ok, {}};
}

}  // namespace quadruped::backends::mujoco
