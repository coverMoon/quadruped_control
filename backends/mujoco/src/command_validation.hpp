/**
 * @file command_validation.hpp
 * @brief 声明 MujocoRobotIO 对 CommandFrame 的额外后端校验。
 */

#pragma once

#include "quadruped/core/robot_io.hpp"
#include "quadruped/core/robot_model.hpp"
#include "quadruped/core/types.hpp"

#include <cstdint>
#include <string>

#include <mujoco/mujoco.h>

namespace quadruped::backends::mujoco
{

// submit() 的校验结果；code 表示处理结论，error_message 仅用于内部诊断。
struct CommandValidationResult
{
    core::RobotIOCode code{core::RobotIOCode::Fault};
    std::string error_message{};
};

// 检查 CommandFrame 是否满足当前会话、序号、控制模式和数值约束。
// 成功时返回 Ok；失败时返回 Rejected 或 InvalidFrame，不修改后端状态。
CommandValidationResult validate_mujoco_command(
    const core::CommandFrame& frame,
    const core::RobotModel& robot_model,
    std::uint64_t session_id,
    std::uint64_t startup_id,
    std::uint64_t latest_command_sequence,
    const mjData& data);

}  // namespace quadruped::backends::mujoco
