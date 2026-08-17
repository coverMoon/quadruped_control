/**
 * @file joint_control.hpp
 * @brief 声明 MuJoCo 后端单关节控制力矩计算与执行器写入辅助函数。
 */

#pragma once

#include "quadruped/backends/mujoco/mujoco_model.hpp"
#include "quadruped/core/robot_model.hpp"
#include "quadruped/core/types.hpp"

#include <array>
#include <cstddef>
#include <string>

namespace quadruped::backends::mujoco
{

// 为当前 mjData 计算并写入全部逻辑关节的执行器控制量。
// 使用 latest_command 中每个关节的显式 ControlMode；
// 没有保存命令或命令已过期时，所有主动执行器输出写为零（Disabled 回退）。
// 最终控制量会同时受 RobotModel 的 max_effort 和 MuJoCo actuator ctrlrange 限制。
// 返回空串表示成功，否则返回可读错误原因且不修改 mjData。
std::string apply_joint_commands(
    mjData& data,
    const MujocoModel& model,
    const core::RobotModel& robot_model,
    const core::CommandFrame& command,
    bool command_active);

}  // namespace quadruped::backends::mujoco
