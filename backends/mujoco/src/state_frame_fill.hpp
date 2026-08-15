/**
 * @file state_frame_fill.hpp
 * @brief 声明从当前 mjData 填充 StateFrame 关节和 IMU 字段的内部辅助函数。
 */

#pragma once

#include "quadruped/backends/mujoco/mujoco_model.hpp"
#include "quadruped/core/robot_model.hpp"
#include "quadruped/core/types.hpp"

#include <string>

namespace quadruped::backends::mujoco
{

// 按加载时建立的映射顺序填充关节状态；返回空串表示成功，否则返回可读原因。
std::string fill_joint_states(
    core::StateFrame& frame,
    const MujocoModel& model,
    const core::RobotModel& robot_model);

// 填充 IMU 状态；返回空串表示成功，否则返回可读原因。
std::string fill_imu_state(core::StateFrame& frame, const MujocoModel& model);

}  // 命名空间 quadruped::backends::mujoco
