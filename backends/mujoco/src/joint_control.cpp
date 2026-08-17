/**
 * @file joint_control.cpp
 * @brief 实现单关节控制力矩计算与执行器写入。
 */

#include "joint_control.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

namespace quadruped::backends::mujoco
{
namespace
{

// 计算单个关节按显式控制模式得到的输出力矩，不做限幅。
// 不同模式只读取各自需要的字段，不根据 KP/KD 数值推断模式。
double compute_joint_effort(
    const core::JointCommand& command,
    double position,
    double velocity)
{
    switch (command.mode)
    {
    case core::ControlMode::Disabled:
        return 0.0;

    case core::ControlMode::Damping:
        return -command.kd * velocity;

    case core::ControlMode::JointImpedance:
        return command.kp * (command.target_position - position) +
            command.kd * (command.target_velocity - velocity) +
            command.feedforward_effort;

    default:
        // 本阶段不支持的 ControlMode 应在更早的校验层拒绝，不应到达此处。
        return 0.0;
    }
}

}  // namespace

std::string apply_joint_commands(
    mjData& data,
    const MujocoModel& model,
    const core::RobotModel& robot_model,
    const core::CommandFrame& command,
    const bool command_active)
{
    const mjModel* raw = model.raw_model();

    // model、data 和关节映射已经在 MujocoRobotIO 创建时完成校验。
    // 先把所有执行器控制量置零，保证未被显式写入的执行器不会保持旧命令。
    for (int i = 0; i < raw->nu; ++i)
    {
        data.ctrl[i] = 0.0;
    }

    for (std::size_t i = 0; i < robot_model.joint_count; ++i)
    {
        const auto& mapping = model.joint_mappings()[i];

        // 无论命令是否有效，步进前都必须检查关节状态有限，防止无命令或过期回退时
        // 把 NaN/Inf 状态交给 mj_step 处理。
        const double position = data.qpos[mapping.qpos_address];
        const double velocity = data.qvel[mapping.qvel_address];
        if (!std::isfinite(position) || !std::isfinite(velocity))
        {
            return "joint state is not finite for joint \"" +
                robot_model.joints[i].name + "\"";
        }

        if (!command_active)
        {
            continue;
        }

        double effort = compute_joint_effort(command.joints[i], position, velocity);
        if (!std::isfinite(effort))
        {
            return "computed effort is not finite for joint \"" +
                robot_model.joints[i].name + "\"";
        }

        // 同时受 RobotModel 力矩上限和 MuJoCo 执行器范围限制。
        const double max_effort = robot_model.joints[i].limits.max_effort;
        const int actuator_id = mapping.actuator_id;
        const double ctrl_min = raw->actuator_ctrlrange[2 * actuator_id];
        const double ctrl_max = raw->actuator_ctrlrange[2 * actuator_id + 1];
        const double lower = std::max(ctrl_min, -max_effort);
        const double upper = std::min(ctrl_max, max_effort);
        data.ctrl[actuator_id] = std::max(lower, std::min(upper, effort));
    }
    return {};
}

}  // namespace quadruped::backends::mujoco
