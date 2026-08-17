/**
 * @file robot_model.hpp
 * @brief 定义机器人、关节及其机械和控制限制。
 */

#pragma once

#include "quadruped/core/constants.hpp"
#include "quadruped/core/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace quadruped::core
{

// MotionRuntime 和底层适配器共同遵守的单关节允许范围。
struct JointLimits
{
    // true 表示校验位置上下限；轮子等可连续旋转的关节应设为 false。
    bool position_limited{true};

    // 位置下限和上限，单位为 rad；position_limited=false 时不参与范围校验。
    double min_position{0.0};
    double max_position{0.0};

    // 关节输出轴允许的速度绝对值上限，单位为 rad/s。
    double max_velocity{0.0};

    // 关节输出轴允许的力矩绝对值上限，单位为 N·m。
    double max_effort{0.0};

    // CommandFrame 允许提交的 KP 和 KD 上限。
    double max_kp{0.0};
    double max_kd{0.0};
};

// 一个统一关节的稳定名称、功能角色和限制。
struct JointDescription
{
    // 跨策略、仿真和实机保持不变的关节名称，例如 FL_hip_joint。
    std::string name{};

    // 控制系统中的功能用途；
    // 不能从源模型类型、名称或超大位置范围猜测。
    JointRole role{JointRole::Leg};

    JointLimits limits{};
};

// 启动时加载并校验的机器人结构定义，不包含控制器和策略参数。
struct RobotModel
{
    // 稳定的机器人型号名称，例如 black 或 blackw。
    std::string name{};

    // 项目为机器人结构版本明确分配的稳定 64 位 ID。
    std::uint64_t model_id{0};

    // 当前部署使用的标定 ID；仿真模型允许为 0。
    std::uint64_t calibration_id{0};

    // joints 数组中有效的关节数量，范围为 1 到 kMaxJoints。
    std::size_t joint_count{0};

    // 统一关节顺序；StateFrame、CommandFrame 和策略必须使用完全相同的顺序。
    std::array<JointDescription, kMaxJoints> joints{};
};

}  // 命名空间 quadruped::core
