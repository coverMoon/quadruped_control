/**
 * @file controller_config.hpp
 * @brief 定义基础运动控制的启动期参数结构。
 */

#pragma once

#include "quadruped/core/constants.hpp"

#include <array>
#include <cstdint>

namespace quadruped::core
{

// 启动期加载并校验的基础运动控制参数，不包含机器人结构定义（见 RobotModel）。
// 所有数组均按 RobotModel 统一关节顺序排列，只有 [0, joint_count) 区间有效。
struct ControllerConfig
{
    // 运动控制周期，单位为 ns；black 使用 5 ms。
    Nanoseconds control_period_ns{0};

    // CommandFrame 从 timestamp 起的有效期，单位为 ns；第一版取两个控制周期。
    Nanoseconds command_validity_ns{0};

    // 起立第一段（落地姿态到预起立姿态）需要的成功控制周期数。
    std::uint32_t getup_pre_cycles{0};

    // 起立第二段（预起立姿态到默认站姿）需要的成功控制周期数。
    std::uint32_t getup_cycles{0};

    // 趴下（当前姿态到记录的落地姿态）需要的成功控制周期数。
    std::uint32_t getdown_cycles{0};

    // 预起立姿态的目标关节位置，单位为 rad。
    std::array<double, kMaxJoints> pre_getup_position{};

    // 默认站姿的目标关节位置，单位为 rad。
    std::array<double, kMaxJoints> stand_position{};

    // 主动控制使用的固定位置增益，单位为 N·m/rad。
    std::array<double, kMaxJoints> fixed_kp{};

    // 主动控制使用的固定阻尼增益，单位为 N·m·s/rad。
    std::array<double, kMaxJoints> fixed_kd{};
};

}  // 命名空间 quadruped::core
