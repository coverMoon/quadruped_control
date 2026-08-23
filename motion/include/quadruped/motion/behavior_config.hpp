/**
 * @file behavior_config.hpp
 * @brief 定义 Retry 和 Event chain 共同行为的启动期配置结构。
 */

#pragma once

#include "quadruped/core/constants.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace quadruped::motion
{

// Retry 配置中的数组按 RobotModel 统一关节顺序排列。
struct RetryConfig
{
    std::string robot_name{};
    std::size_t joint_count{0};
    std::array<std::string, core::kMaxJoints> joint_names{};
    std::uint32_t prepare_cycles{0};
    std::array<double, core::kMaxJoints> target_positions{};
    std::array<double, core::kMaxJoints> kp{};
    std::array<double, core::kMaxJoints> kd{};
};

enum class WheelSide : std::uint8_t
{
    Left = 0,
    Right = 1,
};

// Car、Bridge、Low-bar 共用一个固定姿态差速轮控制器，仅启动期配置不同。
struct FixedDriveConfig
{
    std::string behavior_name{};
    std::string robot_name{};
    std::size_t joint_count{0};
    std::array<std::string, core::kMaxJoints> joint_names{};
    std::uint32_t prepare_cycles{0};
    std::uint32_t exit_to_rl_cycles{0};
    double max_x{0.0};
    double max_yaw{0.0};
    double wheel_velocity_scale{0.0};
    double yaw_to_wheel_velocity{0.0};
    std::size_t wheel_count{0};
    std::array<double, core::kMaxJoints> wheel_velocity_sign{};
    std::array<WheelSide, core::kMaxJoints> wheel_sides{};
    std::array<double, core::kMaxJoints> target_positions{};
    std::array<double, core::kMaxJoints> kp{};
    std::array<double, core::kMaxJoints> kd{};
};

enum class EventType : std::uint8_t
{
    Pose = 0,
    Drive = 1,
    PoseDrive = 2,
};

enum class WheelGroup : std::uint8_t
{
    None = 0,
    Front = 1,
    Rear = 2,
    All = 3,
};

// 单个事件保留公共协议的全部字段；不同 type 只使用与自身相关的字段。
struct EventChainEvent
{
    std::string name{};
    EventType type{EventType::Pose};
    std::uint32_t transition_cycles{0};
    std::uint32_t hold_cycles{0};
    std::array<double, core::kMaxJoints> dof_positions{};
    WheelGroup wheel_group{WheelGroup::None};
    double distance_m{0.0};
    double speed_mps{0.0};
    std::uint32_t timeout_cycles{0};
};

inline constexpr std::size_t kMaxEventChainEvents = 16;

// Event chain 仅描述启动期已验证的事件，不承诺特定事件的物理执行能力。
struct EventChainConfig
{
    std::string robot_name{};
    std::size_t joint_count{0};
    std::array<std::string, core::kMaxJoints> joint_names{};
    std::uint32_t exit_to_rl_cycles{0};
    std::string interpolation{};
    double wheel_radius{0.0};
    std::size_t wheel_count{0};
    std::array<double, core::kMaxJoints> wheel_velocity_sign{};
    std::array<double, core::kMaxJoints> kp{};
    std::array<double, core::kMaxJoints> kd{};
    std::size_t event_count{0};
    std::array<EventChainEvent, kMaxEventChainEvents> events{};
};

}  // 命名空间 quadruped::motion
