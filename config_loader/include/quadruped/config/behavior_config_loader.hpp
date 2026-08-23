/**
 * @file behavior_config_loader.hpp
 * @brief 声明 Retry、固定姿态轮驱和 Event chain YAML 的启动期加载接口。
 */

#pragma once

#include "quadruped/core/robot_model.hpp"
#include "quadruped/motion/behavior_config.hpp"

#include <string>

namespace quadruped::config
{

struct RetryConfigLoadResult
{
    motion::RetryConfig config{};
    std::string error_message{};

    [[nodiscard]] bool ok() const noexcept
    {
        return error_message.empty();
    }
};

struct EventChainConfigLoadResult
{
    motion::EventChainConfig config{};
    std::string error_message{};

    [[nodiscard]] bool ok() const noexcept
    {
        return error_message.empty();
    }
};

struct FixedDriveConfigLoadResult
{
    motion::FixedDriveConfig config{};
    std::string error_message{};

    [[nodiscard]] bool ok() const noexcept
    {
        return error_message.empty();
    }
};

// 加载并校验 Retry 的机器人名称、关节顺序、数组维度、限制和 Wheel 增益。
[[nodiscard]] RetryConfigLoadResult load_retry_config(
    const std::string& path,
    const core::RobotModel& model);

// 加载 Car、Bridge 或 Low-bar 的固定姿态差速轮配置。
[[nodiscard]] FixedDriveConfigLoadResult load_fixed_drive_config(
    const std::string& path,
    const core::RobotModel& model);

// 加载 Event chain 公共字段并在启动期拒绝空事件、非法类型和不具备的轮事件能力。
[[nodiscard]] EventChainConfigLoadResult load_event_chain_config(
    const std::string& path,
    const core::RobotModel& model);

}  // 命名空间 quadruped::config
