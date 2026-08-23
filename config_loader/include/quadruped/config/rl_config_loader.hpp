/**
 * @file rl_config_loader.hpp
 * @brief 声明 black 与 blackW 共用的 RL 策略配置加载接口。
 */

#pragma once

#include "quadruped/core/robot_model.hpp"
#include "quadruped/motion/rl_controller.hpp"

#include <string>

namespace quadruped::config
{

struct RlConfigLoadResult
{
    motion::RlConfig config{};
    std::string error_message{};

    [[nodiscard]] bool ok() const noexcept
    {
        return error_message.empty();
    }
};

// asset_root 必须是绝对路径；返回的模型路径也始终为绝对路径。
[[nodiscard]] RlConfigLoadResult load_rl_config(
    const std::string& path,
    const std::string& asset_root,
    const core::RobotModel& model);

}  // namespace quadruped::config
