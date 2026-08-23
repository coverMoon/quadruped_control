/**
 * @file replay_log.hpp
 * @brief 定义可校验的 StateFrame CSV 日志读写接口。
 */

#pragma once

#include "quadruped/core/robot_model.hpp"
#include "quadruped/core/types.hpp"

#include <string>
#include <vector>

namespace quadruped::backends::replay
{

struct ReplayLogLoadResult
{
    std::vector<core::StateFrame> frames{};
    std::string error_message{};

    [[nodiscard]] bool ok() const noexcept
    {
        return error_message.empty();
    }
};

// 文件操作仅供低频记录和离线回放调用，不得放入周期控制路径。
[[nodiscard]] bool write_replay_log(
    const std::string& path,
    const core::RobotModel& model,
    const std::vector<core::StateFrame>& frames,
    std::string& error_message);

[[nodiscard]] ReplayLogLoadResult load_replay_log(
    const std::string& path, const core::RobotModel& model);

}  // namespace quadruped::backends::replay
