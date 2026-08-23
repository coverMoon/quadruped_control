/**
 * @file replay_robot_io.hpp
 * @brief 定义只读状态回放 RobotIO，并在内存中记录生成命令用于对比。
 */

#pragma once

#include "quadruped/core/robot_io.hpp"
#include "quadruped/core/robot_model.hpp"

#include <memory>
#include <string>
#include <vector>

namespace quadruped::backends::replay
{

class ReplayRobotIO final : public core::RobotIO
{
public:
    struct CreateResult
    {
        std::unique_ptr<ReplayRobotIO> io{};
        std::string error_message{};

        [[nodiscard]] bool ok() const noexcept
        {
            return io != nullptr;
        }
    };

    static CreateResult create(
        core::RobotModel model, std::vector<core::StateFrame> frames);

    core::RobotIOCode read_latest(core::StateFrame& frame) override;
    core::RobotIOCode submit(const core::CommandFrame& frame) override;
    core::RobotIOStatus status() const noexcept override;

    // 推进到下一条历史状态；到达末尾返回 false。
    // 推进不会驱动任何外部执行设备。
    bool advance() noexcept;
    [[nodiscard]] bool finished() const noexcept;
    [[nodiscard]] const std::vector<core::CommandFrame>& generated_commands() const noexcept;

private:
    ReplayRobotIO(core::RobotModel model, std::vector<core::StateFrame> frames);

    core::RobotModel model_{};
    std::vector<core::StateFrame> frames_{};
    std::vector<core::CommandFrame> generated_commands_{};
    std::size_t frame_index_{0};
    bool current_read_{false};
    core::RobotIOStatus status_{};
};

}  // namespace quadruped::backends::replay
