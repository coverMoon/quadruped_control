/**
 * @file terminal_input.hpp
 * @brief 定义与 rl_sar 类似的终端单键速度和模式控制输入。
 */

#pragma once

#include "sim_input.hpp"

#include <array>
#include <cstddef>
#include <termios.h>

namespace quadruped::apps::mujoco_sim
{

// 在交互式终端中关闭规范输入和回显；析构时恢复用户原始终端设置。
class TerminalInput
{
public:
    explicit TerminalInput(std::array<double, 3> command_limits);
    TerminalInput(const TerminalInput&) = delete;
    TerminalInput& operator=(const TerminalInput&) = delete;
    ~TerminalInput();

    // 消费当前已经到达的全部字符，并返回持久速度和一次性模式事件。
    [[nodiscard]] SimInput poll(bool velocity_enabled);

    [[nodiscard]] bool interactive() const noexcept
    {
        return interactive_;
    }

private:
    void handle_character(char character, bool velocity_enabled, SimInput& input);
    void adjust(double& value, double increment, double limit, SimInput& input);
    void adjust_command(
        double& value,
        double increment,
        std::size_t axis,
        bool velocity_enabled,
        SimInput& input);

    std::array<double, 3> command_limits_{};
    double vx_{0.0};
    double vy_{0.0};
    double wz_{0.0};
    termios original_termios_{};
    bool interactive_{false};
    bool warned_outside_running_{false};
};

}  // 命名空间 quadruped::apps::mujoco_sim
