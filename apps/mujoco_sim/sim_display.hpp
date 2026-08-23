/**
 * @file sim_display.hpp
 * @brief 提供仿真终端帮助、速度命令和状态变化日志。
 */

#pragma once

#include "sim_input.hpp"

#include "quadruped/motion/motion_runtime.hpp"

#include <iomanip>
#include <iostream>
#include <string>

namespace quadruped::apps::mujoco_sim
{

inline const char* result_state_name(const core::ModeResultState state)
{
    switch (state)
    {
    case core::ModeResultState::Accepted:
        return "Accepted";
    case core::ModeResultState::Rejected:
        return "Rejected";
    case core::ModeResultState::Running:
        return "Running";
    case core::ModeResultState::Completed:
        return "Completed";
    case core::ModeResultState::Failed:
        return "Failed";
    }
    return "Unknown";
}

inline void print_terminal_help(const bool policy_ready)
{
    std::cout << "终端控制（MuJoCo 窗口只保留官方快捷键）：\n"
              << "  0 起立  1 启动 RL  2/3 切换到策略循环下一项  9 趴下  P 被动  R 重置\n"
              << "  W/S 前后 ±0.1  A/D 横移 ±0.1  Q/E 转向 ±0.1  Space 速度归零\n"
              << "  K 暂停/继续  H 帮助  X/Esc 退出\n";
    if (!policy_ready)
    {
        std::cout << "  当前是 basic 构建，未加载 Torch 策略。\n";
    }
}

// 只在模式、请求终态或错误发生变化时输出，避免周期性刷屏。
class TerminalStatusPrinter
{
public:
    explicit TerminalStatusPrinter(const bool interactive) : interactive_(interactive) {}

    ~TerminalStatusPrinter()
    {
        prepare_event_output();
    }

    void reset() noexcept
    {
        initialized_ = false;
        last_error_.clear();
    }

    void print_help(const bool policy_ready)
    {
        prepare_event_output();
        print_terminal_help(policy_ready);
    }

    void print_command(const SimInput& input)
    {
        const auto previous_flags = std::cout.flags();
        const auto previous_precision = std::cout.precision();
        if (interactive_)
        {
            std::cout << "\r\033[2K";
        }
        std::cout << std::fixed << std::setprecision(1)
                  << "[命令] vx=" << input.vx << " m/s, vy=" << input.vy
                  << " m/s, wz=" << input.wz << " rad/s";
        std::cout.flags(previous_flags);
        std::cout.precision(previous_precision);
        if (interactive_)
        {
            std::cout.flush();
            command_line_active_ = true;
        }
        else
        {
            std::cout << '\n';
        }
    }

    void print_command_rejected()
    {
        prepare_event_output();
        std::cout << "[提示] 速度命令仅在 RL Running 模式下有效。\n";
    }

    void update(const motion::MotionUpdateOutput& output, const double sim_time)
    {
        const bool mode_changed = !initialized_ || output.status.mode != last_mode_;
        const bool error_changed = output.status.error_message != last_error_;
        if (!mode_changed && !output.has_result && output.result_event_count == 0 &&
            !error_changed)
        {
            return;
        }
        prepare_event_output();
        if (mode_changed)
        {
            std::cout << "[t=" << sim_time << "s] 模式="
                      << motion::motion_mode_name(output.status.mode) << '\n';
        }
        if (output.has_result)
        {
            print_result(output.result, sim_time);
        }
        for (std::size_t i = 0; i < output.result_event_count; ++i)
        {
            print_result(output.result_events[i], sim_time);
        }
        if (error_changed)
        {
            if (!output.status.error_message.empty())
            {
                std::cout << "[t=" << sim_time << "s] 错误="
                          << output.status.error_message << '\n';
            }
            else if (!last_error_.empty())
            {
                std::cout << "[t=" << sim_time << "s] 错误已清除\n";
            }
        }

        initialized_ = true;
        last_mode_ = output.status.mode;
        last_error_ = output.status.error_message;
    }

private:
    void prepare_event_output()
    {
        if (!command_line_active_)
        {
            return;
        }
        std::cout << "\r\033[2K";
        std::cout.flush();
        command_line_active_ = false;
    }

    static void print_result(const core::ModeResult& result, const double sim_time)
    {
        std::cout << "[t=" << sim_time << "s] 请求#" << result.request_id << '='
                  << result_state_name(result.state) << '(' << result.message << ")\n";
    }

    bool initialized_{false};
    bool interactive_{false};
    bool command_line_active_{false};
    core::MotionMode last_mode_{core::MotionMode::Passive};
    std::string last_error_{};
};

}  // 命名空间 quadruped::apps::mujoco_sim
