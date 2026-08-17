/**
 * @file sim_display.hpp
 * @brief mujoco_sim 的界面与终端状态显示辅助函数。
 */

#pragma once

#include "quadruped/motion/motion_runtime.hpp"

#include <cstdio>
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

// 叠加在窗口左上角的状态行；界面字体不支持中文，文本使用英文。
inline std::string make_status_text(
    const motion::MotionUpdateOutput& output,
    const double sim_time)
{
    char buffer[256];
    if (output.has_result)
    {
        std::snprintf(buffer, sizeof(buffer), "Mode: %s | Req #%llu: %s | t=%.2fs",
            motion::motion_mode_name(output.status.mode),
            static_cast<unsigned long long>(output.result.request_id),
            result_state_name(output.result.state), sim_time);
    }
    else
    {
        std::snprintf(buffer, sizeof(buffer), "Mode: %s | t=%.2fs",
            motion::motion_mode_name(output.status.mode), sim_time);
    }
    return buffer;
}

// 叠加第二行：优先显示错误，其次暂停提示，否则显示按键帮助。
inline std::string make_detail_text(const motion::MotionUpdateOutput& output, const bool paused)
{
    if (!output.status.error_message.empty())
    {
        return "Error: " + output.status.error_message;
    }
    if (paused)
    {
        return "PAUSED (Space to resume)";
    }
    return "0:GetUp 9:GetDown P:Passive R:Reset Space:Pause Esc:Quit";
}

// 低频终端状态显示，便于不看界面时跟踪流程。
inline void print_terminal_status(
    const motion::MotionUpdateOutput& output,
    const double sim_time)
{
    std::cout << "[t=" << sim_time << "s] 模式="
              << motion::motion_mode_name(output.status.mode);
    if (output.has_result)
    {
        std::cout << " 请求#" << output.result.request_id << "="
                  << result_state_name(output.result.state) << "("
                  << output.result.message << ")";
    }
    if (!output.status.error_message.empty())
    {
        std::cout << " 错误=" << output.status.error_message;
    }
    std::cout << '\n';
}

}  // 命名空间 quadruped::apps::mujoco_sim
