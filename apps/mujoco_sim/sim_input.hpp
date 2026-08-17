/**
 * @file sim_input.hpp
 * @brief 定义 mujoco_sim 主循环使用的按键边沿输入快照。
 */

#pragma once

namespace quadruped::apps::mujoco_sim
{

// 一次 poll_events() 内产生的按键边沿事件；每个字段在下一帧自动清零。
struct SimInput
{
    bool getup{false};          // 0：请求起立
    bool getdown{false};        // 9：请求趴下
    bool enter_passive{false};  // P：请求进入被动
    bool reset{false};          // R：reset 并建立新会话
    bool toggle_pause{false};   // Space：暂停或继续仿真
    bool quit{false};           // Esc：退出程序
};

}  // 命名空间 quadruped::apps::mujoco_sim
