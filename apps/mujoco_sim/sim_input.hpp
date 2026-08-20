/**
 * @file sim_input.hpp
 * @brief 定义 mujoco_sim 主循环使用的按键边沿输入快照。
 */

#pragma once

namespace quadruped::apps::mujoco_sim
{

// 终端线程产生的输入快照；模式字段是边沿事件，速度字段保持最近的累加结果。
struct SimInput
{
    bool getup{false};          // 0：请求起立
    bool start_rl{false};       // 1：从 Stand 启动 RL 行走
    bool getdown{false};        // 9：请求趴下
    bool enter_passive{false};  // P：请求进入被动
    bool reset{false};          // R：reset 并建立新会话
    bool toggle_pause{false};   // K：暂停或继续仿真
    bool quit{false};           // X 或 Esc：退出程序
    bool show_help{false};      // H：重新打印终端帮助
    bool command_changed{false};  // 本次输入是否改变了速度目标
    bool command_rejected{false};  // 非 Running 模式收到速度键，提示一次但不修改目标

    // W/S、A/D、Q/E 逐次调整的机体速度命令，单位为 m/s 和 rad/s。
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
};

}  // 命名空间 quadruped::apps::mujoco_sim
