/**
 * @file simulation_config.hpp
 * @brief 声明交互式 MuJoCo 仿真运行参数及其 YAML 加载接口。
 */

#pragma once

#include <string>

namespace quadruped::config
{

// 这里只保存应用调度参数；物理步长、重力和接触参数仍由 MJCF 模型负责。
struct SimulationConfig
{
    // 仿真时间相对墙钟时间的倍率，有效范围 [0.01, 100]；1.0 表示尽量实时运行。
    double real_time_factor{1.0};

    // 状态交给渲染线程的频率，单位为 Hz，有效范围 [1, 1000]；不是 GPU 换帧率。
    double visual_sync_hz{60.0};

    // 是否让官方 Simulate 界面按显示器垂直同步换帧。
    bool vsync{true};
};

struct SimulationConfigLoadResult
{
    SimulationConfig config{};
    std::string error_message{};

    [[nodiscard]] bool ok() const noexcept
    {
        return error_message.empty();
    }
};

// 从 YAML 加载并校验启动期仿真调度参数；高频循环不得调用。
[[nodiscard]] SimulationConfigLoadResult load_simulation_config(const std::string& path);

}  // 命名空间 quadruped::config
