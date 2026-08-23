/**
 * @file main.cpp
 * @brief 从 StateFrame CSV 只读驱动 MotionRuntime 并输出诊断对比日志。
 */

#include "quadruped/backends/replay/diagnostic_log.hpp"
#include "quadruped/backends/replay/replay_log.hpp"
#include "quadruped/backends/replay/replay_robot_io.hpp"
#include "quadruped/config/robot_config.hpp"
#include "quadruped/motion/motion_runtime.hpp"

#include <iostream>
#include <string>

namespace qc = quadruped::core;
namespace qm = quadruped::motion;
namespace qreplay = quadruped::backends::replay;

int main(const int argc, const char* const argv[])
{
    if (argc != 5)
    {
        std::cerr << "用法: quadruped_replay ROBOT_YAML CONTROLLER_YAML "
                     "STATE_LOG.csv DIAGNOSTIC.csv\n";
        return 2;
    }
    const auto model = quadruped::config::load_robot_model(argv[1]);
    if (!model.ok())
    {
        std::cerr << "机器人配置加载失败: " << model.error_message << '\n';
        return 1;
    }
    const auto controller =
        quadruped::config::load_controller_config(argv[2], model.model);
    if (!controller.ok())
    {
        std::cerr << "控制器配置加载失败: " << controller.error_message << '\n';
        return 1;
    }
    const auto log = qreplay::load_replay_log(argv[3], model.model);
    if (!log.ok())
    {
        std::cerr << "状态日志加载失败: " << log.error_message << '\n';
        return 1;
    }
    auto replay = qreplay::ReplayRobotIO::create(model.model, log.frames);
    auto runtime = qm::MotionRuntime::create(model.model, controller.config);
    auto diagnostics = qreplay::DiagnosticLogWriter::create(argv[4], model.model);
    if (!replay.ok() || !runtime.ok() || !diagnostics.ok())
    {
        std::cerr << "回放初始化失败: " << replay.error_message << runtime.error_message
                  << diagnostics.error_message << '\n';
        return 1;
    }

    std::size_t replayed = 0;
    for (const qc::StateFrame& state : log.frames)
    {
        qm::MotionUpdateInput input;
        input.now_ns = state.header.timestamp_ns;
        const auto update = runtime.runtime->update(*replay.io, input);
        const auto& commands = replay.io->generated_commands();
        const qc::CommandFrame* const command = commands.empty() ? nullptr : &commands.back();
        std::string error;
        if (!diagnostics.writer->write(state, update, command, error))
        {
            std::cerr << "诊断日志写入失败: " << error << '\n';
            return 1;
        }
        ++replayed;
        replay.io->advance();
    }
    std::cout << "回放完成: " << replayed
              << " 帧；生成命令仅写入诊断日志，未连接执行设备\n";
    return 0;
}
