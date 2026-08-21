/**
 * @file mujoco_backendd.cpp
 * @brief 运行独立的无界面 MuJoCo RobotIO 物理后端进程。
 */

#include "quadruped/backends/mujoco/mujoco_robot_io.hpp"
#include "quadruped/config/robot_config.hpp"
#include "quadruped/ipc/conversions.hpp"
#include "quadruped/ipc/shared_memory.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

namespace qc = quadruped::core;
namespace qi = quadruped::ipc;
namespace qmj = quadruped::backends::mujoco;

namespace
{

constexpr const char* kDefaultSharedMemoryName = "/quadruped_control_black";
constexpr const char* kDefaultScenePath = QUADRUPED_DEFAULT_SCENE_PATH;
constexpr const char* kDefaultRobotConfigPath = QUADRUPED_DEFAULT_ROBOT_CONFIG_PATH;
constexpr double kDefaultRealTimeFactor = 1.0;

std::atomic<bool> stop_requested{false};

struct Options
{
    std::string shared_memory_name{kDefaultSharedMemoryName};
    std::string scene_path{kDefaultScenePath};
    double real_time_factor{kDefaultRealTimeFactor};
};

void handle_signal(int)
{
    stop_requested.store(true);
}

void print_usage(const char* program)
{
    std::cout << "用法: " << program << " [--shm <名称>] [--scene <路径>] "
              << "[--real-time-factor <倍率>]\n";
}

bool parse_args(int argc, char** argv, Options& options)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help")
        {
            print_usage(argv[0]);
            std::exit(0);
        }
        if (i + 1 >= argc)
        {
            std::cerr << "选项缺少参数: " << arg << '\n';
            return false;
        }
        if (arg == "--shm")
        {
            options.shared_memory_name = argv[++i];
        }
        else if (arg == "--scene")
        {
            options.scene_path = argv[++i];
        }
        else if (arg == "--real-time-factor")
        {
            try
            {
                options.real_time_factor = std::stod(argv[++i]);
            }
            catch (const std::exception&)
            {
                return false;
            }
        }
        else
        {
            std::cerr << "未知选项: " << arg << '\n';
            return false;
        }
    }
    return options.real_time_factor > 0.0 && options.real_time_factor <= 100.0;
}

std::uint64_t make_startup_id()
{
    const auto value = static_cast<std::uint64_t>(qi::monotonic_now_ns());
    return value == 0 ? 1 : value;
}

void publish_backend_state(
    qi::SharedLayout& layout,
    qmj::MujocoRobotIO& io,
    const std::uint64_t startup_id,
    const std::uint64_t session_id)
{
    qc::StateFrame state;
    if (io.read_latest(state) == qc::RobotIOCode::Ok)
    {
        qi::publish_latest(layout.state, qi::to_wire(state));
    }
    qi::WireRobotIOStatus status = qi::to_wire(io.status());
    status.startup_id = startup_id;
    status.session_id = session_id;
    qi::publish_latest(layout.robot_io_status, status);

    qi::WireHeartbeat heartbeat;
    heartbeat.startup_id = startup_id;
    heartbeat.session_id = session_id;
    heartbeat.monotonic_ns = qi::monotonic_now_ns();
    heartbeat.online = 1;
    qi::publish_latest(layout.backend_heartbeat, heartbeat);
}

void process_control_requests(
    qi::SharedLayout& layout,
    qmj::MujocoRobotIO& io,
    const std::uint64_t startup_id,
    std::uint64_t& session_id,
    std::uint64_t& command_version)
{
    qi::WireControlRequest request;
    while (qi::queue_pop(layout.control_requests, request))
    {
        qi::WireControlResult result;
        result.schema_version = qc::kFrameSchemaVersion;
        result.startup_id = startup_id;
        result.session_id = session_id;
        result.request_id = request.request_id;

        const bool valid_request = request.schema_version == qc::kFrameSchemaVersion &&
            request.startup_id == startup_id && request.session_id == session_id &&
            request.request_id != 0 &&
            request.type == static_cast<std::uint8_t>(qi::WireControlType::Reset);
        if (!valid_request)
        {
            result.success = 0;
            const std::string message = "invalid or stale backend control request";
            std::copy(message.begin(), message.end(), result.message.begin());
        }
        else
        {
            const std::uint64_t next_session_id = session_id + 1;
            const auto reset = io.reset(next_session_id);
            if (reset.ok())
            {
                session_id = next_session_id;
                result.success = 1;
                result.session_id = session_id;
                const std::string message = "simulation reset";
                std::copy(message.begin(), message.end(), result.message.begin());

                // reset 前共享槽中的旧命令不得在新会话再次提交。
                qi::WireCommandFrame ignored_command;
                static_cast<void>(
                    qi::read_latest(layout.command, ignored_command, &command_version));
            }
            else
            {
                result.success = 0;
                std::copy_n(
                    reset.error_message.data(),
                    std::min(reset.error_message.size(), result.message.size() - 1),
                    result.message.data());
            }
        }
        if (!qi::queue_push(layout.control_results, result))
        {
            std::cerr << "后端控制结果队列已满\n";
        }
    }
}

int run(const Options& options)
{
    const auto model = quadruped::config::load_robot_model(kDefaultRobotConfigPath);
    if (!model.ok())
    {
        std::cerr << "加载机器人配置失败: " << model.error_message << '\n';
        return 1;
    }

    auto memory = qi::SharedMemory::create_owner(
        options.shared_memory_name, qi::make_identity(model.model));
    if (!memory.ok())
    {
        std::cerr << "创建共享内存失败: " << memory.error_message << '\n';
        return 1;
    }

    const std::uint64_t startup_id = make_startup_id();
    auto created = qmj::MujocoRobotIO::create(options.scene_path, model.model, startup_id);
    if (!created.ok())
    {
        std::cerr << "创建 MuJoCo 后端失败: " << created.error_message << '\n';
        return 1;
    }

    std::uint64_t session_id = 1;
    const auto reset = created.io->reset(session_id);
    if (!reset.ok())
    {
        std::cerr << "建立初始会话失败: " << reset.error_message << '\n';
        return 1;
    }

    qi::SharedLayout& layout = memory.memory->layout();
    publish_backend_state(layout, *created.io, startup_id, session_id);
    const double timestep = created.io->raw_model()->opt.timestep;
    const auto tick = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timestep / options.real_time_factor));
    auto next_tick = std::chrono::steady_clock::now();
    std::uint64_t command_version = 0;

    std::cout << "mujoco_backendd ready: shm=" << options.shared_memory_name
              << " session=" << session_id << '\n';
    while (!stop_requested.load())
    {
        process_control_requests(
            layout, *created.io, startup_id, session_id, command_version);

        qi::WireCommandFrame wire_command;
        std::uint64_t current_version = 0;
        if (qi::read_latest(layout.command, wire_command, &current_version) &&
            current_version != command_version)
        {
            qc::CommandFrame command;
            if (qi::from_wire(wire_command, command))
            {
                static_cast<void>(created.io->submit(command));
            }
            command_version = current_version;
        }

        if (created.io->step() != qc::RobotIOCode::Ok)
        {
            std::cerr << "MuJoCo 物理步进失败\n";
            break;
        }
        publish_backend_state(layout, *created.io, startup_id, session_id);
        next_tick += tick;
        std::this_thread::sleep_until(next_tick);
    }

    qi::WireHeartbeat heartbeat;
    heartbeat.startup_id = startup_id;
    heartbeat.session_id = session_id;
    heartbeat.monotonic_ns = qi::monotonic_now_ns();
    heartbeat.online = 0;
    qi::publish_latest(layout.backend_heartbeat, heartbeat);
    return stop_requested.load() ? 0 : 1;
}

}  // 匿名命名空间

int main(int argc, char** argv)
{
    Options options;
    if (!parse_args(argc, argv, options))
    {
        print_usage(argv[0]);
        return 2;
    }
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    return run(options);
}
