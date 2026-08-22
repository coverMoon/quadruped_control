/**
 * @file mujoco_backendd.cpp
 * @brief 运行独立的 MuJoCo RobotIO 物理后端进程，支持无界面和官方 GUI 模式。
 */

#include "quadruped/backends/mujoco/mujoco_robot_io.hpp"
#include "quadruped/config/robot_config.hpp"
#include "quadruped/ipc/conversions.hpp"
#include "quadruped/ipc/shared_memory.hpp"
#include "sim_window.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

namespace qc = quadruped::core;
namespace qi = quadruped::ipc;
namespace qmj = quadruped::backends::mujoco;
namespace qsim = quadruped::apps::mujoco_sim;

namespace
{

constexpr const char* kDefaultSharedMemoryName = "/quadruped_control_black";
constexpr const char* kDefaultScenePath = QUADRUPED_DEFAULT_SCENE_PATH;
constexpr const char* kDefaultRobotConfigPath = QUADRUPED_DEFAULT_ROBOT_CONFIG_PATH;
constexpr double kDefaultRealTimeFactor = 1.0;
constexpr double kDefaultVisualSyncHz = 60.0;

std::atomic<bool> stop_requested{false};

struct Options
{
    std::string shared_memory_name{kDefaultSharedMemoryName};
    std::string scene_path{kDefaultScenePath};
    std::string robot_config_path{kDefaultRobotConfigPath};
    std::string mode{"headless"};
    double real_time_factor{kDefaultRealTimeFactor};
    double visual_sync_hz{kDefaultVisualSyncHz};
    bool vsync{true};
};

void handle_signal(int)
{
    stop_requested.store(true);
}

void print_usage(const char* program)
{
    std::cout << "用法: " << program
              << " [--shm <名称>] [--scene <路径>] [--robot-config <路径>]"
              << " [--mode headless|gui] [--real-time-factor <倍率>]"
              << " [--visual-sync-hz <频率>] [--vsync true|false]\n";
}

bool parse_bool(const std::string& value, bool& output)
{
    if (value == "true" || value == "1" || value == "yes")
    {
        output = true;
        return true;
    }
    if (value == "false" || value == "0" || value == "no")
    {
        output = false;
        return true;
    }
    return false;
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
        const std::string value = argv[++i];
        if (arg == "--shm")
        {
            options.shared_memory_name = value;
        }
        else if (arg == "--scene")
        {
            options.scene_path = value;
        }
        else if (arg == "--robot-config")
        {
            options.robot_config_path = value;
        }
        else if (arg == "--mode")
        {
            options.mode = value;
        }
        else if (arg == "--real-time-factor")
        {
            try
            {
                options.real_time_factor = std::stod(value);
            }
            catch (const std::exception&)
            {
                return false;
            }
        }
        else if (arg == "--visual-sync-hz")
        {
            try
            {
                options.visual_sync_hz = std::stod(value);
            }
            catch (const std::exception&)
            {
                return false;
            }
        }
        else if (arg == "--vsync")
        {
            if (!parse_bool(value, options.vsync))
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
    return (options.mode == "headless" || options.mode == "gui") &&
        options.real_time_factor >= 0.01 && options.real_time_factor <= 100.0 &&
        options.visual_sync_hz >= 1.0 && options.visual_sync_hz <= 1000.0;
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

bool reset_session(
    qi::SharedLayout& layout,
    qmj::MujocoRobotIO& io,
    const std::uint64_t startup_id,
    std::uint64_t& session_id,
    std::uint64_t& command_version,
    std::string* error_message = nullptr)
{
    const std::uint64_t next_session_id = session_id + 1;
    const auto reset = io.reset(next_session_id);
    if (!reset.ok())
    {
        if (error_message != nullptr)
        {
            *error_message = reset.error_message;
        }
        return false;
    }
    session_id = next_session_id;

    // reset 前共享槽中的旧命令不得在新会话再次提交。
    qi::WireCommandFrame ignored_command;
    static_cast<void>(qi::read_latest(layout.command, ignored_command, &command_version));
    publish_backend_state(layout, io, startup_id, session_id);
    return true;
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
            std::string error_message;
            if (reset_session(
                    layout, io, startup_id, session_id, command_version, &error_message))
            {
                result.success = 1;
                result.session_id = session_id;
                const std::string message = "simulation reset";
                std::copy(message.begin(), message.end(), result.message.begin());
            }
            else
            {
                result.success = 0;
                std::copy_n(
                    error_message.begin(),
                    std::min(error_message.size(), result.message.size() - 1),
                    result.message.begin());
            }
        }
        if (!qi::queue_push(layout.control_results, result))
        {
            std::cerr << "后端控制结果队列已满\n";
        }
    }
}

int run_physics_loop(
    const Options& options,
    qi::SharedLayout& layout,
    qmj::MujocoRobotIO& io,
    const std::uint64_t startup_id,
    std::uint64_t& session_id,
    qsim::SimWindow* const window)
{
    using clock = std::chrono::steady_clock;
    const double timestep = io.raw_model()->opt.timestep;
    const auto tick = std::max(
        clock::duration{1},
        std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(timestep / options.real_time_factor)));
    const auto visual_tick = std::max(
        clock::duration{1},
        std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(1.0 / options.visual_sync_hz)));

    if (window != nullptr)
    {
        try
        {
            window->load(io.raw_model(), io.raw_data(), options.scene_path);
            static_cast<void>(window->sync(io.raw_model(), io.raw_data()));
        }
        catch (const std::exception& error)
        {
            std::cerr << "加载 MuJoCo GUI 失败: " << error.what() << '\n';
            window->request_exit();
            return 1;
        }
    }

    auto next_tick = clock::now();
    auto next_visual_sync = next_tick;
    std::uint64_t command_version = 0;
    while (!stop_requested.load() && (window == nullptr || !window->should_close()))
    {
        process_control_requests(layout, io, startup_id, session_id, command_version);

        const auto now = clock::now();
        if (window != nullptr && now >= next_visual_sync)
        {
            const bool gui_reset_requested = window->sync(io.raw_model(), io.raw_data());
            if (gui_reset_requested)
            {
                std::string error_message;
                if (!reset_session(
                        layout, io, startup_id, session_id, command_version, &error_message))
                {
                    std::cerr << "界面 reset 失败: " << error_message << '\n';
                    window->request_exit();
                    return 1;
                }
            }
            next_visual_sync = now + visual_tick;
        }

        const bool paused = window != nullptr && window->paused();
        if (!paused)
        {
            qi::WireCommandFrame wire_command;
            std::uint64_t current_version = 0;
            if (qi::read_latest(layout.command, wire_command, &current_version) &&
                current_version != command_version)
            {
                qc::CommandFrame command;
                if (qi::from_wire(wire_command, command))
                {
                    static_cast<void>(io.submit(command));
                }
                command_version = current_version;
            }

            if (io.step() != qc::RobotIOCode::Ok)
            {
                std::cerr << "MuJoCo 物理步进失败\n";
                if (window != nullptr)
                {
                    window->request_exit();
                }
                return 1;
            }
            publish_backend_state(layout, io, startup_id, session_id);
        }
        else
        {
            // 暂停时仍维持 heartbeat 和最新状态，不推进 MuJoCo 时间。
            publish_backend_state(layout, io, startup_id, session_id);
        }

        next_tick += tick;
        std::this_thread::sleep_until(next_tick);
    }

    qi::WireHeartbeat heartbeat;
    heartbeat.startup_id = startup_id;
    heartbeat.session_id = session_id;
    heartbeat.monotonic_ns = qi::monotonic_now_ns();
    heartbeat.online = 0;
    qi::publish_latest(layout.backend_heartbeat, heartbeat);
    return 0;
}

int run(const Options& options)
{
    const auto model = quadruped::config::load_robot_model(options.robot_config_path);
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
    std::cout << "mujoco_backendd ready: shm=" << options.shared_memory_name
              << " mode=" << options.mode << " session=" << session_id << '\n';

    if (options.mode == "headless")
    {
        return run_physics_loop(
            options, layout, *created.io, startup_id, session_id, nullptr);
    }

    auto window_result = qsim::SimWindow::create(options.vsync);
    if (!window_result.ok())
    {
        std::cerr << "创建 MuJoCo GUI 失败: " << window_result.error_message << '\n';
        return 1;
    }
    std::unique_ptr<qsim::SimWindow> window = std::move(window_result.window);
    std::atomic<int> physics_result{-1};
    std::thread physics_thread([&]()
    {
        physics_result.store(run_physics_loop(
            options, layout, *created.io, startup_id, session_id, window.get()));
    });

    window->render_loop();
    stop_requested.store(true);
    window->request_exit();
    physics_thread.join();
    return physics_result.load() == 0 ? 0 : 1;
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
