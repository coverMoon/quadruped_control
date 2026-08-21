/**
 * @file main.cpp
 * @brief 带 GLFW 界面的 MuJoCo 基础运动与 RL 行走入口。
 */

#include "sim_controller.hpp"
#include "sim_display.hpp"
#include "sim_window.hpp"
#include "terminal_input.hpp"

#include "quadruped/backends/mujoco/mujoco_robot_io.hpp"
#include "quadruped/config/robot_config.hpp"
#include "quadruped/config/simulation_config.hpp"
#include "quadruped/motion/motion_runtime.hpp"

#if defined(QUADRUPED_WITH_TORCH)
#include "quadruped/config/rl_config_loader.hpp"
#include "quadruped/policy/torch_policy.hpp"
#endif

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

namespace qc = quadruped::core;
namespace qm = quadruped::motion;
namespace qmj = quadruped::backends::mujoco;
namespace qsim = quadruped::apps::mujoco_sim;

namespace
{

constexpr const char* kDefaultScenePath = QUADRUPED_DEFAULT_SCENE_PATH;
constexpr const char* kDefaultRobotConfigPath = QUADRUPED_DEFAULT_ROBOT_CONFIG_PATH;
constexpr const char* kDefaultControllerConfigPath = QUADRUPED_DEFAULT_CONTROLLER_CONFIG_PATH;
constexpr const char* kDefaultSimulationConfigPath = QUADRUPED_DEFAULT_SIMULATION_CONFIG_PATH;
#if defined(QUADRUPED_WITH_TORCH)
constexpr const char* kDefaultPolicyConfigPath = QUADRUPED_DEFAULT_POLICY_CONFIG_PATH;
#endif

// 程序启动标识固定为非零值；会话号由 SimController 从 1 开始递增。
constexpr std::uint64_t kStartupId = 1;

struct Options
{
    std::string scene_path = kDefaultScenePath;
};

void print_usage(const char* program)
{
    std::cout << "用法: " << program << " [选项]\n"
              << "\n选项:\n"
              << "  --scene <路径>  MuJoCo 场景 XML（默认 black 平地）\n"
              << "  -h, --help      显示本帮助\n"
              << "\n机器人按键在启动程序的终端中输入，MuJoCo 窗口保留官方快捷键。\n"
              << "W/S、A/D、Q/E 每次调整 0.1，Space 速度归零，H 显示完整帮助。\n";
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
        if (arg == "--scene")
        {
            if (i + 1 >= argc)
            {
                std::cerr << "--scene 需要路径参数\n";
                return false;
            }
            options.scene_path = argv[++i];
        }
        else
        {
            std::cerr << "未知选项: " << arg << '\n';
            return false;
        }
    }
    return true;
}

// 物理线程拥有 RobotIO 和 MotionRuntime；主线程只运行官方 Simulate 界面。
int run_physics_loop(
    qmj::MujocoRobotIO& io,
    qsim::SimController& controller,
    qsim::SimWindow& window,
    qsim::TerminalInput& terminal,
    const bool policy_ready,
    const std::string& scene_path,
    const quadruped::config::SimulationConfig& simulation_config)
{
    using clock = std::chrono::steady_clock;
    const double physics_timestep = io.raw_model()->opt.timestep;
    const auto tick_duration = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(
            physics_timestep / simulation_config.real_time_factor));
    const auto visual_sync_duration = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(1.0 / simulation_config.visual_sync_hz));

    qsim::TerminalStatusPrinter status_printer(terminal.interactive());
    window.load(io.raw_model(), io.raw_data(), scene_path);
    static_cast<void>(window.sync());

    auto next_tick = clock::now();
    auto next_visual_sync = next_tick;
    std::uint64_t printed_update_sequence = 0;
    while (!window.should_close())
    {
        // 官方 Reset/Load key 会直接回拨 mjData 时间；同步重建后端和运动会话。
        if (io.raw_data()->time + physics_timestep < controller.sim_time())
        {
            if (const std::string error = controller.reset_new_session(); !error.empty())
            {
                std::cerr << "界面 reset 后重建会话失败: " << error << '\n';
                window.request_exit();
                return 1;
            }
            status_printer.reset();
            printed_update_sequence = 0;
            next_tick = clock::now();
            next_visual_sync = next_tick;
        }
        const bool velocity_enabled =
            controller.last_output().status.mode == qc::MotionMode::Running &&
            controller.last_output().status.behavior_name == "rl_locomotion";
        const auto input = terminal.poll(velocity_enabled);
        if (input.quit)
        {
            window.request_exit();
            break;
        }
        if (input.toggle_pause)
        {
            controller.toggle_pause();
            std::cout << (controller.paused() ? "[仿真] 已暂停\n" : "[仿真] 已继续\n");
        }
        if (input.show_help)
        {
            status_printer.print_help(policy_ready);
        }
        if (input.command_changed)
        {
            status_printer.print_command(input);
        }
        if (input.command_rejected)
        {
            status_printer.print_command_rejected();
        }
        if (input.reset)
        {
            if (const std::string error = controller.reset_new_session(); !error.empty())
            {
                std::cerr << "reset 失败: " << error << '\n';
                window.request_exit();
                return 1;
            }
            status_printer.reset();
            printed_update_sequence = 0;
            window.focus_on_robot(io.raw_model(), io.raw_data());
        }
        controller.apply_input(input);

        if (!controller.paused() && !controller.step())
        {
            std::cerr << "仿真步进或控制失败，后端进入不可恢复状态\n";
            window.request_exit();
            return 1;
        }
        const auto now = clock::now();
        if (now >= next_visual_sync)
        {
            const bool gui_reset_requested = window.sync();
            if (gui_reset_requested)
            {
                if (const std::string error = controller.reset_new_session(); !error.empty())
                {
                    std::cerr << "界面 reset 后重建会话失败: " << error << '\n';
                    window.request_exit();
                    return 1;
                }
                status_printer.reset();
                printed_update_sequence = 0;
                window.focus_on_robot(io.raw_model(), io.raw_data());
                next_tick = now;
            }
            next_visual_sync = now + visual_sync_duration;
        }
        if (!controller.paused() &&
            controller.update_sequence() != printed_update_sequence)
        {
            status_printer.update(controller.last_output(), controller.sim_time());
            printed_update_sequence = controller.update_sequence();
        }

        next_tick += tick_duration;
        std::this_thread::sleep_until(next_tick);
    }
    return 0;
}

int run(const Options& options)
{
    // basic 构建不会使用速度命令；RL 构建会用策略 YAML 覆盖这份保守默认值。
    std::array<double, 3> command_limits{3.0, 1.0, 3.0};
    bool policy_ready = false;
    const auto model = quadruped::config::load_robot_model(kDefaultRobotConfigPath);
    if (!model.ok())
    {
        std::cerr << "加载机器人配置失败: " << model.error_message << '\n';
        return 1;
    }
    const auto controller =
        quadruped::config::load_controller_config(kDefaultControllerConfigPath, model.model);
    if (!controller.ok())
    {
        std::cerr << "加载控制器配置失败: " << controller.error_message << '\n';
        return 1;
    }
    const auto simulation_config =
        quadruped::config::load_simulation_config(kDefaultSimulationConfigPath);
    if (!simulation_config.ok())
    {
        std::cerr << "加载仿真配置失败: " << simulation_config.error_message << '\n';
        return 1;
    }

    auto created = qmj::MujocoRobotIO::create(options.scene_path, model.model, kStartupId);
    if (!created.ok())
    {
        std::cerr << "创建 MuJoCo 后端失败: " << created.error_message << '\n';
        return 1;
    }
    auto runtime = qm::MotionRuntime::create(model.model, controller.config);
    if (!runtime.ok())
    {
        std::cerr << "创建 MotionRuntime 失败: " << runtime.error_message << '\n';
        return 1;
    }
#if defined(QUADRUPED_WITH_TORCH)
    const auto rl_config = quadruped::config::load_rl_config(
        kDefaultPolicyConfigPath, QUADRUPED_PROJECT_SOURCE_DIR, model.model);
    if (!rl_config.ok())
    {
        std::cerr << "加载 RL 配置失败: " << rl_config.error_message << '\n';
        return 1;
    }
    auto policy = quadruped::policy::TorchPolicy::create(rl_config.config);
    if (!policy.ok())
    {
        std::cerr << "加载 RL 策略失败: " << policy.error_message << '\n';
        return 1;
    }
    std::string attach_error;
    if (!runtime.runtime->attach_policy(rl_config.config, *policy.policy, attach_error))
    {
        std::cerr << "接入 RL 策略失败: " << attach_error << '\n';
        return 1;
    }
    command_limits = rl_config.config.command_limits;
    policy_ready = true;
#endif
    qsim::TerminalInput terminal(command_limits);
    if (!terminal.interactive())
    {
        std::cerr << "标准输入不是交互式终端，终端键盘控制已禁用。\n";
    }
    qsim::print_terminal_help(policy_ready);
    auto window = qsim::SimWindow::create(simulation_config.config.vsync);
    if (!window.ok())
    {
        std::cerr << "创建窗口失败: " << window.error_message << '\n';
        return 1;
    }

    qsim::SimController sim(*created.io, *runtime.runtime, controller.config);
    if (const std::string error = sim.reset_new_session(); !error.empty())
    {
        std::cerr << "reset 失败: " << error << '\n';
        return 1;
    }
    std::atomic<int> physics_result{0};
    std::thread physics_thread([&] {
        physics_result.store(
            run_physics_loop(
                *created.io,
                sim,
                *window.window,
                terminal,
                policy_ready,
                options.scene_path,
                simulation_config.config));
    });
    window.window->render_loop();
    window.window->request_exit();
    physics_thread.join();
    return physics_result.load();
}

}  // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parse_args(argc, argv, options))
    {
        print_usage(argv[0]);
        return 1;
    }
    return run(options);
}
