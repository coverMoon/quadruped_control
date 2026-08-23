/**
 * @file main.cpp
 * @brief 带 GLFW 界面的 MuJoCo 基础运动与 RL 行走入口。
 */

#include "sim_controller.hpp"
#include "sim_display.hpp"
#include "sim_window.hpp"
#include "terminal_input.hpp"

#include "quadruped/backends/mujoco/mujoco_robot_io.hpp"
#include "quadruped/config/behavior_config_loader.hpp"
#include "quadruped/config/robot_config.hpp"
#include "quadruped/config/simulation_config.hpp"
#include "quadruped/motion/motion_runtime.hpp"

#if defined(QUADRUPED_WITH_TORCH)
#include "quadruped/config/rl_config_loader.hpp"
#include "quadruped/config/policy_switch_loader.hpp"
#include "quadruped/policy/torch_policy.hpp"
#endif

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <utility>

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
constexpr const char* kDefaultRetryConfigPath = QUADRUPED_DEFAULT_RETRY_CONFIG_PATH;
#if defined(QUADRUPED_WITH_TORCH)
constexpr const char* kDefaultPolicySwitchConfigPath =
    QUADRUPED_DEFAULT_POLICY_SWITCH_CONFIG_PATH;
#endif

// 程序启动标识固定为非零值；会话号由 SimController 从 1 开始递增。
constexpr std::uint64_t kStartupId = 1;

struct Options
{
    std::string scene_path = kDefaultScenePath;
    std::string robot_config_path = kDefaultRobotConfigPath;
    std::string controller_config_path = kDefaultControllerConfigPath;
    std::string simulation_config_path = kDefaultSimulationConfigPath;
    std::string retry_config_path = kDefaultRetryConfigPath;
    std::string event_chain_config_path{};
    std::string fixed_drive_config_dir{};
#if defined(QUADRUPED_WITH_TORCH)
    std::string policy_switch_config_path = kDefaultPolicySwitchConfigPath;
    bool load_policy{true};
#endif
};

#if defined(QUADRUPED_WITH_TORCH)
struct LoadedPolicy
{
    quadruped::motion::RlConfig config{};
    std::unique_ptr<quadruped::policy::TorchPolicy> policy{};
};

bool load_policies(
    const std::string& policy_switch_config_path,
    const qc::RobotModel& model,
    qm::MotionRuntime& runtime,
    std::vector<LoadedPolicy>& policies,
    std::array<double, 3>& command_limits,
    std::string& error_message)
{
    const std::filesystem::path switch_path(policy_switch_config_path);
    const auto switch_config = quadruped::config::load_policy_switch_config(
        policy_switch_config_path, model.name, switch_path.parent_path().string());
    for (const std::string& warning : switch_config.warnings)
    {
        std::cerr << "策略配置警告: " << warning << '\n';
    }
    if (!switch_config.ok())
    {
        error_message = switch_config.error_message;
        return false;
    }

    policies.reserve(switch_config.config.policy_names.size());
    for (const std::string& policy_name : switch_config.config.policy_names)
    {
        const std::filesystem::path policy_path = switch_path.parent_path() / (policy_name + ".yaml");
        const auto loaded = quadruped::config::load_rl_config(
            policy_path.string(), QUADRUPED_PROJECT_SOURCE_DIR, model);
        if (!loaded.ok())
        {
            error_message = loaded.error_message;
            return false;
        }
        if (loaded.config.name != policy_name)
        {
            error_message = "policy config name does not match cycle item: " + policy_name;
            return false;
        }
        auto policy = quadruped::policy::TorchPolicy::create(loaded.config);
        if (!policy.ok())
        {
            error_message = policy.error_message;
            return false;
        }
        policies.push_back({loaded.config, std::move(policy.policy)});
    }

    if (policies.empty())
    {
        error_message = "policy switch cycle is empty";
        return false;
    }
    command_limits = policies.front().config.command_limits;
    if (!runtime.attach_policy(policies.front().config, *policies.front().policy, error_message))
    {
        return false;
    }
    for (std::size_t i = 1; i < policies.size(); ++i)
    {
        if (!runtime.register_policy(policies[i].config, *policies[i].policy, error_message))
        {
            return false;
        }
    }
    if (!runtime.set_policy_cycle(
            switch_config.config.policy_names,
            switch_config.config.posture_transition_cycles,
            error_message))
    {
        return false;
    }
    return true;
}
#endif

void print_usage(const char* program)
{
    std::cout << "用法: " << program << " [选项]\n"
              << "\n选项:\n"
              << "  --scene <路径>  MuJoCo 场景 XML（默认 black 地形）\n"
              << "  --robot-config <路径>      RobotModel YAML\n"
              << "  --controller-config <路径> 基础动作 YAML\n"
              << "  --simulation-config <路径> 仿真调度 YAML\n"
              << "  --retry-config <路径>      Retry 行为 YAML\n"
              << "  --event-chain-config <路径> Event chain 行为 YAML\n"
              << "  --fixed-drive-config-dir <目录> Car/Bridge/Low-bar YAML 目录\n"
#if defined(QUADRUPED_WITH_TORCH)
              << "  --policy-switch-config <路径> RL 策略循环 YAML\n"
              << "  --no-policy                不加载 Torch 策略\n"
#endif
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
#if defined(QUADRUPED_WITH_TORCH)
        if (arg == "--no-policy")
        {
            options.load_policy = false;
            continue;
        }
#endif
        if (i + 1 >= argc)
        {
            std::cerr << arg << " 需要路径参数\n";
            return false;
        }
        const std::string value = argv[++i];
        if (arg == "--scene")
        {
            options.scene_path = value;
        }
        else if (arg == "--robot-config")
        {
            options.robot_config_path = value;
        }
        else if (arg == "--controller-config")
        {
            options.controller_config_path = value;
        }
        else if (arg == "--simulation-config")
        {
            options.simulation_config_path = value;
        }
        else if (arg == "--retry-config")
        {
            options.retry_config_path = value;
        }
        else if (arg == "--event-chain-config")
        {
            options.event_chain_config_path = value;
        }
        else if (arg == "--fixed-drive-config-dir")
        {
            options.fixed_drive_config_dir = value;
        }
#if defined(QUADRUPED_WITH_TORCH)
        else if (arg == "--policy-switch-config")
        {
            options.policy_switch_config_path = value;
        }
#endif
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
    static_cast<void>(window.sync(io.raw_model(), io.raw_data()));

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
            (controller.last_output().status.behavior_name == "rl_locomotion" ||
                controller.last_output().status.behavior_name == "bridge_drive" ||
                controller.last_output().status.behavior_name == "low_bar_drive" ||
                controller.last_output().status.behavior_name == "car_drive");
        if (controller.last_output().status.policy_ready)
        {
            terminal.set_command_limits(controller.last_output().status.command_limits);
        }
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
            const int default_pose_id =
                mj_name2id(io.raw_model(), mjOBJ_KEY, "default_pose");
            if (const std::string error =
                    controller.reset_simulation_state(default_pose_id);
                !error.empty())
            {
                std::cerr << "reset 失败: " << error << '\n';
                window.request_exit();
                return 1;
            }
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
            const qsim::SimWindow::SimulationAction action =
                window.sync(io.raw_model(), io.raw_data());
            if (action.type != qsim::SimWindow::SimulationAction::Type::None)
            {
                const int keyframe_id =
                    action.type == qsim::SimWindow::SimulationAction::Type::LoadKey
                    ? action.keyframe_id
                    : -1;
                if (const std::string error =
                        controller.reset_simulation_state(keyframe_id);
                    !error.empty())
                {
                    std::cerr << "界面仿真状态复位失败: " << error << '\n';
                    window.request_exit();
                    return 1;
                }
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
    const auto model = quadruped::config::load_robot_model(options.robot_config_path);
    if (!model.ok())
    {
        std::cerr << "加载机器人配置失败: " << model.error_message << '\n';
        return 1;
    }
    const auto controller =
        quadruped::config::load_controller_config(options.controller_config_path, model.model);
    if (!controller.ok())
    {
        std::cerr << "加载控制器配置失败: " << controller.error_message << '\n';
        return 1;
    }
    const auto simulation_config =
        quadruped::config::load_simulation_config(options.simulation_config_path);
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
    const auto retry = quadruped::config::load_retry_config(
        options.retry_config_path, model.model);
    if (!retry.ok())
    {
        std::cerr << "加载 Retry 配置失败: " << retry.error_message << '\n';
        return 1;
    }
    std::string behavior_error;
    if (!runtime.runtime->configure_retry(retry.config, behavior_error))
    {
        std::cerr << "配置 Retry 失败: " << behavior_error << '\n';
        return 1;
    }
    if (!options.event_chain_config_path.empty())
    {
        const auto event_chain = quadruped::config::load_event_chain_config(
            options.event_chain_config_path, model.model);
        if (!event_chain.ok() ||
            !runtime.runtime->configure_event_chain(event_chain.config, behavior_error))
        {
            const std::string reason = event_chain.ok()
                ? behavior_error
                : event_chain.error_message;
            std::cerr << "配置 Event chain 失败: " << reason << '\n';
            return 1;
        }
    }
    if (!options.fixed_drive_config_dir.empty())
    {
        constexpr const char* kFixedDriveFiles[] = {
            "bridge_drive.yaml", "low_bar_drive.yaml", "car_drive.yaml"};
        for (const char* file : kFixedDriveFiles)
        {
            const std::filesystem::path path =
                std::filesystem::path(options.fixed_drive_config_dir) / file;
            const auto fixed = quadruped::config::load_fixed_drive_config(
                path.string(), model.model);
            if (!fixed.ok() ||
                !runtime.runtime->configure_fixed_drive(fixed.config, behavior_error))
            {
                const std::string reason = fixed.ok()
                    ? behavior_error
                    : fixed.error_message;
                std::cerr << "配置固定姿态轮驱失败: " << reason << '\n';
                return 1;
            }
        }
    }
#if defined(QUADRUPED_WITH_TORCH)
    std::vector<LoadedPolicy> policies;
    std::string policy_error;
    if (options.load_policy &&
        !load_policies(options.policy_switch_config_path,
            model.model,
            *runtime.runtime,
            policies,
            command_limits,
            policy_error))
    {
        std::cerr << "加载 RL 策略失败: " << policy_error << '\n';
        return 1;
    }
    policy_ready = options.load_policy;
#endif
    qsim::TerminalInput terminal(
        command_limits, !options.fixed_drive_config_dir.empty());
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
