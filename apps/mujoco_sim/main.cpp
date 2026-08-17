/**
 * @file main.cpp
 * @brief 带 GLFW 界面的交互式 MuJoCo 仿真入口，是 M2 起立和趴下的人工验收入口。
 */

#include "sim_controller.hpp"
#include "sim_display.hpp"
#include "sim_window.hpp"

#include "quadruped/backends/mujoco/mujoco_robot_io.hpp"
#include "quadruped/config/robot_config.hpp"
#include "quadruped/motion/motion_runtime.hpp"

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

// 程序启动标识固定为非零值；会话号由 SimController 从 1 开始递增。
constexpr std::uint64_t kStartupId = 1;

// 低频终端状态显示间隔和渲染帧间隔。
constexpr double kTerminalStatusIntervalSeconds = 2.0;
constexpr double kRenderIntervalSeconds = 1.0 / 60.0;

struct Options
{
    std::string scene_path = kDefaultScenePath;
};

// 按固定墙钟间隔触发的节流器，控制低频渲染和终端显示频率。
class IntervalThrottle
{
public:
    explicit IntervalThrottle(const double interval_seconds)
        : interval_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(interval_seconds)))
    {
    }

    bool due(const std::chrono::steady_clock::time_point now)
    {
        if (now >= next_)
        {
            next_ = now + interval_;
            return true;
        }
        return false;
    }

private:
    std::chrono::steady_clock::duration interval_;
    std::chrono::steady_clock::time_point next_{};
};

void print_usage(const char* program)
{
    std::cout << "用法: " << program << " [选项]\n"
              << "\n选项:\n"
              << "  --scene <路径>  MuJoCo 场景 XML（默认 black 平地）\n"
              << "  -h, --help      显示本帮助\n"
              << "\n按键:\n"
              << "  0 起立  9 趴下  P 被动  R 重置  Space 暂停/继续  Esc 退出\n"
              << "  鼠标左键拖动旋转视角，右键拖动平移，滚轮缩放\n";
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

// 渲染和终端显示的低频节流状态。
struct DisplayThrottles
{
    IntervalThrottle render{kRenderIntervalSeconds};
    IntervalThrottle status{kTerminalStatusIntervalSeconds};
};

// 按节流间隔渲染窗口并打印终端状态。
void update_display(
    qsim::SimWindow& window,
    qsim::SimController& controller,
    DisplayThrottles& throttles)
{
    const auto now = std::chrono::steady_clock::now();
    if (throttles.render.due(now))
    {
        window.render(controller.data(),
            qsim::make_status_text(controller.last_output(), controller.sim_time()),
            qsim::make_detail_text(controller.last_output(), controller.paused()));
    }
    if (throttles.status.due(now))
    {
        qsim::print_terminal_status(controller.last_output(), controller.sim_time());
    }
}

// 主循环：统一调度输入、物理、控制和渲染；键盘回调只更新输入副本。
int run_loop(
    qmj::MujocoRobotIO& io,
    qsim::SimController& controller,
    qsim::SimWindow& window)
{
    using clock = std::chrono::steady_clock;
    const double physics_timestep = io.raw_model()->opt.timestep;
    const auto tick_duration = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(physics_timestep));

    DisplayThrottles throttles;
    const auto loop_start = clock::now();
    auto next_tick = loop_start;
    while (!window.should_close())
    {
        window.poll_events();
        const auto& input = window.input();
        if (input.quit)
        {
            break;
        }
        if (input.toggle_pause)
        {
            controller.toggle_pause();
        }
        if (input.reset)
        {
            if (const std::string error = controller.reset_new_session(); !error.empty())
            {
                std::cerr << "reset 失败: " << error << '\n';
                return 1;
            }
        }
        controller.apply_input(input);

        if (!controller.paused() && !controller.step())
        {
            std::cerr << "仿真步进或控制失败，后端进入不可恢复状态\n";
            return 1;
        }
        update_display(window, controller, throttles);

        next_tick += tick_duration;
        std::this_thread::sleep_until(next_tick);
    }
    return 0;
}

int run(const Options& options)
{
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
    const qsim::WindowOptions window_options{1280, 720, "quadruped mujoco_sim"};
    auto window = qsim::SimWindow::create(created.io->raw_model(), window_options);
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
    return run_loop(*created.io, sim, *window.window);
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
