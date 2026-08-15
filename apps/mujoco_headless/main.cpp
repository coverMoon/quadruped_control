/**
 * @file main.cpp
 * @brief 最小无界面 MuJoCo 仿真入口，用于手动检查开环行为。
 */

#include "quadruped/backends/mujoco/mujoco_robot_io.hpp"
#include "quadruped/core/core.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace qc = quadruped::core;
namespace qm = quadruped::backends::mujoco;

namespace
{

constexpr const char* kDefaultScenePath = QUADRUPED_DEFAULT_SCENE_PATH;
constexpr double kDefaultDurationSeconds = 2.0;
constexpr double kDefaultSummaryIntervalSeconds = 0.5;
constexpr std::uint64_t kStartupId = 1;
constexpr std::uint64_t kSessionId = 1;

struct Options
{
    std::string scene_path = kDefaultScenePath;
    double duration_seconds = kDefaultDurationSeconds;
    double summary_interval_seconds = kDefaultSummaryIntervalSeconds;
};

qc::RobotModel make_black_model()
{
    qc::RobotModel model;
    model.name = "black";
    model.model_id = 0x008A1E56CD69E8F4;
    model.joint_count = 12;
    constexpr const char* names[12] = {
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    };
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        model.joints[i].name = names[i];
        model.joints[i].role = qc::JointRole::Leg;
        model.joints[i].limits = {true, -3.0, 3.0, 20.0, 40.0, 100.0, 10.0};
    }
    return model;
}

void print_usage(const char* program)
{
    std::cout << "用法: " << program << " [选项]\n"
              << "\n选项:\n"
              << "  --scene <路径>     MuJoCo 场景 XML（默认 black 平地）\n"
              << "  --duration <秒数>  仿真时长（默认 " << kDefaultDurationSeconds << "）\n"
              << "  --summary <秒数>   摘要打印间隔（默认 " << kDefaultSummaryIntervalSeconds
              << "）\n"
              << "  -h, --help         显示本帮助\n";
}

bool parse_double(const char* text, double& out)
{
    try
    {
        std::size_t parsed_length = 0;
        const double value = std::stod(text, &parsed_length);
        if (parsed_length != std::string(text).size() || !std::isfinite(value))
        {
            return false;
        }
        out = value;
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
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
        else if (arg == "--duration")
        {
            if (i + 1 >= argc || !parse_double(argv[++i], options.duration_seconds))
            {
                std::cerr << "--duration 需要正数秒数\n";
                return false;
            }
        }
        else if (arg == "--summary")
        {
            if (i + 1 >= argc || !parse_double(argv[++i], options.summary_interval_seconds))
            {
                std::cerr << "--summary 需要正数秒数\n";
                return false;
            }
        }
        else
        {
            std::cerr << "未知选项: " << arg << '\n';
            return false;
        }
    }
    return true;
}

bool validate_options(const Options& options)
{
    if (options.duration_seconds <= 0.0)
    {
        std::cerr << "仿真时长必须大于 0\n";
        return false;
    }
    if (options.summary_interval_seconds <= 0.0)
    {
        std::cerr << "摘要间隔必须大于 0\n";
        return false;
    }
    return true;
}

void print_summary(double sim_time, std::uint64_t sequence, qc::RobotIOState state)
{
    std::cout << "[t=" << std::fixed << std::setprecision(3) << sim_time
              << "s] seq=" << sequence << " state=" << static_cast<int>(state) << '\n';
}

bool scene_file_exists(const std::string& path)
{
    std::ifstream file(path);
    return file.is_open();
}

bool run_headless(const Options& options)
{
    if (!scene_file_exists(options.scene_path))
    {
        std::cerr << "未找到场景文件：" << options.scene_path
                  << "。请检查 --scene 参数，或恢复仓库内 black 模型。\n";
        return false;
    }

    auto created = qm::MujocoRobotIO::create(options.scene_path, make_black_model(), kStartupId);
    if (!created.ok())
    {
        std::cerr << "创建 MuJoCo 后端失败: " << created.error_message
                  << "。请检查场景文件和 RobotModel 是否匹配。\n";
        return false;
    }

    const auto reset_result = created.io->reset(kSessionId);
    if (!reset_result.ok())
    {
        std::cerr << "reset 失败: " << reset_result.error_message << '\n';
        return false;
    }

    double next_summary_time = 0.0;
    while (true)
    {
        const auto code = created.io->step();
        if (code != qc::RobotIOCode::Ok)
        {
            std::cerr << "仿真步进失败\n";
            return false;
        }

        qc::StateFrame state;
        if (created.io->read_latest(state) != qc::RobotIOCode::Ok)
        {
            std::cerr << "读取最新状态失败\n";
            return false;
        }
        const double sim_time = static_cast<double>(state.header.timestamp_ns) / 1.0e9;

        if (sim_time >= next_summary_time)
        {
            print_summary(sim_time, state.header.sequence, created.io->status().state);
            next_summary_time += options.summary_interval_seconds;
        }

        if (sim_time >= options.duration_seconds)
        {
            break;
        }
    }

    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parse_args(argc, argv, options) || !validate_options(options))
    {
        print_usage(argv[0]);
        return 1;
    }

    return run_headless(options) ? 0 : 1;
}
