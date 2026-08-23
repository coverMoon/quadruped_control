/**
 * @file motiond.cpp
 * @brief 运行独立 MotionRuntime，连接共享内存 RobotIO、速度命令和可靠请求队列。
 */

#include "quadruped/config/rl_config_loader.hpp"
#include "quadruped/config/robot_config.hpp"
#include "quadruped/config/policy_switch_loader.hpp"
#include "quadruped/ipc/conversions.hpp"
#include "quadruped/ipc/remote_robot_io.hpp"
#include "quadruped/ipc/shared_memory.hpp"
#include "quadruped/motion/motion_runtime.hpp"
#include "quadruped/policy/torch_policy.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>

namespace qc = quadruped::core;
namespace qi = quadruped::ipc;
namespace qm = quadruped::motion;

namespace
{

constexpr const char* kDefaultSharedMemoryName = "/quadruped_control_black";
constexpr const char* kDefaultRobotConfigPath = QUADRUPED_DEFAULT_ROBOT_CONFIG_PATH;
constexpr const char* kDefaultControllerConfigPath = QUADRUPED_DEFAULT_CONTROLLER_CONFIG_PATH;
constexpr const char* kDefaultPolicySwitchConfigPath = QUADRUPED_DEFAULT_POLICY_SWITCH_CONFIG_PATH;
constexpr std::int64_t kOpenTimeoutNs = 10'000'000'000;
constexpr auto kPollInterval = std::chrono::microseconds(250);

std::atomic<bool> stop_requested{false};

struct Options
{
    std::string shared_memory_name{kDefaultSharedMemoryName};
    std::string robot_config_path{kDefaultRobotConfigPath};
    std::string controller_config_path{kDefaultControllerConfigPath};
    std::string policy_switch_config_path{kDefaultPolicySwitchConfigPath};
    std::string initial_policy{};
};

struct LoadedPolicy
{
    quadruped::motion::RlConfig config{};
    std::unique_ptr<quadruped::policy::TorchPolicy> policy{};
};

struct Policies
{
    std::vector<LoadedPolicy> entries{};
};

void handle_signal(int)
{
    stop_requested.store(true);
}

bool parse_args(int argc, char** argv, Options& options)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help")
        {
            std::cout << "用法: " << argv[0]
                      << " [--shm <名称>] [--robot-config <路径>]"
                      << " [--controller-config <路径>] [--policy-switch-config <路径>]"
                      << " [--initial-policy <策略名>]\n";
            std::exit(0);
        }
        if (i + 1 >= argc)
        {
            return false;
        }
        const std::string value = argv[++i];
        if (arg == "--shm")
        {
            options.shared_memory_name = value;
        }
        else if (arg == "--robot-config")
        {
            options.robot_config_path = value;
        }
        else if (arg == "--controller-config")
        {
            options.controller_config_path = value;
        }
        else if (arg == "--policy-switch-config")
        {
            options.policy_switch_config_path = value;
        }
        else if (arg == "--initial-policy")
        {
            options.initial_policy = value;
        }
        else
        {
            return false;
        }
    }
    return true;
}

qi::SharedMemory::OpenResult wait_for_shared_memory(const std::string& name)
{
    const std::int64_t deadline = qi::monotonic_now_ns() + kOpenTimeoutNs;
    qi::SharedMemory::OpenResult result;
    do
    {
        result = qi::SharedMemory::open_existing(name);
        if (result.ok())
        {
            return result;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (!stop_requested.load() && qi::monotonic_now_ns() < deadline);
    return result;
}

bool load_policies(
    const Options& options,
    const qc::RobotModel& model,
    qm::MotionRuntime& runtime,
    Policies& policies,
    std::string& error_message)
{
    const std::filesystem::path switch_path(options.policy_switch_config_path);
    const auto switch_config = quadruped::config::load_policy_switch_config(
        options.policy_switch_config_path, model.name, switch_path.parent_path().string());
    for (const std::string& warning : switch_config.warnings)
    {
        std::cerr << "策略配置警告: " << warning << '\n';
    }
    if (!switch_config.ok())
    {
        error_message = switch_config.error_message;
        return false;
    }

    const std::string initial_policy = options.initial_policy.empty()
        ? switch_config.config.policy_names.front()
        : options.initial_policy;
    if (std::find(
            switch_config.config.policy_names.begin(),
            switch_config.config.policy_names.end(),
            initial_policy) == switch_config.config.policy_names.end())
    {
        error_message = "initial policy is not in policy_config_cycle: " + initial_policy;
        return false;
    }

    policies.entries.reserve(switch_config.config.policy_names.size());
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
        policies.entries.push_back({loaded.config, std::move(policy.policy)});
    }

    auto initial = std::find_if(
        policies.entries.begin(), policies.entries.end(),
        [&initial_policy](const LoadedPolicy& entry)
        {
            return entry.config.name == initial_policy;
        });
    if (initial == policies.entries.end())
    {
        error_message = "initial policy was not loaded: " + initial_policy;
        return false;
    }
    if (!runtime.attach_policy(initial->config, *initial->policy, error_message))
    {
        return false;
    }
    for (auto& entry : policies.entries)
    {
        if (&entry == &*initial)
        {
            continue;
        }
        if (!runtime.register_policy(entry.config, *entry.policy, error_message))
        {
            return false;
        }
    }
    std::string configure_error;
    if (!runtime.set_policy_cycle(
            switch_config.config.policy_names,
            switch_config.config.posture_transition_cycles,
            configure_error))
    {
        error_message = configure_error;
        return false;
    }
    return true;
}

bool publish_result(
    qi::SharedLayout& layout,
    const qc::ModeResult& result,
    const std::uint64_t startup_id,
    const std::uint64_t session_id)
{
    qi::WireModeResult wire = qi::to_wire(result);
    wire.startup_id = startup_id;
    wire.session_id = session_id;
    if (!qi::queue_push(layout.results, wire))
    {
        std::cerr << "ModeResult 队列已满，无法交付 request_id=" << result.request_id << '\n';
        return false;
    }
    return true;
}

void publish_rejected_wire_request(
    qi::SharedLayout& layout,
    const qi::WireModeRequest& request,
    const std::uint64_t startup_id,
    const std::uint64_t session_id)
{
    qc::ModeResult result;
    result.request_id = request.request_id;
    result.state = qc::ModeResultState::Rejected;
    result.message = "invalid or stale IPC request";
    static_cast<void>(publish_result(layout, result, startup_id, session_id));
}

int run(const Options& options)
{
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

    auto memory = wait_for_shared_memory(options.shared_memory_name);
    if (!memory.ok())
    {
        std::cerr << "连接共享内存失败: " << memory.error_message << '\n';
        return 1;
    }
    std::string identity_error;
    if (!qi::identity_matches(memory.memory->layout().identity, model.model, identity_error))
    {
        std::cerr << "机器人身份校验失败: " << identity_error << '\n';
        return 1;
    }

    auto runtime = qm::MotionRuntime::create(model.model, controller.config);
    if (!runtime.ok())
    {
        std::cerr << "创建 MotionRuntime 失败: " << runtime.error_message << '\n';
        return 1;
    }
    Policies policies;
    std::string policy_error;
    if (!load_policies(options, model.model, *runtime.runtime, policies, policy_error))
    {
        std::cerr << "加载策略失败: " << policy_error << '\n';
        return 1;
    }

    qi::SharedLayout& layout = memory.memory->layout();
    const std::uint64_t motion_startup_id =
        static_cast<std::uint64_t>(qi::monotonic_now_ns());
    qi::RemoteRobotIO io(*memory.memory, model.model);
    std::uint64_t state_version = 0;
    std::uint64_t base_command_version = 0;
    qc::BaseCommand base_command;
    bool has_base_command = false;
    qc::Nanoseconds next_control_ns = -1;
    qc::Nanoseconds latest_state_ns = 0;
    std::uint64_t latest_startup_id = 0;
    std::uint64_t latest_session_id = 0;
    std::uint64_t tracked_request_id = 0;
    qc::ModeResultState tracked_state = qc::ModeResultState::Rejected;
    auto next_disconnected_update = std::chrono::steady_clock::now();

    std::cout << "motiond ready: shm=" << options.shared_memory_name << '\n';
    while (!stop_requested.load())
    {
        qi::WireStateFrame wire_state;
        std::uint64_t current_state_version = 0;
        const bool has_state = qi::read_latest(layout.state, wire_state, &current_state_version);
        if (has_state)
        {
            const bool session_changed = latest_startup_id != 0 &&
                (wire_state.startup_id != latest_startup_id ||
                    wire_state.session_id != latest_session_id);
            latest_state_ns = wire_state.timestamp_ns;
            latest_startup_id = wire_state.startup_id;
            latest_session_id = wire_state.session_id;
            if (session_changed)
            {
                has_base_command = false;
                tracked_request_id = 0;
                next_control_ns = -1;
            }
        }

        const bool new_state = has_state && current_state_version != state_version;
        const auto wall_now = std::chrono::steady_clock::now();
        const bool disconnected_tick = !io.backend_online() && wall_now >= next_disconnected_update;
        bool control_due = false;
        if (new_state)
        {
            state_version = current_state_version;
            if (next_control_ns < 0 || latest_state_ns >= next_control_ns)
            {
                control_due = true;
            }
        }
        if (disconnected_tick)
        {
            control_due = true;
            next_disconnected_update = wall_now +
                std::chrono::nanoseconds(controller.config.control_period_ns);
        }

        if (control_due)
        {
            qi::WireBaseCommand wire_base_command;
            std::uint64_t current_base_version = 0;
            if (qi::read_latest(layout.base_command, wire_base_command, &current_base_version) &&
                current_base_version != base_command_version)
            {
                qi::WireHeartbeat gateway_heartbeat;
                has_base_command =
                    qi::read_latest(layout.gateway_heartbeat, gateway_heartbeat) &&
                    gateway_heartbeat.startup_id == wire_base_command.startup_id &&
                    wire_base_command.session_id == latest_session_id &&
                    qi::from_wire(wire_base_command, base_command);
                base_command_version = current_base_version;
            }

            qc::ModeRequest request;
            qi::WireModeRequest wire_request;
            bool has_request = false;
            if (qi::queue_pop(layout.requests, wire_request))
            {
                qi::WireHeartbeat gateway_heartbeat;
                const bool gateway_matches =
                    qi::read_latest(layout.gateway_heartbeat, gateway_heartbeat) &&
                    gateway_heartbeat.startup_id == wire_request.startup_id;
                has_request = gateway_matches && wire_request.session_id == latest_session_id &&
                    qi::from_wire(wire_request, request);
                if (!has_request)
                {
                    publish_rejected_wire_request(
                        layout, wire_request, motion_startup_id, latest_session_id);
                }
            }

            qm::MotionUpdateInput input;
            input.now_ns = latest_state_ns;
            input.base_command = has_base_command ? &base_command : nullptr;
            input.request = has_request ? &request : nullptr;
            const qm::MotionUpdateOutput output = runtime.runtime->update(io, input);

            qi::WireMotionStatus motion_status = qi::to_wire(output.status);
            motion_status.startup_id = motion_startup_id;
            motion_status.session_id = latest_session_id;
            qi::publish_latest(layout.motion_status, motion_status);

            if (output.has_result)
            {
                static_cast<void>(publish_result(
                    layout, output.result, motion_startup_id, latest_session_id));
                if (output.result.state == qc::ModeResultState::Accepted ||
                    output.result.state == qc::ModeResultState::Running)
                {
                    tracked_request_id = output.result.request_id;
                    tracked_state = output.result.state;
                }
            }
            for (std::size_t i = 0; i < output.result_event_count; ++i)
            {
                static_cast<void>(publish_result(
                    layout, output.result_events[i], motion_startup_id, latest_session_id));
                if (output.result_events[i].request_id == tracked_request_id)
                {
                    tracked_request_id = 0;
                }
            }
            if (tracked_request_id != 0)
            {
                const qc::ModeResult current = runtime.runtime->query_result(tracked_request_id);
                if (current.state != tracked_state)
                {
                    static_cast<void>(publish_result(
                        layout, current, motion_startup_id, latest_session_id));
                    tracked_state = current.state;
                    if (current.state == qc::ModeResultState::Completed ||
                        current.state == qc::ModeResultState::Rejected ||
                        current.state == qc::ModeResultState::Failed)
                    {
                        tracked_request_id = 0;
                    }
                }
            }

            if (next_control_ns < 0)
            {
                next_control_ns = latest_state_ns;
            }
            do
            {
                next_control_ns += controller.config.control_period_ns;
            } while (next_control_ns <= latest_state_ns);
        }

        qi::WireHeartbeat heartbeat;
        heartbeat.startup_id = motion_startup_id;
        heartbeat.session_id = latest_session_id;
        heartbeat.monotonic_ns = qi::monotonic_now_ns();
        heartbeat.online = 1;
        qi::publish_latest(layout.motion_heartbeat, heartbeat);
        std::this_thread::sleep_for(kPollInterval);
    }

    qi::WireHeartbeat heartbeat;
    heartbeat.startup_id = motion_startup_id;
    heartbeat.session_id = latest_session_id;
    heartbeat.monotonic_ns = qi::monotonic_now_ns();
    heartbeat.online = 0;
    qi::publish_latest(layout.motion_heartbeat, heartbeat);
    return 0;
}

}  // 匿名命名空间

int main(int argc, char** argv)
{
    Options options;
    if (!parse_args(argc, argv, options))
    {
        std::cerr << "参数错误，使用 --help 查看说明\n";
        return 2;
    }
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    return run(options);
}
