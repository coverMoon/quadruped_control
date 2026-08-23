/**
 * @file ros2_gateway.cpp
 * @brief 实现 ROS 2 顶层网关到本机共享内存运行链路的请求和状态接入。
 */

#include "quadruped/core/types.hpp"
#include "quadruped/ipc/conversions.hpp"
#include "quadruped/ipc/shared_memory.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "quadruped_interfaces/action/get_down.hpp"
#include "quadruped_interfaces/action/get_up.hpp"
#include "quadruped_interfaces/action/start_behavior.hpp"
#include "quadruped_interfaces/action/switch_policy.hpp"
#include "quadruped_interfaces/msg/mode_result.hpp"
#include "quadruped_interfaces/msg/motion_status.hpp"
#include "quadruped_interfaces/msg/robot_io_status.hpp"
#include "quadruped_interfaces/msg/state_diagnostic.hpp"
#include "quadruped_interfaces/srv/enter_passive.hpp"
#include "quadruped_interfaces/srv/reset_fault.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace qc = quadruped::core;
namespace qi = quadruped::ipc;
namespace qmsg = quadruped_interfaces::msg;
namespace qsrv = quadruped_interfaces::srv;

namespace
{

using GetUp = quadruped_interfaces::action::GetUp;
using GetDown = quadruped_interfaces::action::GetDown;
using StartBehavior = quadruped_interfaces::action::StartBehavior;
using SwitchPolicy = quadruped_interfaces::action::SwitchPolicy;

constexpr const char* kDefaultSharedMemoryName = "/quadruped_control_black";
constexpr std::int64_t kDefaultCommandTimeoutNs = 200'000'000;
constexpr std::int64_t kResultWaitTimeoutNs = 10'000'000'000;
constexpr std::int64_t kHeartbeatTimeoutNs = 500'000'000;
constexpr auto kPumpInterval = std::chrono::milliseconds(2);
constexpr auto kStatusPeriod = std::chrono::milliseconds(50);
constexpr auto kKeyboardPollPeriod = std::chrono::milliseconds(50);
constexpr std::int64_t kDefaultJoyTimeoutNs = 250'000'000;
constexpr double kKeyboardIncrement = 0.1;
constexpr std::array<double, 3> kFallbackCommandLimits{3.0, 1.0, 3.0};

struct JoyEdgeState
{
    bool a{false};
    bool b{false};
    bool x{false};
    bool y{false};
    bool lb{false};
    bool rb{false};
    bool passive_combo{false};
    bool reset_combo{false};
    bool rl_combo{false};
    bool event_chain_combo{false};
    bool bridge_combo{false};
    bool low_bar_combo{false};
    bool car_combo{false};
    bool pause_combo{false};
};

std::uint64_t make_startup_id()
{
    const auto value = static_cast<std::uint64_t>(qi::monotonic_now_ns());
    return value == 0 ? 1 : value;
}

qmsg::ModeResult to_ros_result(const qc::ModeResult& result)
{
    qmsg::ModeResult output;
    output.request_id = result.request_id;
    output.state = static_cast<std::uint8_t>(result.state);
    output.message = result.message;
    return output;
}

qmsg::ModeResult to_ros_result(const qi::WireModeResult& wire)
{
    qc::ModeResult result;
    if (!qi::from_wire(wire, result))
    {
        result.request_id = wire.request_id;
        result.state = qc::ModeResultState::Failed;
        result.message = "invalid ModeResult wire frame";
    }
    return to_ros_result(result);
}

bool is_terminal(const std::uint8_t state)
{
    return state == static_cast<std::uint8_t>(qc::ModeResultState::Completed) ||
        state == static_cast<std::uint8_t>(qc::ModeResultState::Rejected) ||
        state == static_cast<std::uint8_t>(qc::ModeResultState::Failed);
}

const char* motion_mode_name(const qc::MotionMode mode) noexcept
{
    switch (mode)
    {
    case qc::MotionMode::Passive:
        return "Passive";
    case qc::MotionMode::GetUp:
        return "GetUp";
    case qc::MotionMode::Stand:
        return "Stand";
    case qc::MotionMode::Running:
        return "Running";
    case qc::MotionMode::GetDown:
        return "GetDown";
    default:
        return "Unknown";
    }
}


}  // 匿名命名空间

class Ros2Gateway final : public rclcpp::Node
{
public:
    Ros2Gateway()
        : Node("ros2_gateway"), gateway_startup_id_(make_startup_id())
    {
        shared_memory_name_ = declare_parameter<std::string>(
            "shared_memory_name", kDefaultSharedMemoryName);
        command_timeout_ns_ = declare_parameter<std::int64_t>(
            "cmd_vel_timeout_ns", kDefaultCommandTimeoutNs);
        if (command_timeout_ns_ <= 0)
        {
            command_timeout_ns_ = kDefaultCommandTimeoutNs;
        }
        joy_timeout_ns_ = declare_parameter<std::int64_t>(
            "joy_timeout_ns", kDefaultJoyTimeoutNs);
        if (joy_timeout_ns_ <= 0)
        {
            joy_timeout_ns_ = kDefaultJoyTimeoutNs;
        }
        keyboard_enabled_ = declare_parameter<bool>("keyboard_enabled", true);
        fixed_drive_keys_enabled_ = declare_parameter<bool>(
            "fixed_drive_keys_enabled", false);
        joy_require_connection_frame_ = declare_parameter<bool>(
            "joy_require_connection_frame", true);
        joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
        // 指令终端不仅负责键盘输入，手柄模式也需要显示 RL 当前速度指令。
        terminal_ui_enabled_ = isatty(STDOUT_FILENO) != 0;

        auto opened = qi::SharedMemory::open_existing(shared_memory_name_);
        if (!opened.ok())
        {
            throw std::runtime_error("打开共享内存失败: " + opened.error_message);
        }
        memory_ = std::move(opened.memory);

        cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
            [this](const geometry_msgs::msg::Twist::SharedPtr message)
            {
                handle_cmd_vel(*message);
            });
        joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
            joy_topic_,
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            [this](const sensor_msgs::msg::Joy::SharedPtr message)
            {
                handle_joy(*message);
            });

        motion_status_publisher_ = create_publisher<qmsg::MotionStatus>(
            "/motion/status", rclcpp::QoS(1).reliable().transient_local());
        robot_io_status_publisher_ = create_publisher<qmsg::RobotIOStatus>(
            "/robot_io/status", rclcpp::QoS(1).reliable().transient_local());
        diagnostic_publisher_ = create_publisher<qmsg::StateDiagnostic>(
            "/state/diagnostic", rclcpp::QoS(1).best_effort());
        result_publisher_ = create_publisher<qmsg::ModeResult>(
            "/motion/result", rclcpp::QoS(16).reliable());

        get_up_server_ = rclcpp_action::create_server<GetUp>(
            this, "/motion/get_up",
            [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const GetUp::Goal> goal)
            {
                return goal_callback(goal->request_id);
            },
            [this](const std::shared_ptr<GoalHandle<GetUp>> goal_handle)
            {
                return cancel_callback(goal_handle);
            },
            [this](const std::shared_ptr<GoalHandle<GetUp>> goal_handle)
            {
                accept_get_up(goal_handle);
            });
        get_down_server_ = rclcpp_action::create_server<GetDown>(
            this, "/motion/get_down",
            [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const GetDown::Goal> goal)
            {
                return goal_callback(goal->request_id);
            },
            [this](const std::shared_ptr<GoalHandle<GetDown>> goal_handle)
            {
                return cancel_callback(goal_handle);
            },
            [this](const std::shared_ptr<GoalHandle<GetDown>> goal_handle)
            {
                accept_get_down(goal_handle);
            });
        start_behavior_server_ = rclcpp_action::create_server<StartBehavior>(
            this, "/motion/start_behavior",
            [this](const rclcpp_action::GoalUUID&,
                std::shared_ptr<const StartBehavior::Goal> goal)
            {
                return goal_callback(goal->request_id);
            },
            [this](const std::shared_ptr<GoalHandle<StartBehavior>> goal_handle)
            {
                return cancel_callback(goal_handle);
            },
            [this](const std::shared_ptr<GoalHandle<StartBehavior>> goal_handle)
            {
                accept_start_behavior(goal_handle);
            });
        switch_policy_server_ = rclcpp_action::create_server<SwitchPolicy>(
            this, "/motion/switch_policy",
            [this](const rclcpp_action::GoalUUID&,
                std::shared_ptr<const SwitchPolicy::Goal> goal)
            {
                return goal_callback(goal->request_id);
            },
            [this](const std::shared_ptr<GoalHandle<SwitchPolicy>> goal_handle)
            {
                return cancel_callback(goal_handle);
            },
            [this](const std::shared_ptr<GoalHandle<SwitchPolicy>> goal_handle)
            {
                accept_switch_policy(goal_handle);
            });

        enter_passive_service_ = create_service<qsrv::EnterPassive>(
            "/motion/enter_passive",
            [this](const std::shared_ptr<rmw_request_id_t>,
                const std::shared_ptr<qsrv::EnterPassive::Request> request,
                std::shared_ptr<qsrv::EnterPassive::Response> response)
            {
                handle_enter_passive(*request, *response);
            });
        reset_fault_service_ = create_service<qsrv::ResetFault>(
            "/motion/reset_fault",
            [this](const std::shared_ptr<rmw_request_id_t>,
                const std::shared_ptr<qsrv::ResetFault::Request> request,
                std::shared_ptr<qsrv::ResetFault::Response> response)
            {
                handle_reset_fault(*request, *response);
            });

        status_timer_ = create_wall_timer(kStatusPeriod, [this]()
        {
            publish_status();
            refresh_manual_command();
        });
        result_thread_ = std::thread([this]() { result_pump(); });
        if (keyboard_enabled_ && isatty(STDIN_FILENO) != 0)
        {
            keyboard_thread_ = std::thread([this]() { keyboard_loop(); });
        }
    }

    ~Ros2Gateway() override
    {
        stopping_.store(true);
        result_condition_.notify_all();
        if (terminal_ui_enabled_)
        {
            std::lock_guard<std::mutex> lock(terminal_output_mutex_);
            std::cout << '\n' << std::flush;
        }
        restore_terminal();
        if (keyboard_thread_.joinable())
        {
            keyboard_thread_.join();
        }
        if (result_thread_.joinable())
        {
            result_thread_.join();
        }
        std::lock_guard<std::mutex> lock(worker_mutex_);
        for (std::thread& worker : workers_)
        {
            if (worker.joinable())
            {
                worker.join();
            }
        }
    }

private:
    template<typename ActionT>
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    rclcpp_action::GoalResponse goal_callback(const std::uint64_t request_id) const
    {
        if (request_id == 0)
        {
            RCLCPP_WARN(get_logger(), "拒绝 request_id=0 的 ROS action 请求");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    template<typename ActionT>
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<GoalHandle<ActionT>>&)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    bool current_session(std::uint64_t& startup_id, std::uint64_t& session_id,
        std::int64_t& timestamp_ns) const
    {
        qi::WireStateFrame state;
        qi::WireHeartbeat heartbeat;
        if (!qi::read_latest(memory_->layout().state, state) ||
            !qi::read_latest(memory_->layout().backend_heartbeat, heartbeat) ||
            heartbeat.online == 0 || state.startup_id != heartbeat.startup_id ||
            state.session_id != heartbeat.session_id)
        {
            return false;
        }
        startup_id = gateway_startup_id_;
        session_id = state.session_id;
        timestamp_ns = state.timestamp_ns;
        return true;
    }

    bool motion_online(const std::uint64_t session_id) const
    {
        qi::WireHeartbeat heartbeat;
        if (!qi::read_latest(memory_->layout().motion_heartbeat, heartbeat) ||
            heartbeat.online == 0 || heartbeat.session_id != session_id)
        {
            return false;
        }
        const std::int64_t now_ns = qi::monotonic_now_ns();
        return heartbeat.monotonic_ns > 0 && now_ns >= heartbeat.monotonic_ns &&
            now_ns - heartbeat.monotonic_ns <= kHeartbeatTimeoutNs;
    }

    bool submit_request(const qc::ModeRequest& request)
    {
        std::uint64_t ignored_startup = 0;
        std::uint64_t session_id = 0;
        std::int64_t state_timestamp_ns = 0;
        if (!current_session(ignored_startup, session_id, state_timestamp_ns) ||
            !motion_online(session_id))
        {
            return false;
        }
        qi::WireModeRequest wire = qi::to_wire(request);
        wire.startup_id = gateway_startup_id_;
        wire.session_id = session_id;
        wire.timestamp_ns = state_timestamp_ns;
        std::lock_guard<std::mutex> lock(request_mutex_);
        return qi::queue_push(memory_->layout().requests, wire);
    }

    qc::ModeRequest make_request(
        const std::uint64_t request_id,
        const qc::ModeRequestType type,
        const std::string& behavior_name = {},
        const std::string& policy_name = {}) const
    {
        qc::ModeRequest request;
        request.request_id = request_id;
        request.timestamp_ns = qi::monotonic_now_ns();
        request.type = type;
        request.behavior_name = behavior_name;
        request.policy_name = policy_name;
        return request;
    }

    void publish_base_command(
        const double vx,
        const double vy,
        const double wz,
        const qc::CommandSource source,
        const std::uint8_t priority)
    {
        std::uint64_t ignored_startup = 0;
        std::uint64_t session_id = 0;
        std::int64_t state_timestamp_ns = 0;
        if (!current_session(ignored_startup, session_id, state_timestamp_ns))
        {
            return;
        }
        qc::BaseCommand command;
        command.sequence = ++base_command_sequence_;
        command.timestamp_ns = state_timestamp_ns;
        command.expires_at_ns = state_timestamp_ns + command_timeout_ns_;
        command.source = source;
        command.priority = priority;
        command.vx = std::isfinite(vx)
            ? std::clamp(vx, -command_limit(0), command_limit(0))
            : 0.0;
        command.vy = std::isfinite(vy)
            ? std::clamp(vy, -command_limit(1), command_limit(1))
            : 0.0;
        command.wz = std::isfinite(wz)
            ? std::clamp(wz, -command_limit(2), command_limit(2))
            : 0.0;
        published_command_vx_.store(command.vx);
        published_command_vy_.store(command.vy);
        published_command_wz_.store(command.wz);
        qi::WireBaseCommand wire = qi::to_wire(command);
        wire.startup_id = gateway_startup_id_;
        wire.session_id = session_id;
        std::lock_guard<std::mutex> lock(command_mutex_);
        qi::publish_latest(memory_->layout().base_command, wire);
    }

    void handle_cmd_vel(const geometry_msgs::msg::Twist& message)
    {
        navigation_vx_.store(std::isfinite(message.linear.x) ? message.linear.x : 0.0);
        navigation_vy_.store(std::isfinite(message.linear.y) ? message.linear.y : 0.0);
        navigation_wz_.store(std::isfinite(message.angular.z) ? message.angular.z : 0.0);
        last_cmd_vel_ns_.store(qi::monotonic_now_ns());
        if (!manual_input_active_.load())
        {
            publish_base_command(
                navigation_vx_.load(), navigation_vy_.load(), navigation_wz_.load(),
                qc::CommandSource::Navigation, 100);
        }
    }

    void handle_joy(const sensor_msgs::msg::Joy& message)
    {
        const bool marked_connected =
            message.header.frame_id.rfind("joy_connected", 0) == 0;
        const bool marked_disconnected =
            message.header.frame_id.rfind("joy_disconnected", 0) == 0;
        const bool connected = !marked_disconnected &&
            (!joy_require_connection_frame_ || marked_connected);
        const bool valid_axes = message.axes.size() >= 8;
        const bool valid_buttons = message.buttons.size() >= 6;
        if (!connected || !valid_axes || !valid_buttons)
        {
            const bool was_online = joy_online_.exchange(false);
            {
                std::lock_guard<std::mutex> lock(joy_info_mutex_);
                joy_profile_name_.clear();
            }
            last_joy_buttons_ = {};
            if (was_online)
            {
                manual_input_active_.store(false);
                set_manual_command(0.0, 0.0, 0.0);
            }
            return;
        }
        const bool was_online = joy_online_.exchange(true);
        {
            const std::string prefix = "joy_connected:";
            std::lock_guard<std::mutex> lock(joy_info_mutex_);
            if (message.header.frame_id.rfind(prefix, 0) == 0)
            {
                joy_profile_name_ = message.header.frame_id.substr(prefix.size());
            }
            else if (joy_profile_name_.empty())
            {
                joy_profile_name_ = "标准手柄";
            }
        }
        if (!was_online)
        {
            last_joy_buttons_ = {};
            manual_input_active_.store(true);
        }
        last_joy_ns_.store(qi::monotonic_now_ns());
        if (manual_input_active_.load())
        {
            const double vx = std::clamp(static_cast<double>(message.axes[1]), -1.0, 1.0) *
                command_limit(0);
            const double vy = std::clamp(static_cast<double>(message.axes[0]), -1.0, 1.0) *
                command_limit(1);
            const double wz = std::clamp(static_cast<double>(message.axes[3]), -1.0, 1.0) *
                command_limit(2);
            set_manual_command(vx, vy, wz);
        }
        publish_joy_edges(message);
    }

    void set_manual_command(const double vx, const double vy, const double wz)
    {
        manual_vx_.store(std::clamp(vx, -command_limit(0), command_limit(0)));
        manual_vy_.store(std::clamp(vy, -command_limit(1), command_limit(1)));
        manual_wz_.store(std::clamp(wz, -command_limit(2), command_limit(2)));
    }

    [[nodiscard]] double command_limit(const std::size_t axis) const noexcept
    {
        return command_limits_[axis].load();
    }

    void update_command_limits(const std::array<double, 3>& limits)
    {
        for (std::size_t axis = 0; axis < limits.size(); ++axis)
        {
            if (!std::isfinite(limits[axis]) || limits[axis] <= 0.0)
            {
                return;
            }
        }
        for (std::size_t axis = 0; axis < limits.size(); ++axis)
        {
            command_limits_[axis].store(limits[axis]);
        }
        set_manual_command(manual_vx_.load(), manual_vy_.load(), manual_wz_.load());
    }

    void refresh_manual_command()
    {
        const std::int64_t now_ns = qi::monotonic_now_ns();
        const std::int64_t last_joy_ns = last_joy_ns_.load();
        if (joy_online_.load() && last_joy_ns != 0 &&
            now_ns - last_joy_ns > joy_timeout_ns_)
        {
            joy_online_.store(false);
            {
                std::lock_guard<std::mutex> lock(joy_info_mutex_);
                joy_profile_name_.clear();
            }
            manual_input_active_.store(false);
            set_manual_command(0.0, 0.0, 0.0);
            last_joy_buttons_ = {};
        }
        if (manual_input_active_.load())
        {
            if (current_mode_.load() != static_cast<std::uint8_t>(qc::MotionMode::Running))
            {
                publish_base_command(0.0, 0.0, 0.0, qc::CommandSource::Gamepad, 110);
                return;
            }
            publish_base_command(
                manual_vx_.load(), manual_vy_.load(), manual_wz_.load(),
                qc::CommandSource::Gamepad, 110);
            return;
        }
        const std::int64_t last_cmd_vel_ns = last_cmd_vel_ns_.load();
        if (last_cmd_vel_ns == 0 || now_ns - last_cmd_vel_ns > command_timeout_ns_)
        {
            return;
        }
        publish_base_command(
            navigation_vx_.load(), navigation_vy_.load(), navigation_wz_.load(),
            qc::CommandSource::Navigation, 100);
    }

    void publish_joy_edges(const sensor_msgs::msg::Joy& message)
    {
        const bool a = message.buttons[0] != 0;
        const bool b = message.buttons[1] != 0;
        const bool x = message.buttons[2] != 0;
        const bool y = message.buttons[3] != 0;
        const bool lb = message.buttons[4] != 0;
        const bool rb = message.buttons[5] != 0;
        const bool dpad_up = message.axes[7] > 0.5;
        const bool dpad_down = message.axes[7] < -0.5;
        const bool dpad_left = message.axes[6] < -0.5;
        const bool dpad_right = message.axes[6] > 0.5;
        const bool passive_combo = lb && x;
        const bool reset_combo = rb && y;
        const bool rl_combo = rb && dpad_up;
        const bool event_chain_combo = lb && dpad_up;
        const bool bridge_combo = rb && dpad_right;
        const bool low_bar_combo = rb && dpad_down;
        const bool car_combo = rb && dpad_left;
        const bool pause_combo = rb && x;

        if (a && !last_joy_buttons_.a)
        {
            submit_keyboard_request(qc::ModeRequestType::GetUp);
        }
        if (b && !last_joy_buttons_.b)
        {
            submit_keyboard_request(qc::ModeRequestType::GetDown);
        }
        if (x && !lb && !rb && !last_joy_buttons_.x)
        {
            manual_input_active_.store(!manual_input_active_.load());
        }
        if (y && !rb && !last_joy_buttons_.y)
        {
            submit_keyboard_request(qc::ModeRequestType::SwitchPolicy, {}, "toggle");
        }
        if (passive_combo && !last_joy_buttons_.passive_combo)
        {
            submit_keyboard_request(qc::ModeRequestType::EnterPassive);
        }
        if (reset_combo && !last_joy_buttons_.reset_combo)
        {
            submit_backend_reset();
        }
        if (rl_combo && !last_joy_buttons_.rl_combo)
        {
            // 旧 rl_sar 使用 RB+DPadUp 进入基础 locomotion，不能只依赖单独的数字键。
            submit_start_rl_behavior();
        }
        if (event_chain_combo && !last_joy_buttons_.event_chain_combo)
        {
            submit_keyboard_request(qc::ModeRequestType::StartBehavior, "event_chain");
        }
        if (fixed_drive_keys_enabled_ && bridge_combo &&
            !last_joy_buttons_.bridge_combo)
        {
            submit_keyboard_request(qc::ModeRequestType::StartBehavior, "bridge_drive");
        }
        if (fixed_drive_keys_enabled_ && low_bar_combo &&
            !last_joy_buttons_.low_bar_combo)
        {
            submit_keyboard_request(qc::ModeRequestType::StartBehavior, "low_bar_drive");
        }
        if (fixed_drive_keys_enabled_ && car_combo && !last_joy_buttons_.car_combo)
        {
            submit_keyboard_request(qc::ModeRequestType::StartBehavior, "car_drive");
        }
        if (pause_combo && !last_joy_buttons_.pause_combo)
        {
            submit_backend_pause_toggle();
        }

        last_joy_buttons_.a = a;
        last_joy_buttons_.b = b;
        last_joy_buttons_.x = x;
        last_joy_buttons_.y = y;
        last_joy_buttons_.lb = lb;
        last_joy_buttons_.rb = rb;
        last_joy_buttons_.passive_combo = passive_combo;
        last_joy_buttons_.reset_combo = reset_combo;
        last_joy_buttons_.rl_combo = rl_combo;
        last_joy_buttons_.event_chain_combo = event_chain_combo;
        last_joy_buttons_.bridge_combo = bridge_combo;
        last_joy_buttons_.low_bar_combo = low_bar_combo;
        last_joy_buttons_.car_combo = car_combo;
        last_joy_buttons_.pause_combo = pause_combo;
    }


    template<typename ActionT>
    void start_action_worker(
        const std::shared_ptr<GoalHandle<ActionT>>& goal_handle,
        const qc::ModeRequest& request)
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        workers_.emplace_back([this, goal_handle, request]()
        {
            run_action_worker<ActionT>(goal_handle, request);
        });
    }

    template<typename ActionT>
    void run_action_worker(
        const std::shared_ptr<GoalHandle<ActionT>>& goal_handle,
        const qc::ModeRequest& request)
    {
        qmsg::ModeResult initial;
        initial.request_id = request.request_id;
        initial.state = static_cast<std::uint8_t>(qc::ModeResultState::Accepted);
        initial.message = "request submitted";
        auto feedback = std::make_shared<typename ActionT::Feedback>();
        feedback->status = initial;
        goal_handle->publish_feedback(feedback);

        std::uint64_t ignored_startup = 0;
        std::uint64_t expected_session_id = 0;
        std::int64_t ignored_timestamp = 0;
        static_cast<void>(
            current_session(ignored_startup, expected_session_id, ignored_timestamp));
        std::uint8_t last_state = initial.state;
        std::string last_message = initial.message;
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::nanoseconds(kResultWaitTimeoutNs);
        while (!stopping_.load() && rclcpp::ok() &&
            std::chrono::steady_clock::now() < deadline)
        {
            if (!motion_online(expected_session_id))
            {
                auto result = std::make_shared<typename ActionT::Result>();
                result->result = initial;
                result->result.state = static_cast<std::uint8_t>(qc::ModeResultState::Failed);
                result->result.message = "motiond disconnected";
                clear_active_action(request.request_id);
                goal_handle->abort(result);
                return;
            }
            if (goal_handle->is_canceling())
            {
                auto result = std::make_shared<typename ActionT::Result>();
                result->result = initial;
                result->result.state = static_cast<std::uint8_t>(qc::ModeResultState::Failed);
                result->result.message = "ROS action goal canceled";
                clear_active_action(request.request_id);
                goal_handle->canceled(result);
                return;
            }

            qi::WireModeResult wire;
            bool has_result = false;
            {
                std::lock_guard<std::mutex> lock(result_mutex_);
                const auto found = latest_results_.find(request.request_id);
                if (found != latest_results_.end())
                {
                    wire = found->second;
                    has_result = true;
                }
            }
            if (has_result)
            {
                const qmsg::ModeResult current = to_ros_result(wire);
                if (current.state != last_state || current.message != last_message)
                {
                    feedback = std::make_shared<typename ActionT::Feedback>();
                    feedback->status = current;
                    goal_handle->publish_feedback(feedback);
                    last_state = current.state;
                    last_message = current.message;
                }
                if (is_terminal(current.state))
                {
                    auto result = std::make_shared<typename ActionT::Result>();
                    result->result = current;
                    clear_active_action(request.request_id);
                    if (current.state == static_cast<std::uint8_t>(qc::ModeResultState::Completed))
                    {
                        goal_handle->succeed(result);
                    }
                    else
                    {
                        goal_handle->abort(result);
                    }
                    return;
                }
            }
            std::unique_lock<std::mutex> lock(result_mutex_);
            result_condition_.wait_for(lock, std::chrono::milliseconds(50));
        }

        auto result = std::make_shared<typename ActionT::Result>();
        result->result = initial;
        result->result.state = static_cast<std::uint8_t>(qc::ModeResultState::Failed);
        result->result.message = stopping_.load()
            ? "gateway stopped"
            : "timeout waiting for motiond result";
        clear_active_action(request.request_id);
        goal_handle->abort(result);
    }

    void clear_active_action(const std::uint64_t request_id)
    {
        std::uint64_t expected = request_id;
        static_cast<void>(active_action_request_id_.compare_exchange_strong(expected, 0));
    }

    void accept_get_up(const std::shared_ptr<GoalHandle<GetUp>>& goal_handle)
    {
        const auto request = make_request(
            goal_handle->get_goal()->request_id, qc::ModeRequestType::GetUp);
        if (!submit_request(request))
        {
            finish_action_immediately<GetUp>(goal_handle, request.request_id, "motiond unavailable");
            return;
        }
        active_action_request_id_.store(request.request_id);
        start_action_worker(goal_handle, request);
    }

    void accept_get_down(const std::shared_ptr<GoalHandle<GetDown>>& goal_handle)
    {
        const auto request = make_request(
            goal_handle->get_goal()->request_id, qc::ModeRequestType::GetDown);
        if (!submit_request(request))
        {
            finish_action_immediately<GetDown>(
                goal_handle, request.request_id, "motiond unavailable");
            return;
        }
        active_action_request_id_.store(request.request_id);
        start_action_worker(goal_handle, request);
    }

    void accept_start_behavior(const std::shared_ptr<GoalHandle<StartBehavior>>& goal_handle)
    {
        const auto request = make_request(
            goal_handle->get_goal()->request_id,
            qc::ModeRequestType::StartBehavior,
            goal_handle->get_goal()->behavior_name);
        if (!submit_request(request))
        {
            finish_action_immediately<StartBehavior>(
                goal_handle, request.request_id, "motiond unavailable");
            return;
        }
        active_action_request_id_.store(request.request_id);
        start_action_worker(goal_handle, request);
    }

    void accept_switch_policy(const std::shared_ptr<GoalHandle<SwitchPolicy>>& goal_handle)
    {
        const auto request = make_request(
            goal_handle->get_goal()->request_id,
            qc::ModeRequestType::SwitchPolicy,
            {}, goal_handle->get_goal()->policy_name);
        if (!submit_request(request))
        {
            finish_action_immediately<SwitchPolicy>(
                goal_handle, request.request_id, "motiond unavailable");
            return;
        }
        active_action_request_id_.store(request.request_id);
        start_action_worker(goal_handle, request);
    }

    template<typename ActionT>
    void finish_action_immediately(
        const std::shared_ptr<GoalHandle<ActionT>>& goal_handle,
        const std::uint64_t request_id,
        const std::string& message)
    {
        auto result = std::make_shared<typename ActionT::Result>();
        result->result.request_id = request_id;
        result->result.state = static_cast<std::uint8_t>(qc::ModeResultState::Failed);
        result->result.message = message;
        goal_handle->abort(result);
    }

    void handle_enter_passive(
        const qsrv::EnterPassive::Request& request,
        qsrv::EnterPassive::Response& response)
    {
        const std::uint64_t interrupted_request_id = active_action_request_id_.load();
        const auto mode_request = make_request(
            request.request_id, qc::ModeRequestType::EnterPassive);
        if (!submit_request(mode_request))
        {
            response.result.request_id = request.request_id;
            response.result.state = static_cast<std::uint8_t>(qc::ModeResultState::Failed);
            response.result.message = "motiond unavailable";
            return;
        }
        const auto result = wait_for_result(request.request_id);
        response.result = result;
        response.has_interrupted_request = false;
        if (interrupted_request_id != 0 && interrupted_request_id != request.request_id)
        {
            qmsg::ModeResult interrupted_result;
            if (wait_for_terminal_result(
                    interrupted_request_id, std::chrono::milliseconds(500), interrupted_result))
            {
                response.has_interrupted_request = true;
                response.interrupted_result = interrupted_result;
            }
        }
    }

    void handle_reset_fault(
        const qsrv::ResetFault::Request& request,
        qsrv::ResetFault::Response& response)
    {
        const auto mode_request = make_request(
            request.request_id, qc::ModeRequestType::ResetFault);
        if (!submit_request(mode_request))
        {
            response.result.request_id = request.request_id;
            response.result.state = static_cast<std::uint8_t>(qc::ModeResultState::Failed);
            response.result.message = "motiond unavailable";
            return;
        }
        response.result = wait_for_result(request.request_id);
    }

    bool wait_for_terminal_result(
        const std::uint64_t request_id,
        const std::chrono::steady_clock::duration timeout,
        qmsg::ModeResult& result)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!stopping_.load() && std::chrono::steady_clock::now() < deadline)
        {
            {
                std::lock_guard<std::mutex> lock(result_mutex_);
                const auto found = latest_results_.find(request_id);
                if (found != latest_results_.end())
                {
                    result = to_ros_result(found->second);
                    if (is_terminal(result.state))
                    {
                        return true;
                    }
                }
            }
            std::unique_lock<std::mutex> lock(result_mutex_);
            result_condition_.wait_for(lock, std::chrono::milliseconds(50));
        }
        return false;
    }

    qmsg::ModeResult wait_for_result(const std::uint64_t request_id)
    {
        qmsg::ModeResult result;
        if (wait_for_terminal_result(
                request_id, std::chrono::nanoseconds(kResultWaitTimeoutNs), result))
        {
            return result;
        }
        result.request_id = request_id;
        result.state = static_cast<std::uint8_t>(qc::ModeResultState::Failed);
        result.message = "timeout waiting for motiond result";
        return result;
    }

    void result_pump()
    {
        while (!stopping_.load())
        {
            qi::WireModeResult wire;
            bool received = false;
            while (qi::queue_pop(memory_->layout().results, wire))
            {
                qi::WireHeartbeat motion_heartbeat;
                qi::WireStateFrame state;
                const bool current =
                    qi::read_latest(memory_->layout().motion_heartbeat, motion_heartbeat) &&
                    qi::read_latest(memory_->layout().state, state) &&
                    wire.startup_id == motion_heartbeat.startup_id &&
                    wire.session_id == state.session_id;
                if (!current)
                {
                    continue;
                }
                std::lock_guard<std::mutex> lock(result_mutex_);
                latest_results_[wire.request_id] = wire;
                pending_result_events_.push_back(wire);
                if (is_terminal(wire.state) &&
                    active_action_request_id_.load() == wire.request_id)
                {
                    active_action_request_id_.store(0);
                }
                received = true;
            }
            if (received)
            {
                result_condition_.notify_all();
            }
            std::this_thread::sleep_for(kPumpInterval);
        }
    }

    void submit_start_rl_behavior()
    {
        // MotionRuntime 要求启动 RL 前已有有效 BaseCommand；键盘模式也先发布零速度。
        set_manual_command(0.0, 0.0, 0.0);
        manual_input_active_.store(true);
        publish_base_command(0.0, 0.0, 0.0, qc::CommandSource::Gamepad, 110);
        submit_keyboard_request(qc::ModeRequestType::StartBehavior, "rl_locomotion");
    }

    void submit_keyboard_request(
        const qc::ModeRequestType type,
        const std::string& behavior_name = {},
        const std::string& policy_name = {})
    {
        const std::uint64_t request_id = ++local_request_id_;
        const auto request = make_request(request_id, type, behavior_name, policy_name);
        if (!submit_request(request))
        {
            RCLCPP_WARN(get_logger(), "人工请求提交失败 request_id=%lu", request_id);
        }
    }

    void submit_backend_reset()
    {
        std::uint64_t ignored_startup = 0;
        std::uint64_t session_id = 0;
        std::int64_t ignored_timestamp = 0;
        if (!current_session(ignored_startup, session_id, ignored_timestamp))
        {
            return;
        }
        qi::WireHeartbeat heartbeat;
        if (!qi::read_latest(memory_->layout().backend_heartbeat, heartbeat))
        {
            return;
        }
        qi::WireControlRequest request;
        request.schema_version = qc::kFrameSchemaVersion;
        request.startup_id = heartbeat.startup_id;
        request.session_id = session_id;
        request.request_id = ++local_request_id_;
        // 用户按键只复位仿真姿态，不建立新会话，保持当前运动行为。
        request.type =
            static_cast<std::uint8_t>(qi::WireControlType::SimulationStateReset);
        std::lock_guard<std::mutex> lock(request_mutex_);
        if (!qi::queue_push(memory_->layout().control_requests, request))
        {
            RCLCPP_WARN(get_logger(), "后端 reset 请求队列已满");
        }
    }

    void submit_backend_pause_toggle()
    {
        std::uint64_t ignored_startup = 0;
        std::uint64_t session_id = 0;
        std::int64_t ignored_timestamp = 0;
        if (!current_session(ignored_startup, session_id, ignored_timestamp))
        {
            return;
        }
        qi::WireHeartbeat heartbeat;
        if (!qi::read_latest(memory_->layout().backend_heartbeat, heartbeat))
        {
            return;
        }
        qi::WireControlRequest request;
        request.schema_version = qc::kFrameSchemaVersion;
        request.startup_id = heartbeat.startup_id;
        request.session_id = session_id;
        request.request_id = ++local_request_id_;
        request.type = static_cast<std::uint8_t>(qi::WireControlType::PauseToggle);
        std::lock_guard<std::mutex> lock(request_mutex_);
        if (!qi::queue_push(memory_->layout().control_requests, request))
        {
            RCLCPP_WARN(get_logger(), "后端暂停请求队列已满");
        }
    }

    void keyboard_loop()
    {
        termios original{};
        if (tcgetattr(STDIN_FILENO, &original) != 0)
        {
            return;
        }
        termios raw = original;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
        {
            return;
        }
        original_termios_ = original;
        terminal_active_.store(true);
        while (!stopping_.load() && rclcpp::ok())
        {
            fd_set read_set;
            FD_ZERO(&read_set);
            FD_SET(STDIN_FILENO, &read_set);
            timeval timeout{0, static_cast<suseconds_t>(kKeyboardPollPeriod.count() * 1000)};
            const int ready = select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout);
            if (ready <= 0)
            {
                refresh_manual_command();
                continue;
            }
            char key = 0;
            if (read(STDIN_FILENO, &key, 1) != 1)
            {
                continue;
            }
            if (key == 27)
            {
                // Linux 方向键会发送 ESC [ A/B/C/D；读取并丢弃完整序列，
                // 避免把方向键误判成退出，也不让控制字符进入运动映射。
                timeval sequence_timeout{0, 10'000};
                if (select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &sequence_timeout) > 0)
                {
                    char sequence[2]{};
                    const ssize_t count = read(STDIN_FILENO, sequence, sizeof(sequence));
                    static_cast<void>(count);
                }
                continue;
            }
            process_keyboard_key(key);
        }
        restore_terminal();
    }

    void restore_terminal()
    {
        if (terminal_active_.exchange(false))
        {
            tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        }
    }

    void process_keyboard_key(const char input)
    {
        const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(input)));
        switch (key)
        {
        case '0': submit_keyboard_request(qc::ModeRequestType::GetUp); return;
        case '1': submit_start_rl_behavior(); return;
        case '2':
            submit_keyboard_request(fixed_drive_keys_enabled_
                    ? qc::ModeRequestType::StartBehavior
                    : qc::ModeRequestType::SwitchPolicy,
                fixed_drive_keys_enabled_ ? "bridge_drive" : "",
                fixed_drive_keys_enabled_ ? "" : "toggle");
            return;
        case '3':
            submit_keyboard_request(fixed_drive_keys_enabled_
                    ? qc::ModeRequestType::StartBehavior
                    : qc::ModeRequestType::SwitchPolicy,
                fixed_drive_keys_enabled_ ? "low_bar_drive" : "",
                fixed_drive_keys_enabled_ ? "" : "toggle");
            return;
        case '4':
            if (fixed_drive_keys_enabled_)
            {
                submit_keyboard_request(qc::ModeRequestType::StartBehavior, "car_drive");
            }
            return;
        case '6': submit_keyboard_request(
            qc::ModeRequestType::StartBehavior, "event_chain"); return;
        case '9': submit_keyboard_request(qc::ModeRequestType::GetDown); return;
        case 'p': submit_keyboard_request(qc::ModeRequestType::EnterPassive); return;
        case 'r': submit_backend_reset(); return;
        case '\n':
        case '\r': submit_backend_pause_toggle(); return;
        case 'n': manual_input_active_.store(!manual_input_active_.load()); return;
        case ' ': set_manual_command(0.0, 0.0, 0.0); manual_input_active_.store(true); return;
        case 'x': stopping_.store(true); rclcpp::shutdown(); return;
        case 27:
            // 方向键以 ESC [ A/B/C/D 开头；不能把 ESC 直接当作退出键。
            return;
        case 'h':
            RCLCPP_INFO(
                get_logger(),
                "0 Stand 1 RL locomotion 2/3 Switch policy or Bridge/Low-bar "
                "4 Car 6 Event chain "
                "9 Lie down P Passive "
                "R Reset Enter Pause/continue W/S A/D Q/E Command speed "
                "Space Clear command N Navigation X Exit");
            return;
        default: break;
        }
        if (current_mode_.load() != static_cast<std::uint8_t>(qc::MotionMode::Running))
        {
            return;
        }
        double vx = manual_vx_.load();
        double vy = manual_vy_.load();
        double wz = manual_wz_.load();
        if (key == 'w') vx += kKeyboardIncrement;
        else if (key == 's') vx -= kKeyboardIncrement;
        else if (key == 'a') vy += kKeyboardIncrement;
        else if (key == 'd') vy -= kKeyboardIncrement;
        else if (key == 'q') wz += kKeyboardIncrement;
        else if (key == 'e') wz -= kKeyboardIncrement;
        else return;
        set_manual_command(vx, vy, wz);
        manual_input_active_.store(true);
    }

    void render_terminal_status(const qc::MotionStatus& motion_status)
    {
        if (!terminal_ui_enabled_)
        {
            return;
        }
        const double vx = published_command_vx_.load();
        const double vy = published_command_vy_.load();
        const double wz = published_command_wz_.load();
        const bool rl_running = motion_status.mode == qc::MotionMode::Running &&
            motion_status.behavior_name == "rl_locomotion";
        const char* const policy = motion_status.policy_name.empty()
            ? "-"
            : motion_status.policy_name.c_str();
        const char* const input = manual_input_active_.load() ? "manual" : "navigation";
        std::string joy_profile;
        {
            std::lock_guard<std::mutex> lock(joy_info_mutex_);
            joy_profile = joy_profile_name_;
        }
        std::lock_guard<std::mutex> lock(terminal_output_mutex_);
        if (terminal_status_rendered_)
        {
            // 光标当前位于第二行；回到第一行后整体刷新两行状态。
            std::cout << "\033[1A";
        }
        std::cout << "\r\033[2K[Controller] ";
        if (joy_online_.load())
        {
            std::cout << "Connected";
            if (!joy_profile.empty())
            {
                std::cout << ": " << joy_profile;
            }
        }
        else
        {
            std::cout << "\033[33mNot connected\033[0m";
        }
        std::cout << "\n\r\033[2K" << std::fixed << std::setprecision(2);
        if (rl_running)
        {
            // 指令单位分别为 m/s、m/s、rad/s。
            std::cout << "RL Controller policy=" << policy
                      << " x:" << vx
                      << " y:" << vy
                      << " yaw:" << wz;
        }
        else
        {
            std::cout << "mode=" << motion_mode_name(motion_status.mode)
                      << " policy=" << policy
                      << " input=" << input
                      << " command=" << vx << "," << vy << "," << wz;
            if (!motion_status.error_message.empty())
            {
                std::cout << " error=" << motion_status.error_message;
            }
        }
        std::cout << std::flush;
        terminal_status_rendered_ = true;
    }

    void publish_status()
    {
        qi::WireHeartbeat gateway_heartbeat;
        gateway_heartbeat.startup_id = gateway_startup_id_;
        gateway_heartbeat.monotonic_ns = qi::monotonic_now_ns();
        gateway_heartbeat.online = 1;

        qi::WireStateFrame state;
        if (qi::read_latest(memory_->layout().state, state))
        {
            gateway_heartbeat.session_id = state.session_id;
            publish_diagnostic(state);
        }
        qi::publish_latest(memory_->layout().gateway_heartbeat, gateway_heartbeat);

        qi::WireMotionStatus wire_motion_status;
        qc::MotionStatus motion_status;
        if (qi::read_latest(memory_->layout().motion_status, wire_motion_status) &&
            qi::from_wire(wire_motion_status, motion_status))
        {
            qmsg::MotionStatus output;
            output.mode = static_cast<std::uint8_t>(motion_status.mode);
            output.active_source = static_cast<std::uint8_t>(motion_status.active_source);
            output.behavior_name = motion_status.behavior_name;
            output.behavior_phase = motion_status.behavior_phase;
            output.policy_name = motion_status.policy_name;
            output.error_message = motion_status.error_message;
            output.policy_ready = motion_status.policy_ready;
            output.command_limits = motion_status.command_limits;
            if (motion_status.policy_ready)
            {
                update_command_limits(motion_status.command_limits);
            }
            current_mode_.store(static_cast<std::uint8_t>(motion_status.mode));
            {
                std::lock_guard<std::mutex> lock(policy_mutex_);
                current_policy_ = motion_status.policy_name;
            }
            motion_status_publisher_->publish(output);
            render_terminal_status(motion_status);
        }

        qi::WireRobotIOStatus wire_io_status;
        qc::RobotIOStatus io_status;
        if (qi::read_latest(memory_->layout().robot_io_status, wire_io_status) &&
            qi::from_wire(wire_io_status, io_status))
        {
            qmsg::RobotIOStatus output;
            output.state = static_cast<std::uint8_t>(io_status.state);
            output.latest_state_sequence = io_status.latest_state_sequence;
            output.latest_command_sequence = io_status.latest_command_sequence;
            output.dropped_state_frames = io_status.dropped_state_frames;
            output.rejected_command_frames = io_status.rejected_command_frames;
            robot_io_status_publisher_->publish(output);
        }

        std::vector<qi::WireModeResult> events;
        {
            std::lock_guard<std::mutex> lock(result_mutex_);
            events.swap(pending_result_events_);
        }
        for (const qi::WireModeResult& event : events)
        {
            result_publisher_->publish(to_ros_result(event));
        }
    }

    void publish_diagnostic(const qi::WireStateFrame& wire)
    {
        qmsg::StateDiagnostic output;
        output.schema_version = wire.schema_version;
        output.startup_id = wire.startup_id;
        output.session_id = wire.session_id;
        output.sequence = wire.sequence;
        output.timestamp_ns = wire.timestamp_ns;
        output.joint_count = wire.joint_count;
        output.safety_state = wire.safety_state;
        output.last_accepted_command_sequence = wire.last_accepted_command_sequence;
        output.effective_command_sequence = wire.effective_command_sequence;
        output.imu_valid = wire.imu.valid != 0;
        output.imu_orientation_w = wire.imu.orientation[0];
        output.imu_orientation_x = wire.imu.orientation[1];
        output.imu_orientation_y = wire.imu.orientation[2];
        output.imu_orientation_z = wire.imu.orientation[3];
        output.imu_angular_velocity_x = wire.imu.angular_velocity[0];
        output.imu_angular_velocity_y = wire.imu.angular_velocity[1];
        output.imu_angular_velocity_z = wire.imu.angular_velocity[2];
        output.imu_linear_acceleration_x = wire.imu.linear_acceleration[0];
        output.imu_linear_acceleration_y = wire.imu.linear_acceleration[1];
        output.imu_linear_acceleration_z = wire.imu.linear_acceleration[2];
        diagnostic_publisher_->publish(output);
    }

    std::string shared_memory_name_{};
    std::string joy_topic_{"/joy"};
    std::int64_t command_timeout_ns_{kDefaultCommandTimeoutNs};
    std::int64_t joy_timeout_ns_{kDefaultJoyTimeoutNs};
    bool keyboard_enabled_{true};
    bool fixed_drive_keys_enabled_{false};
    bool joy_require_connection_frame_{true};
    bool terminal_ui_enabled_{false};
    std::uint64_t gateway_startup_id_{0};
    std::atomic<std::uint64_t> base_command_sequence_{0};
    std::atomic<std::uint64_t> local_request_id_{1};
    std::atomic<std::uint64_t> active_action_request_id_{0};
    std::atomic<std::uint8_t> current_mode_{static_cast<std::uint8_t>(qc::MotionMode::Passive)};
    std::atomic<bool> joy_online_{false};
    std::mutex joy_info_mutex_{};
    std::string joy_profile_name_{};
    std::atomic<bool> manual_input_active_{false};
    std::atomic<double> manual_vx_{0.0};
    std::atomic<double> manual_vy_{0.0};
    std::atomic<double> manual_wz_{0.0};
    std::array<std::atomic<double>, 3> command_limits_{
        kFallbackCommandLimits[0], kFallbackCommandLimits[1], kFallbackCommandLimits[2]};
    // 最近一次实际发布给 MotionRuntime 的机体速度指令，单位为 m/s、m/s、rad/s。
    std::atomic<double> published_command_vx_{0.0};
    std::atomic<double> published_command_vy_{0.0};
    std::atomic<double> published_command_wz_{0.0};
    std::atomic<double> navigation_vx_{0.0};
    std::atomic<double> navigation_vy_{0.0};
    std::atomic<double> navigation_wz_{0.0};
    std::atomic<std::int64_t> last_cmd_vel_ns_{0};
    std::atomic<std::int64_t> last_joy_ns_{0};
    JoyEdgeState last_joy_buttons_{};
    termios original_termios_{};
    std::atomic<bool> terminal_active_{false};
    bool terminal_status_rendered_{false};
    std::mutex command_mutex_{};
    std::mutex policy_mutex_{};
    std::string current_policy_{};
    std::unique_ptr<qi::SharedMemory> memory_{};
    std::atomic<bool> stopping_{false};
    std::mutex terminal_output_mutex_{};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_{};
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_{};
    rclcpp::Publisher<qmsg::MotionStatus>::SharedPtr motion_status_publisher_{};
    rclcpp::Publisher<qmsg::RobotIOStatus>::SharedPtr robot_io_status_publisher_{};
    rclcpp::Publisher<qmsg::StateDiagnostic>::SharedPtr diagnostic_publisher_{};
    rclcpp::Publisher<qmsg::ModeResult>::SharedPtr result_publisher_{};
    rclcpp::TimerBase::SharedPtr status_timer_{};

    rclcpp_action::Server<GetUp>::SharedPtr get_up_server_{};
    rclcpp_action::Server<GetDown>::SharedPtr get_down_server_{};
    rclcpp_action::Server<StartBehavior>::SharedPtr start_behavior_server_{};
    rclcpp_action::Server<SwitchPolicy>::SharedPtr switch_policy_server_{};
    rclcpp::Service<qsrv::EnterPassive>::SharedPtr enter_passive_service_{};
    rclcpp::Service<qsrv::ResetFault>::SharedPtr reset_fault_service_{};

    std::thread result_thread_{};
    std::thread keyboard_thread_{};
    std::mutex request_mutex_{};
    std::mutex result_mutex_{};
    std::condition_variable result_condition_{};
    std::unordered_map<std::uint64_t, qi::WireModeResult> latest_results_{};
    std::vector<qi::WireModeResult> pending_result_events_{};
    std::mutex worker_mutex_{};
    std::vector<std::thread> workers_{};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<Ros2Gateway>();
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
        executor.remove_node(node);
    }
    catch (const std::exception& exception)
    {
        std::cerr << "ros2_gateway 启动失败: " << exception.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
