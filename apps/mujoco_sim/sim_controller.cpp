/**
 * @file sim_controller.cpp
 * @brief 实现 mujoco_sim 的物理步进、控制周期调度和按键请求转换。
 */

#include "sim_controller.hpp"

#include <utility>

namespace qc = quadruped::core;
namespace qm = quadruped::motion;

namespace quadruped::apps::mujoco_sim
{

SimController::SimController(
    quadruped::backends::mujoco::MujocoRobotIO& io,
    qm::MotionRuntime& runtime,
    qc::ControllerConfig config)
    : io_(io), runtime_(runtime), config_(std::move(config))
{
}

std::string SimController::reset_new_session()
{
    const auto result = io_.reset(++session_id_);
    if (!result.ok())
    {
        return result.error_message;
    }
    has_pending_request_ = false;
    next_control_ns_ = -1;
    last_output_ = {};

    // 立即同步新会话时间并刷新显示快照，避免暂停期间 reset 后把旧会话时间
    // 写入后续请求（旧时间会让请求在新会话中因“未来时间戳”被持续拒绝）。
    // 读取失败时清零时间并返回错误，防止旧时间残留。
    core::StateFrame state;
    if (io_.read_latest(state) != core::RobotIOCode::Ok)
    {
        latest_state_ns_ = 0;
        sim_time_ = 0.0;
        return "reset succeeded but the new session state cannot be read";
    }
    latest_state_ns_ = state.header.timestamp_ns;
    sim_time_ = static_cast<double>(latest_state_ns_) / 1.0e9;
    return {};
}

void SimController::apply_input(const SimInput& input)
{
    qc::ModeRequestType type;
    if (input.getup)
    {
        type = qc::ModeRequestType::GetUp;
    }
    else if (input.getdown)
    {
        type = qc::ModeRequestType::GetDown;
    }
    else if (input.enter_passive)
    {
        type = qc::ModeRequestType::EnterPassive;
    }
    else
    {
        return;
    }

    pending_request_ = qc::ModeRequest{};
    pending_request_.request_id = next_request_id_++;
    pending_request_.timestamp_ns = latest_state_ns_;
    pending_request_.type = type;
    has_pending_request_ = true;
}

void SimController::toggle_pause() noexcept
{
    paused_ = !paused_;
    if (!paused_)
    {
        next_control_ns_ = -1;
    }
}

bool SimController::step()
{
    qc::StateFrame state;
    if (io_.read_latest(state) != qc::RobotIOCode::Ok)
    {
        return false;
    }
    latest_state_ns_ = state.header.timestamp_ns;
    sim_time_ = static_cast<double>(latest_state_ns_) / 1.0e9;

    if (next_control_ns_ < 0)
    {
        next_control_ns_ = latest_state_ns_;
    }
    if (latest_state_ns_ >= next_control_ns_)
    {
        qm::MotionUpdateInput input;
        input.now_ns = latest_state_ns_;
        input.request = has_pending_request_ ? &pending_request_ : nullptr;
        last_output_ = runtime_.update(io_, input);
        next_control_ns_ += config_.control_period_ns;
    }
    return io_.step() == qc::RobotIOCode::Ok;
}

}  // 命名空间 quadruped::apps::mujoco_sim
