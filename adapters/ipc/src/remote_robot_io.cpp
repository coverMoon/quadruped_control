/**
 * @file remote_robot_io.cpp
 * @brief 实现共享内存 RobotIO 代理的最新状态读取、命令提交和故障判定。
 */

#include "quadruped/ipc/remote_robot_io.hpp"

#include "quadruped/core/validation.hpp"
#include "quadruped/ipc/conversions.hpp"

namespace quadruped::ipc
{
namespace
{

constexpr std::int64_t kHeartbeatTimeoutNs = 500'000'000;

}  // 匿名命名空间

RemoteRobotIO::RemoteRobotIO(SharedMemory& memory, core::RobotModel model)
    : memory_(memory), model_(std::move(model))
{
}

bool RemoteRobotIO::read_backend_heartbeat(WireHeartbeat& heartbeat) const noexcept
{
    WireHeartbeat latest;
    if (ipc::read_latest(memory_.layout().backend_heartbeat, latest))
    {
        cached_backend_heartbeat_ = latest;
        has_cached_backend_heartbeat_ = true;
    }
    if (!has_cached_backend_heartbeat_)
    {
        return false;
    }
    heartbeat = cached_backend_heartbeat_;
    return true;
}

bool RemoteRobotIO::backend_online() const noexcept
{
    if (memory_.layout().ready.load(std::memory_order_acquire) != 1)
    {
        return false;
    }
    WireHeartbeat heartbeat;
    if (!read_backend_heartbeat(heartbeat) || heartbeat.online == 0)
    {
        return false;
    }
    const std::int64_t now = monotonic_now_ns();
    return heartbeat.monotonic_ns > 0 && now >= heartbeat.monotonic_ns &&
        now - heartbeat.monotonic_ns <= kHeartbeatTimeoutNs;
}

core::RobotIOCode RemoteRobotIO::read_latest(core::StateFrame& frame)
{
    std::uint64_t ignored_version = 0;
    return read_latest(frame, ignored_version);
}

core::RobotIOCode RemoteRobotIO::read_latest(
    core::StateFrame& frame,
    std::uint64_t& version)
{
    version = 0;
    WireHeartbeat heartbeat;
    if (!read_backend_heartbeat(heartbeat) || heartbeat.online == 0)
    {
        cached_status_.state = core::RobotIOState::Disconnected;
        return core::RobotIOCode::Disconnected;
    }
    const std::int64_t now = monotonic_now_ns();
    if (heartbeat.monotonic_ns <= 0 || now < heartbeat.monotonic_ns ||
        now - heartbeat.monotonic_ns > kHeartbeatTimeoutNs)
    {
        cached_status_.state = core::RobotIOState::Disconnected;
        return core::RobotIOCode::Disconnected;
    }

    WireStateFrame wire;
    std::uint64_t current_version = 0;
    if (!ipc::read_latest(memory_.layout().state, wire, &current_version))
    {
        // 发布者短暂占用状态槽时复用同一 backend 会话内最后一帧。
        // heartbeat 的 500 ms 时限仍会让真正断线进入安全退路。
        if (has_cached_state_ &&
            cached_state_.header.startup_id == heartbeat.startup_id &&
            cached_state_.header.session_id == heartbeat.session_id)
        {
            frame = cached_state_;
            version = cached_state_version_;
            cached_status_.state = core::RobotIOState::Ready;
            cached_status_.latest_state_sequence = frame.header.sequence;
            return core::RobotIOCode::Ok;
        }
        cached_status_.state = core::RobotIOState::Paused;
        return core::RobotIOCode::NoData;
    }
    if (wire.startup_id != heartbeat.startup_id || wire.session_id != heartbeat.session_id ||
        !from_wire(wire, frame) ||
        !core::validate(frame, model_, frame.header.timestamp_ns).ok())
    {
        cached_status_.state = core::RobotIOState::Fault;
        return core::RobotIOCode::InvalidFrame;
    }
    cached_state_ = frame;
    cached_state_version_ = current_version;
    has_cached_state_ = true;
    version = current_version;
    cached_status_.state = core::RobotIOState::Ready;
    cached_status_.latest_state_sequence = frame.header.sequence;
    return core::RobotIOCode::Ok;
}

core::RobotIOCode RemoteRobotIO::submit(const core::CommandFrame& frame)
{
    WireHeartbeat heartbeat;
    if (!read_backend_heartbeat(heartbeat) || heartbeat.online == 0)
    {
        cached_status_.state = core::RobotIOState::Disconnected;
        return core::RobotIOCode::Disconnected;
    }
    const std::int64_t now = monotonic_now_ns();
    if (heartbeat.monotonic_ns <= 0 || now < heartbeat.monotonic_ns ||
        now - heartbeat.monotonic_ns > kHeartbeatTimeoutNs)
    {
        cached_status_.state = core::RobotIOState::Disconnected;
        return core::RobotIOCode::Disconnected;
    }
    if (frame.header.startup_id != heartbeat.startup_id ||
        frame.header.session_id != heartbeat.session_id ||
        !core::validate(frame, model_, frame.header.timestamp_ns).ok())
    {
        ++cached_status_.rejected_command_frames;
        return core::RobotIOCode::InvalidFrame;
    }
    publish_latest_and_notify(
        memory_.layout().command,
        to_wire(frame),
        memory_.layout().backend_event);
    cached_status_.latest_command_sequence = frame.header.sequence;
    return core::RobotIOCode::Ok;
}

core::RobotIOStatus RemoteRobotIO::status() const noexcept
{
    WireRobotIOStatus wire;
    core::RobotIOStatus status;
    if (ipc::read_latest(memory_.layout().robot_io_status, wire) && from_wire(wire, status))
    {
        cached_status_ = status;
    }
    if (!backend_online())
    {
        cached_status_.state = core::RobotIOState::Disconnected;
    }
    return cached_status_;
}

}  // 命名空间 quadruped::ipc
