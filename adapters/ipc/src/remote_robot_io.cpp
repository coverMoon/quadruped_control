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

bool RemoteRobotIO::backend_online() const noexcept
{
    WireHeartbeat heartbeat;
    if (!ipc::read_latest(memory_.layout().backend_heartbeat, heartbeat) || heartbeat.online == 0)
    {
        return false;
    }
    const std::int64_t now = monotonic_now_ns();
    return heartbeat.monotonic_ns > 0 && now >= heartbeat.monotonic_ns &&
        now - heartbeat.monotonic_ns <= kHeartbeatTimeoutNs;
}

core::RobotIOCode RemoteRobotIO::read_latest(core::StateFrame& frame)
{
    if (!backend_online())
    {
        cached_status_.state = core::RobotIOState::Disconnected;
        return core::RobotIOCode::Disconnected;
    }

    WireHeartbeat heartbeat;
    WireStateFrame wire;
    if (!ipc::read_latest(memory_.layout().backend_heartbeat, heartbeat) ||
        !ipc::read_latest(memory_.layout().state, wire))
    {
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
    cached_status_.state = core::RobotIOState::Ready;
    cached_status_.latest_state_sequence = frame.header.sequence;
    return core::RobotIOCode::Ok;
}

core::RobotIOCode RemoteRobotIO::submit(const core::CommandFrame& frame)
{
    if (!backend_online())
    {
        cached_status_.state = core::RobotIOState::Disconnected;
        return core::RobotIOCode::Disconnected;
    }
    WireHeartbeat heartbeat;
    if (!ipc::read_latest(memory_.layout().backend_heartbeat, heartbeat) ||
        frame.header.startup_id != heartbeat.startup_id ||
        frame.header.session_id != heartbeat.session_id ||
        !core::validate(frame, model_, frame.header.timestamp_ns).ok())
    {
        ++cached_status_.rejected_command_frames;
        return core::RobotIOCode::InvalidFrame;
    }
    publish_latest(memory_.layout().command, to_wire(frame));
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
