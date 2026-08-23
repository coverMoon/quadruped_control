/**
 * @file remote_robot_io.hpp
 * @brief 定义 motiond 通过共享内存访问独立后端进程的 RobotIO 适配器。
 */

#pragma once

#include "quadruped/core/robot_io.hpp"
#include "quadruped/core/robot_model.hpp"
#include "quadruped/ipc/shared_memory.hpp"

#include <string>

namespace quadruped::ipc
{

class RemoteRobotIO final : public core::RobotIO
{
public:
    RemoteRobotIO(SharedMemory& memory, core::RobotModel model);

    core::RobotIOCode read_latest(core::StateFrame& frame) override;
    core::RobotIOCode submit(const core::CommandFrame& frame) override;
    core::RobotIOStatus status() const noexcept override;

    [[nodiscard]] bool backend_online() const noexcept;

private:
    [[nodiscard]] bool read_backend_heartbeat(WireHeartbeat& heartbeat) const noexcept;

    SharedMemory& memory_;
    core::RobotModel model_{};
    mutable core::RobotIOStatus cached_status_{};
    mutable WireHeartbeat cached_backend_heartbeat_{};
    mutable bool has_cached_backend_heartbeat_{false};
    core::StateFrame cached_state_{};
    bool has_cached_state_{false};
};

}  // 命名空间 quadruped::ipc
