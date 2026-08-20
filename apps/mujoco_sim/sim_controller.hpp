/**
 * @file sim_controller.hpp
 * @brief 定义 mujoco_sim 的仿真调度状态：物理步进、控制周期和按键请求。
 */

#pragma once

#include "quadruped/backends/mujoco/mujoco_robot_io.hpp"
#include "quadruped/core/core.hpp"
#include "quadruped/motion/motion_runtime.hpp"

#include "sim_input.hpp"

#include <cstdint>
#include <string>

namespace quadruped::apps::mujoco_sim
{

// 主循环持有的仿真控制状态，统一调度 2 ms 物理步进、5 ms 控制周期和按键请求，
// 避免主循环直接搬运多项调度变量。
class SimController
{
public:
    SimController(
        quadruped::backends::mujoco::MujocoRobotIO& io,
        quadruped::motion::MotionRuntime& runtime,
        core::ControllerConfig config);

    // 建立新会话（会话号递增）并清除未完成请求；失败时返回可读原因，空串表示成功。
    std::string reset_new_session();

    // 把终端输入转换为速度命令和一次性运动请求。
    void apply_input(const SimInput& input);

    // 翻转暂停状态；继续仿真时重新对齐控制调度，避免补跑暂停期间落下的周期。
    void toggle_pause() noexcept;

    [[nodiscard]] bool paused() const noexcept
    {
        return paused_;
    }

    // 推进一个物理步；到达控制周期时先运行 MotionRuntime。
    // 返回 false 表示后端进入不可恢复状态。
    bool step();

    [[nodiscard]] const quadruped::motion::MotionUpdateOutput& last_output() const noexcept
    {
        return last_output_;
    }

    [[nodiscard]] double sim_time() const noexcept
    {
        return sim_time_;
    }

    [[nodiscard]] std::uint64_t update_sequence() const noexcept
    {
        return update_sequence_;
    }

    // 渲染所需的只读 MuJoCo 数据访问；正式代码不得通过它修改 mjData。
    mjData* data() noexcept
    {
        return io_.raw_data();
    }

private:
    quadruped::backends::mujoco::MujocoRobotIO& io_;
    quadruped::motion::MotionRuntime& runtime_;
    core::ControllerConfig config_;

    // 控制周期调度：从最新状态时间起按配置周期触发 MotionRuntime。
    core::Nanoseconds next_control_ns_{-1};
    core::Nanoseconds latest_state_ns_{0};
    quadruped::motion::MotionUpdateOutput last_output_{};
    std::uint64_t update_sequence_{0};
    double sim_time_{0.0};

    // 暂停状态和当前会话号；会话号从 1 开始，每次 reset 递增。
    bool paused_{false};
    std::uint64_t session_id_{0};

    // 终端按键产生的待处理请求只向 MotionRuntime 提交一次。
    std::uint64_t next_request_id_{1};
    core::ModeRequest pending_request_{};
    bool has_pending_request_{false};

    // 界面持续刷新一份短有效期速度命令；松开按键后显式发送零速度。
    std::uint64_t base_command_sequence_{0};
    core::BaseCommand base_command_{};
};

}  // 命名空间 quadruped::apps::mujoco_sim
