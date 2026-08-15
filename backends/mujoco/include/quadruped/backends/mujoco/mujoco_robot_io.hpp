/**
 * @file mujoco_robot_io.hpp
 * @brief 定义基于 MuJoCo 的 RobotIO 后端，负责 reset 和生成 StateFrame。
 */

#pragma once

#include "quadruped/backends/mujoco/mujoco_model.hpp"
#include "quadruped/core/robot_io.hpp"
#include "quadruped/core/robot_model.hpp"
#include "quadruped/core/types.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace quadruped::backends::mujoco
{

// 基于 MuJoCo 的 RobotIO 后端。
// 当前阶段（M1-4）实现 reset、StateFrame 生成、命令校验与显式单物理步进。
class MujocoRobotIO final : public core::RobotIO
{
public:
    // create() 的返回结果：成功时 io 非空，失败时 error_message 含有可读原因。
    struct CreateResult
    {
        std::unique_ptr<MujocoRobotIO> io{};
        std::string error_message{};

        [[nodiscard]] bool ok() const noexcept
        {
            return io != nullptr;
        }
    };

    // reset() 的返回结果：成功时 code 为 Ok，失败时 code 与 error_message 说明原因。
    struct ResetResult
    {
        core::RobotIOCode code{core::RobotIOCode::Fault};
        std::string error_message{};

        [[nodiscard]] bool ok() const noexcept
        {
            return code == core::RobotIOCode::Ok;
        }
    };

    MujocoRobotIO(const MujocoRobotIO&) = delete;
    MujocoRobotIO& operator=(const MujocoRobotIO&) = delete;
    ~MujocoRobotIO() override = default;

    // 加载场景并按值保存一份 RobotModel 副本，保证生命周期独立于调用方对象。
    // startup_id 为 0 时拒绝创建；加载失败时返回空 io 和可读原因。
    static CreateResult create(
        const std::string& scene_path,
        const core::RobotModel& robot_model,
        std::uint64_t startup_id);

    // 重置到 XML 中名为 default_pose 的 keyframe，并生成新会话的第一份 StateFrame。
    // session_id 必须非零；已建立会话后重复使用同一 session_id 会被拒绝。
    ResetResult reset(std::uint64_t session_id);

    // RobotIO 接口实现。
    core::RobotIOCode read_latest(core::StateFrame& frame) override;
    core::RobotIOCode submit(const core::CommandFrame& frame) override;
    core::RobotIOStatus status() const noexcept override;

    // MuJoCo 后端专用的显式单物理步接口。
    // 应用当前最新有效命令（或 Disabled 回退）、调用一次 mj_step、
    // 从步进后的 mjData 生成新的 StateFrame。失败时进入 Fault，不发布不完整状态。
    core::RobotIOCode step();

    // 非拥有的 MuJoCo 数据访问，指针生命周期与本对象一致。
    // 仅用于测试注入异常数据；正式运行代码不得依赖外部直接修改 mjData。
    mjData* raw_data() noexcept
    {
        return model_.raw_data();
    }

    const mjData* raw_data() const noexcept
    {
        return model_.raw_data();
    }

    // 仅供测试在注入异常 mjData 后触发一次状态刷新的最小入口。
    // 帧的 session_id 只能来自当前已建立的会话，调用方不能指定；
    // 会话未建立（尚未成功 reset）时直接失败，不会发布任何状态。
    // 正式运行代码不得调用。
    std::string refresh_for_test();

private:
    // 只能通过 create() 完成全部检查后构造。
    MujocoRobotIO(core::RobotModel robot_model, MujocoModel model, std::uint64_t startup_id)
        : robot_model_(std::move(robot_model)),
          model_(std::move(model)),
          startup_id_(startup_id)
    {
        status_.state = core::RobotIOState::Paused;
    }

    // 从当前 mjData 重新生成并发布最新状态，序号使用当前 sequence_ 并自增。
    // 仅供 reset 和后续 M1-4 的步进内部复用；session_id 由内部调用方给出，
    // 外部无法经由公共 API 调用。失败时不修改已发布的上一份状态，
    // 并把后端状态置为 Fault，返回可读原因（空串表示成功）。
    std::string refresh_latest_state(std::uint64_t session_id);

    core::RobotModel robot_model_{};
    MujocoModel model_;

    // 程序每次启动生成的非零标识，写入每帧 header；0 表示尚未初始化。
    std::uint64_t startup_id_{0};

    // 当前已建立的会话编号；0 表示尚未 reset，此时没有可用状态。
    std::uint64_t session_id_{0};

    // 下一个待发布帧的序号，会话内单调递增；每次 reset 开始时重置为 1。
    std::uint64_t sequence_{0};

    core::StateFrame latest_state_{};
    bool has_state_{false}; // 是否已经生成过至少一份完整状态。
    bool latest_read_{false}; // 最新状态是否已经被读取过，用于统计丢帧。
    core::RobotIOStatus status_{};

    // 最近一次成功 submit 的完整命令；没有成功接受的命令时 has_command_ 为 false。
    core::CommandFrame latest_command_{};
    bool has_command_{false};
};

}  // 命名空间 quadruped::backends::mujoco
