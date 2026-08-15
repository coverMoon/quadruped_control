/**
 * @file robot_io.hpp
 * @brief 定义运动控制访问机器人状态和提交关节命令的统一接口。
 */

#pragma once

#include "quadruped/core/types.hpp"

#include <cstdint>

namespace quadruped::core
{

// 单次 RobotIO 操作的结果；调用方必须显式处理非 Ok 状态。
enum class RobotIOCode : std::uint8_t
{
    Ok = 0,             // 操作成功。
    NoData = 1,         // 当前还没有可读取的完整状态。
    Disconnected = 2,   // 后端未连接或已经断开。
    InvalidFrame = 3,   // 帧字段、维度或数值不合法。
    Rejected = 4,       // 帧合法，但因当前状态或权限被拒绝。
    Fault = 5,          // 后端内部故障，不能继续正常读写。
};

// RobotIO 后端当前的整体运行状态。
enum class RobotIOState : std::uint8_t
{
    Disconnected = 0, // 后端尚未建立连接或连接已经断开。
    Ready = 1,        // 后端可以读取状态并接受合法命令。
    Paused = 2,       // 后端保持连接，但暂时不推进仿真或执行命令。
    Fault = 3,        // 后端发生需要干预的故障。
};

// 供诊断和低频状态显示使用的 RobotIO 统计快照。
struct RobotIOStatus
{
    RobotIOState state{RobotIOState::Disconnected};

    // 后端最近生成的 StateFrame 序号；0 表示尚未生成状态。
    std::uint64_t latest_state_sequence{0};

    // 后端最近接受的 CommandFrame 序号；0 表示尚未接受命令。
    std::uint64_t latest_command_sequence{0};

    // 因最新值覆盖而没有被读取的历史状态帧总数。
    std::uint64_t dropped_state_frames{0};

    // 因校验、过期或运行状态不允许而拒绝的命令帧总数。
    std::uint64_t rejected_command_frames{0};
};

class RobotIO
{
public:
    virtual ~RobotIO() = default;

    // 返回最新的一份完整状态并写入 frame。没有数据时返回 NoData，frame 内容不作保证。
    // 实现不能让 MotionRuntime 逐帧消费已经过时的状态队列。
    virtual RobotIOCode read_latest(StateFrame& frame) = 0;

    // 提交一份完整命令。更新的有效命令可以覆盖尚未执行的旧命令；实现必须在使用前
    // 检查模型、会话、序号、有效期和数值范围。
    virtual RobotIOCode submit(const CommandFrame& frame) = 0;

    // 返回无需阻塞即可取得的后端状态快照。
    virtual RobotIOStatus status() const noexcept = 0;
};

}  // 命名空间 quadruped::core
