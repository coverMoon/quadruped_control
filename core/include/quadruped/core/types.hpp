/**
 * @file types.hpp
 * @brief 定义控制系统跨模块传递的公共数据结构和枚举。
 */

#pragma once

#include "quadruped/core/constants.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace quadruped::core
{

// 关节角色表示它在机器人中的用途，不等同于 URDF/MJCF 中的源关节类型。
// 例如，训练模型可以用大位置范围的 revolute 表示轮子，
// 加载后仍应归一化为 Wheel。
enum class JointRole : std::uint8_t
{
    Leg = 0,       // 髋、大腿和小腿等腿部关节。
    Wheel = 1,     // 轮足机器人的驱动轮关节。
    Auxiliary = 2, // 机械臂或以后增加的其他关节。
};

// 每个关节命令都显式携带控制模式，不能通过 KP、KD 是否为零推断模式。
enum class ControlMode : std::uint8_t
{
    Disabled = 0,       // 不输出主动控制命令。
    Damping = 1,        // 只使用速度阻尼抑制关节运动。
    JointImpedance = 2, // 使用位置、速度、KP、KD 和前馈力矩的关节阻抗控制。
    Velocity = 3,       // 以目标速度为主要控制量。
    Torque = 4,         // 以前馈力矩为主要控制量。
};

// 标记机体速度命令来自哪个上层模块，用于优先级选择、超时和状态显示。
enum class CommandSource : std::uint8_t
{
    None = 0,       // 当前运动不由外部连续速度命令触发。
    Gamepad = 1,    // 本地手柄输入。
    Navigation = 2, // ROS 2 导航或路径跟踪输入。
    Remote = 3,     // 远程上位机或调试界面输入。
    Test = 4,       // 自动测试或开发工具输入。
};

// MotionRuntime 当前正在执行的运动阶段。
enum class MotionMode : std::uint8_t
{
    Passive = 0, // 保持阻尼或失能，不执行主动姿态运动。
    GetUp = 1,   // 从当前姿态插值到站立准备姿态。
    Stand = 2,   // 保持稳定站立，尚未运行其他运动功能。
    Running = 3, // 正在运行 RL 行走、固定姿态行驶或事件链等功能。
    GetDown = 4, // 从站立姿态平滑回到趴下姿态。
};

// 上层一次性请求的动作类型；请求与连续更新的 BaseCommand 分开处理。
enum class ModeRequestType : std::uint8_t
{
    EnterPassive = 0,  // 请求立即退出主动运动并进入被动状态。
    GetUp = 1,         // 请求执行起立流程。
    Stand = 2,         // 请求进入稳定站立。
    StartBehavior = 3, // 请求运行 behavior_name 指定的运动功能。
    GetDown = 4,       // 请求执行趴下流程。
    SwitchPolicy = 5,  // 请求切换到 policy_name 指定的策略。
    ResetFault = 6,    // 请求清除允许由软件复位的锁存故障。
};

// ModeRequest 从接收到结束的处理状态。
enum class ModeResultState : std::uint8_t
{
    Accepted = 0,  // 请求已通过前置检查，但尚未开始或完成。
    Rejected = 1,  // 请求因当前状态或参数不合法而未执行。
    Running = 2,   // 请求对应的动作正在执行。
    Completed = 3, // 请求已经成功完成。
    Failed = 4,    // 请求开始执行后因运行错误而失败。
};

// 最终执行侧报告的安全状态，MotionRuntime 只能读取，不能越过它强制控制。
enum class SafetyState : std::uint8_t
{
    Unknown = 0,        // 尚未获得可信的执行侧安全状态。
    Damping = 1,        // 执行侧只允许阻尼输出。
    ControlEnabled = 2, // 执行侧允许接受正常关节命令。
    Fault = 3,          // 执行侧检测到故障并拒绝正常控制。
    EmergencyStop = 4,  // 急停已经触发，只能通过明确流程解除。
};

// StateFrame 和 CommandFrame 共用的身份、顺序和时间信息。
struct FrameHeader
{
    // 公共帧格式版本，用于拒绝字段含义不兼容的数据。
    std::uint32_t schema_version{kFrameSchemaVersion};

    // 程序每次启动生成一个非零新值，用于识别对端重启后的旧帧；0 表示尚未初始化。
    std::uint64_t startup_id{0};

    // 当前控制会话编号；控制权重新建立后必须更换，0 表示尚未建立控制会话。
    std::uint64_t session_id{0};

    // 同一启动和会话内单调递增的帧编号；0 表示尚未发布第一帧。
    std::uint64_t sequence{0};

    // 生成完整帧时的单调时钟时间，单位为 ns。
    Nanoseconds timestamp_ns{0};

};

// 一个统一关节在 StateFrame 生成时的反馈状态。
struct JointState
{
    // 关节输出轴位置，单位为 rad。
    double position{0.0};

    // 关节输出轴速度，单位为 rad/s。
    double velocity{0.0};

    // 关节输出轴估算力矩，单位为 N·m。
    double effort{0.0};

    // 电机或关节执行器温度，单位为摄氏度。
    double temperature_c{0.0};

    // 底层归一化后的故障位或错误编号；0 表示没有已知错误。
    std::uint32_t error_code{0};

    // 从该关节最后一次实际更新到 StateFrame 生成时经过的时间，单位为 ns。
    Nanoseconds age_ns{0};

    // 底层通信链路当前是否仍能联系到该执行器。
    bool online{false};

    // 本帧数值是否通过底层完整性检查；在线不代表数据一定有效。
    bool valid{false};
};

// 以机体坐标系为基础的惯性测量状态。
struct ImuState
{
    // 机体相对世界坐标系的姿态四元数，顺序固定为 w、x、y、z。
    std::array<double, 4> orientation{1.0, 0.0, 0.0, 0.0};

    // 机体坐标系角速度，顺序为 X、Y、Z，单位为 rad/s。
    std::array<double, 3> angular_velocity{0.0, 0.0, 0.0};

    // 机体坐标系线加速度，顺序为 X、Y、Z，单位为 m/s²。
    std::array<double, 3> linear_acceleration{0.0, 0.0, 0.0};

    // 从 IMU 最后一次更新到 StateFrame 生成时经过的时间，单位为 ns。
    Nanoseconds age_ns{0};

    // 姿态、角速度和加速度是否通过底层完整性检查。
    bool valid{false};
};

// RobotIO 向 MotionRuntime 提供的一份完整机器人状态快照。
struct StateFrame
{
    // 标识本帧所属的程序启动、控制会话、机器人和生成时间。
    FrameHeader header{};

    // joints 数组中从下标 0 开始有效的元素数量，必须与 RobotModel 一致。
    std::size_t joint_count{0};

    // 按 RobotModel 统一关节顺序排列的反馈状态。
    std::array<JointState, kMaxJoints> joints{};
    ImuState imu{};

    // 最终执行侧当前安全状态，优先级高于 MotionRuntime 的运动模式。
    SafetyState safety_state{SafetyState::Unknown};

    // 最终执行侧最近接受的 CommandFrame 序号，不代表已经发送给执行器。
    std::uint64_t last_accepted_command_sequence{0};

    // 最终执行侧实际应用的命令序号，用于识别安全覆盖、限幅或执行延迟。
    std::uint64_t effective_command_sequence{0};
};

// MotionRuntime 对一个统一关节提出的控制目标。JointImpedance 使用常见 MIT 形式：
// tau = kp * (target_position - position)
//     + kd * (target_velocity - velocity)
//     + feedforward_effort。
struct JointCommand
{
    ControlMode mode{ControlMode::Disabled};

    // 目标关节位置，单位为 rad；仅在相应控制模式下使用。
    double target_position{0.0};

    // 目标关节速度，单位为 rad/s；仅在相应控制模式下使用。
    double target_velocity{0.0};

    // 位置比例增益，单位为 N·m/rad。
    double kp{0.0};

    // 速度阻尼增益，单位为 N·m·s/rad。
    double kd{0.0};

    // 叠加到反馈控制上的关节输出轴前馈力矩，单位为 N·m。
    double feedforward_effort{0.0};
};

// MotionRuntime 在一个控制周期内请求执行的完整关节命令。
struct CommandFrame
{
    // 标识本帧所属的程序启动、控制会话、机器人和生成时间。
    FrameHeader header{};

    // 命令失效的单调时钟时间，单位为 ns；执行侧不得使用已经过期的命令。
    Nanoseconds expires_at_ns{0};

    // 当前关节目标实际生成的时间；重新包装 CommandFrame 不得改变该值。
    Nanoseconds target_generated_at_ns{0};

    // 当前关节目标的硬过期时间；不得晚于此时继续执行目标。
    Nanoseconds target_expires_at_ns{0};

    // joints 数组中从下标 0 开始有效的元素数量，必须与 RobotModel 一致。
    std::size_t joint_count{0};

    // 按 RobotModel 统一关节顺序排列的控制目标。
    std::array<JointCommand, kMaxJoints> joints{};

    // 生成该命令时 MotionRuntime 所处的运动阶段，用于诊断和安全策略。
    MotionMode motion_mode{MotionMode::Passive};

    // 触发当前运动输出的上层速度命令来源；无外部来源时可以为 None。
    CommandSource source{CommandSource::None};
};

// 手柄、导航或远程程序给出的连续机体速度命令。
struct BaseCommand
{
    // 同一命令来源内单调递增的编号，用于拒绝倒退或重复数据。
    std::uint64_t sequence{0};

    // 命令生成时的单调时钟时间，单位为 ns。
    Nanoseconds timestamp_ns{0};

    // 命令失效的单调时钟时间，单位为 ns；过期后速度应自动归零。
    Nanoseconds expires_at_ns{0};
    // 发布该命令的上层来源；有效 BaseCommand 不能使用 None。
    CommandSource source{CommandSource::None};

    // 多个来源同时有效时的选择优先级，范围为 0～255，数值越大优先级越高。
    std::uint8_t priority{0};

    // 机体坐标系 +X 方向线速度，单位为 m/s。
    double vx{0.0};

    // 机体坐标系 +Y 方向线速度，单位为 m/s。
    double vy{0.0};

    // 绕机体坐标系 +Z 轴的角速度，单位为 rad/s。
    double wz{0.0};
};

// 需要可靠执行并返回结果的一次性运动模式请求。
struct ModeRequest
{
    // 请求方生成的非零唯一编号，用于去重并关联 ModeResult。
    std::uint64_t request_id{0};

    // 请求生成时的单调时钟时间，单位为 ns。
    Nanoseconds timestamp_ns{0};

    // 希望 MotionRuntime 执行的一次性动作。
    ModeRequestType type{ModeRequestType::EnterPassive};

    // StartBehavior 请求的功能名称，例如 rl_locomotion 或 bridge_drive。
    // MotionRuntime 必须拒绝未注册的名称；其他请求中保持为空。
    std::string behavior_name{};

    // SwitchPolicy 请求的目标策略名称；其他请求中保持为空。
    std::string policy_name{};
};

// MotionRuntime 对某个 ModeRequest 的当前处理结果。
struct ModeResult
{
    // 必须与原始 ModeRequest 的 request_id 一致。
    std::uint64_t request_id{0};
    // 请求当前处于接受、执行、完成、拒绝或失败中的哪一阶段。
    ModeResultState state{ModeResultState::Rejected};

    // 面向日志和界面的简短说明，不用于程序分支判断。
    std::string message{};
};

// MotionRuntime 提供给外围状态显示和诊断程序的低频状态。
struct MotionStatus
{
    // 当前实际运行的运动阶段。
    MotionMode mode{MotionMode::Passive};

    // 当前被命令选择器采用的 BaseCommand 来源。
    CommandSource active_source{CommandSource::None};

    // Running 阶段当前执行的功能名称；其他阶段可以为空。
    std::string behavior_name{};

    // 功能内部的低频诊断阶段，例如 prepare 或 driving；不用于控制分支。
    std::string behavior_phase{};

    // 当前已经加载或正在使用的策略名称；未加载策略时为空。
    std::string policy_name{};

    // 最近一次运动控制错误的可读说明；没有错误时为空。
    std::string error_message{};

    // 当前策略是否已经完成加载并可以执行推理。
    bool policy_ready{false};

    // 当前策略允许的 vx、vy、wz 绝对值上限；未加载策略时均为 0。
    std::array<double, 3> command_limits{};
};

}  // 命名空间 quadruped::core
