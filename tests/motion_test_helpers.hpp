/**
 * @file motion_test_helpers.hpp
 * @brief MotionRuntime 测试共用的假 RobotIO、模型/配置构造和断言辅助函数。
 */

#pragma once

#include "quadruped/core/core.hpp"
#include "quadruped/motion/motion_runtime.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace qc = quadruped::core;
namespace qm = quadruped::motion;

namespace motion_test
{

enum class RobotIOFaultInjection : std::uint8_t
{
    None = 0,
    NoData,
    Disconnected,
    BackendFault,
    FutureTimestamp,
    InvalidImu,
    NonFiniteJoint,
    InfiniteJoint,
    ExpiredCommand,
};

// 简单测试程序累计失败断言，全部用例运行后统一返回非零退出码。
inline int failures = 0;

inline void expect(bool condition, const std::string& description)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << description << '\n';
        ++failures;
    }
}

// 一次近似比较的全部输入，集中命名避免断言辅助函数参数过多。
struct CloseCheck
{
    double actual;
    double expected;
    double tolerance;
    std::string description;
};

inline void expect_close(const CloseCheck& check)
{
    if (!std::isfinite(check.actual) || std::abs(check.actual - check.expected) >
        check.tolerance)
    {
        std::cerr << "FAIL: " << check.description << "（期望 " << check.expected
                  << "，实际 " << check.actual << "）\n";
        ++failures;
    }
}

// 可编程的 RobotIO：read_latest/submit 的返回码和状态由用例控制，提交的命令被记录。
class FakeRobotIO final : public qc::RobotIO
{
public:
    qc::RobotIOCode read_code{qc::RobotIOCode::Ok};
    qc::RobotIOCode submit_code{qc::RobotIOCode::Ok};
    qc::StateFrame state{};
    std::vector<qc::CommandFrame> submitted{};
    RobotIOFaultInjection injection{RobotIOFaultInjection::None};
    qc::RobotIOStatus status_snapshot{qc::RobotIOState::Ready};
    qc::Nanoseconds clock_ns{0};

    void inject(const RobotIOFaultInjection fault)
    {
        injection = fault;
        status_snapshot.state = fault == RobotIOFaultInjection::Disconnected
            ? qc::RobotIOState::Disconnected
            : (fault == RobotIOFaultInjection::BackendFault
                      ? qc::RobotIOState::Fault
                      : qc::RobotIOState::Ready);
    }

    void clear_injection()
    {
        injection = RobotIOFaultInjection::None;
        status_snapshot.state = qc::RobotIOState::Ready;
    }

    qc::RobotIOCode read_latest(qc::StateFrame& frame) override
    {
        if (injection == RobotIOFaultInjection::NoData)
        {
            status_snapshot.state = qc::RobotIOState::Paused;
            return qc::RobotIOCode::NoData;
        }
        if (injection == RobotIOFaultInjection::Disconnected)
        {
            return qc::RobotIOCode::Disconnected;
        }
        if (injection == RobotIOFaultInjection::BackendFault)
        {
            return qc::RobotIOCode::Fault;
        }
        if (read_code != qc::RobotIOCode::Ok)
        {
            return read_code;
        }
        frame = state;
        if (injection == RobotIOFaultInjection::FutureTimestamp)
        {
            frame.header.timestamp_ns += 1;
        }
        else if (injection == RobotIOFaultInjection::InvalidImu)
        {
            frame.imu.valid = false;
        }
        else if (injection == RobotIOFaultInjection::NonFiniteJoint)
        {
            frame.joints[0].position = std::numeric_limits<double>::quiet_NaN();
        }
        else if (injection == RobotIOFaultInjection::InfiniteJoint)
        {
            frame.joints[0].velocity = std::numeric_limits<double>::infinity();
        }
        status_snapshot.latest_state_sequence = frame.header.sequence;
        return qc::RobotIOCode::Ok;
    }

    qc::RobotIOCode submit(const qc::CommandFrame& frame) override
    {
        if (injection == RobotIOFaultInjection::Disconnected)
        {
            ++status_snapshot.rejected_command_frames;
            return qc::RobotIOCode::Disconnected;
        }
        if (injection == RobotIOFaultInjection::BackendFault)
        {
            ++status_snapshot.rejected_command_frames;
            return qc::RobotIOCode::Fault;
        }
        if (injection == RobotIOFaultInjection::ExpiredCommand)
        {
            ++status_snapshot.rejected_command_frames;
            return qc::RobotIOCode::InvalidFrame;
        }
        if (submit_code == qc::RobotIOCode::Ok)
        {
            submitted.push_back(frame);
            status_snapshot.latest_command_sequence = frame.header.sequence;
        }
        return submit_code;
    }

    qc::RobotIOStatus status() const noexcept override
    {
        return status_snapshot;
    }

    qc::Nanoseconds clock_now_ns() const noexcept override
    {
        return clock_ns;
    }
};

inline qc::RobotModel make_test_model()
{
    qc::RobotModel model;
    model.name = "test_quadruped";
    model.joint_count = 12;
    constexpr const char* names[12] = {
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    };
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        model.joints[i].name = names[i];
        model.joints[i].role = qc::JointRole::Leg;
        model.joints[i].limits = {true, -3.0, 3.0, 20.0, 40.0, 100.0, 10.0};
    }
    return model;
}

inline qc::RobotModel make_wheel_test_model()
{
    qc::RobotModel model;
    model.name = "test_wheel_quadruped";
    model.joint_count = 16;
    constexpr const char* names[16] = {
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FL_wheel_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FR_wheel_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RL_wheel_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RR_wheel_joint",
    };
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        const bool is_wheel = i % 4 == 3;
        model.joints[i].name = names[i];
        model.joints[i].role = is_wheel ? qc::JointRole::Wheel : qc::JointRole::Leg;
        model.joints[i].limits = is_wheel
            ? qc::JointLimits{false, 0.0, 0.0, 50.0, 20.0, 0.0, 10.0}
            : qc::JointLimits{true, -3.0, 3.0, 20.0, 40.0, 100.0, 10.0};
    }
    return model;
}

// 测试用小幅周期数的控制器配置，姿态和增益与 black 真实配置一致。
inline qc::ControllerConfig make_test_config()
{
    qc::ControllerConfig config;
    config.control_period_ns = 5'000'000;
    config.command_validity_ns = 10'000'000;
    config.getup_pre_cycles = 2;
    config.getup_cycles = 2;
    config.getdown_cycles = 4;
    constexpr double pre[12] = {
        0.0, 1.4, -2.2, 0.0, -1.4, 2.2, 0.0, 1.4, -2.2, 0.0, -1.4, 2.2};
    constexpr double stand[12] = {
        0.0, 0.82, -1.5, 0.0, -0.82, 1.5, 0.0, 0.82, -1.5, 0.0, -0.82, 1.5};
    for (std::size_t i = 0; i < 12; ++i)
    {
        config.pre_getup_position[i] = pre[i];
        config.stand_position[i] = stand[i];
        config.fixed_kp[i] = 80.0;
        config.fixed_kd[i] = 3.0;
    }
    return config;
}

inline qc::ControllerConfig make_wheel_test_config()
{
    qc::ControllerConfig config;
    config.control_period_ns = 5'000'000;
    config.command_validity_ns = 10'000'000;
    config.getup_pre_cycles = 2;
    config.getup_cycles = 1;
    config.getdown_cycles = 2;
    for (std::size_t i = 0; i < 16; ++i)
    {
        const bool is_wheel = i % 4 == 3;
        config.pre_getup_position[i] = is_wheel ? 0.0 : 0.2;
        config.stand_position[i] = is_wheel ? 0.0 : 0.4;
        config.fixed_kp[i] = is_wheel ? 0.0 : 80.0;
        config.fixed_kd[i] = is_wheel ? 0.5 : 3.0;
    }
    return config;
}

inline qm::RetryConfig make_retry_config(const qc::RobotModel& model)
{
    qm::RetryConfig config;
    config.robot_name = model.name;
    config.joint_count = model.joint_count;
    config.prepare_cycles = 2;
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        const bool is_wheel = model.joints[i].role == qc::JointRole::Wheel;
        config.joint_names[i] = model.joints[i].name;
        config.target_positions[i] = is_wheel ? 0.0 : 0.3;
        config.kp[i] = is_wheel ? 0.0 : 70.0;
        config.kd[i] = is_wheel ? 0.7 : 2.5;
    }
    return config;
}

// 每个关节取不同位置，便于发现数组错位；数值远离预起立姿态和站姿。
inline std::array<double, qc::kMaxJoints> make_rest_positions()
{
    std::array<double, qc::kMaxJoints> positions{};
    for (std::size_t i = 0; i < 12; ++i)
    {
        positions[i] = 0.05 * static_cast<double>(i);
    }
    return positions;
}

// 构造一份合法的 StateFrame；会话号和安全状态等特殊场景由用例在返回后覆盖字段。
inline qc::StateFrame make_state(
    const qc::RobotModel& model,
    const std::array<double, qc::kMaxJoints>& positions)
{
    qc::StateFrame state;
    state.header.startup_id = 1;
    state.header.session_id = 1;
    state.header.sequence = 1;
    state.header.timestamp_ns = 0;
    state.joint_count = model.joint_count;
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        state.joints[i].position = positions[i];
        state.joints[i].online = true;
        state.joints[i].valid = true;
    }
    state.imu.valid = true;
    state.safety_state = qc::SafetyState::ControlEnabled;
    return state;
}

inline qc::ModeRequest make_request(std::uint64_t id, qc::ModeRequestType type)
{
    qc::ModeRequest request;
    request.request_id = id;
    request.timestamp_ns = 0;
    request.type = type;
    return request;
}

// 以 now_ns=0 执行一个控制周期；特殊时间用例直接构造 MotionUpdateInput。
inline qm::MotionUpdateOutput update(
    qm::MotionRuntime& runtime,
    FakeRobotIO& io,
    const qc::ModeRequest* request = nullptr)
{
    qm::MotionUpdateInput input;
    input.now_ns = 0;
    input.request = request;
    return runtime.update(io, input);
}

// 创建通过校验的 MotionRuntime；失败时记录断言并返回空指针。
inline qm::MotionRuntime::CreateResult make_runtime()
{
    auto created = qm::MotionRuntime::create(make_test_model(), make_test_config());
    expect(created.ok(), "合法模型和配置应创建成功：" + created.error_message);
    return created;
}

// 从 Passive 驱动一次完整起立直到 Stand；假定中间不出现失败。
inline void drive_getup(qm::MotionRuntime& runtime, FakeRobotIO& io, std::uint64_t request_id)
{
    const auto request = make_request(request_id, qc::ModeRequestType::GetUp);
    update(runtime, io, &request);
    // 预起立 2 周期 + 起立 2 周期 + 1 个完成周期。
    for (int i = 0; i < 5; ++i)
    {
        update(runtime, io);
    }
    expect(io.submitted.back().motion_mode == qc::MotionMode::Stand, "起立驱动后应进入 Stand");
}

}  // namespace motion_test
