/**
 * @file core_tests.cpp
 * @brief 测试核心数据结构、RobotModel 校验和 RobotIO 接口契约。
 */

#include "quadruped/core/core.hpp"

#include <cstddef>
#include <iostream>
#include <limits>
#include <string>

namespace qc = quadruped::core;

namespace
{

// 简单测试程序累计失败断言，全部用例运行后统一返回非零退出码。
int failures = 0;

void expect(bool condition, const std::string& description)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << description << '\n';
        ++failures;
    }
}

qc::RobotModel make_model()
{
    // 所有测试共享一个合法的 12 关节模型，
    // 再由单个用例只修改待验证字段。
    qc::RobotModel model;
    model.name = "test_quadruped";
    model.joint_count = 12;

    constexpr const char* names[12] = {
        "FL_hip",
        "FL_thigh",
        "FL_calf",
        "FR_hip",
        "FR_thigh",
        "FR_calf",
        "RL_hip",
        "RL_thigh",
        "RL_calf",
        "RR_hip",
        "RR_thigh",
        "RR_calf",
    };
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        model.joints[i].name = names[i];
        model.joints[i].role = qc::JointRole::Leg;

        // 测试模型默认使用有限转角的腿部关节。
        auto& limits = model.joints[i].limits;
        limits.position_limited = true;
        limits.min_position = -3.0;
        limits.max_position = 3.0;
        limits.max_velocity = 20.0;
        limits.max_effort = 40.0;
        limits.max_kp = 100.0;
        limits.max_kd = 10.0;
    }
    return model;
}

qc::FrameHeader make_header(qc::Nanoseconds timestamp_ns)
{
    qc::FrameHeader header;
    header.startup_id = 1;
    header.session_id = 2;
    header.sequence = 3;
    header.timestamp_ns = timestamp_ns;
    return header;
}

qc::StateFrame make_state(const qc::RobotModel& model, qc::Nanoseconds timestamp_ns)
{
    qc::StateFrame frame;
    frame.header = make_header(timestamp_ns);
    frame.joint_count = model.joint_count;
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].online = true;
        frame.joints[i].valid = true;
    }
    frame.imu.valid = true;
    return frame;
}

qc::CommandFrame make_command(const qc::RobotModel& model, qc::Nanoseconds timestamp_ns)
{
    qc::CommandFrame frame;
    frame.header = make_header(timestamp_ns);
    // 测试命令默认在生成后的 5 ms 内有效。
    frame.expires_at_ns = timestamp_ns + 5'000'000;
    frame.joint_count = model.joint_count;
    frame.motion_mode = qc::MotionMode::Stand;
    frame.source = qc::CommandSource::Test;
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].mode = qc::ControlMode::JointImpedance;
        frame.joints[i].kp = 80.0;
        frame.joints[i].kd = 3.0;
    }
    return frame;
}

void test_robot_model()
{
    auto model = make_model();
    expect(qc::validate(model).ok(), "valid RobotModel is accepted");

    model.joints[4].name = model.joints[0].name;
    const auto duplicate_result = qc::validate(model);
    expect(
        duplicate_result.error == qc::ValidationError::DuplicateJointName,
        "duplicate joint name is rejected");
    expect(duplicate_result.joint_index == 4, "duplicate joint reports its index");

    model = make_model();
    model.joint_count = qc::kMaxJoints + 1;
    expect(
        qc::validate(model).error == qc::ValidationError::InvalidJointCount,
        "oversized RobotModel is rejected");

    model = make_model();
    model.joints[2].limits.min_position = model.joints[2].limits.max_position;
    expect(
        qc::validate(model).error == qc::ValidationError::InvalidJointLimits,
        "invalid position limits are rejected");

    model = make_model();
    model.joints[3].role = static_cast<qc::JointRole>(255);
    expect(
        qc::validate(model).error == qc::ValidationError::InvalidJointRole,
        "unknown joint role is rejected");

    model = make_model();
    model.joints[3].role = qc::JointRole::Wheel;
    model.joints[3].limits.position_limited = false;
    model.joints[3].limits.min_position = 0.0;
    model.joints[3].limits.max_position = 0.0;
    expect(
        qc::validate(model).ok(),
        "wheel without position limits is accepted independently of source model type");
}

void test_state_frame()
{
    const auto model = make_model();
    auto frame = make_state(model, 1'000);
    expect(qc::validate(frame, model, 1'000).ok(), "valid StateFrame is accepted");

    frame.joint_count = 11;
    expect(
        qc::validate(frame, model, 1'000).error == qc::ValidationError::InvalidJointCount,
        "StateFrame with wrong dimension is rejected");

    frame = make_state(model, 2'000);
    expect(
        qc::validate(frame, model, 1'000).error == qc::ValidationError::InvalidTimestamp,
        "StateFrame from the future is rejected");

    frame = make_state(model, 1'000);
    frame.joints[5].velocity = std::numeric_limits<double>::quiet_NaN();
    expect(
        qc::validate(frame, model, 1'000).error == qc::ValidationError::NonFiniteValue,
        "non-finite joint state is rejected");
}

void test_command_frame()
{
    const auto model = make_model();
    auto frame = make_command(model, 1'000);
    expect(qc::validate(frame, model, 2'000).ok(), "valid CommandFrame is accepted");

    frame.expires_at_ns = 1'500;
    expect(
        qc::validate(frame, model, 2'000).error == qc::ValidationError::Expired,
        "expired CommandFrame is rejected");

    frame = make_command(model, 1'000);
    frame.joints[3].mode = static_cast<qc::ControlMode>(255);
    expect(
        qc::validate(frame, model, 2'000).error == qc::ValidationError::InvalidControlMode,
        "unknown joint control mode is rejected");

    frame = make_command(model, 1'000);
    frame.joints[7].kp = -1.0;
    expect(
        qc::validate(frame, model, 2'000).error == qc::ValidationError::NegativeValue,
        "negative command gain is rejected");

    frame = make_command(model, 1'000);
    frame.joints[1].target_position = 4.0;
    expect(
        qc::validate(frame, model, 2'000).error == qc::ValidationError::InvalidJointLimits,
        "out-of-range active position command is rejected");

    // 轮子仍使用关节阻抗模式，KP=0 时只使用目标速度和 KD 产生力矩。
    auto wheel_model = make_model();
    wheel_model.joints[3].role = qc::JointRole::Wheel;
    wheel_model.joints[3].limits.position_limited = false;
    frame = make_command(wheel_model, 1'000);
    frame.joints[3].target_position = 1'000'000.0;
    frame.joints[3].target_velocity = 5.0;
    frame.joints[3].kp = 0.0;
    frame.joints[3].kd = 2.0;
    expect(
        qc::validate(frame, wheel_model, 2'000).ok(),
        "wheel JointImpedance command accepts velocity and damping without position limits");

    // 即使关节有位置限制，KP=0 时目标位置也不参与输出。
    frame = make_command(model, 1'000);
    frame.joints[1].target_position = 4.0;
    frame.joints[1].kp = 0.0;
    expect(
        qc::validate(frame, model, 2'000).ok(),
        "inactive position term does not trigger position-limit rejection");
}

void test_external_commands()
{
    qc::BaseCommand base;
    base.sequence = 1;
    base.timestamp_ns = 1'000;
    base.expires_at_ns = 2'000;
    base.source = qc::CommandSource::Navigation;
    base.vx = 0.5;
    expect(qc::validate(base, 1'500).ok(), "valid BaseCommand is accepted");
    expect(
        qc::validate(base, 2'001).error == qc::ValidationError::Expired,
        "expired BaseCommand is rejected");

    base.expires_at_ns = 3'000;
    base.vx = std::numeric_limits<double>::infinity();
    expect(
        qc::validate(base, 1'500).error == qc::ValidationError::NonFiniteValue,
        "non-finite BaseCommand is rejected");

    qc::ModeRequest request;
    request.request_id = 7;
    request.timestamp_ns = 1'000;
    request.type = qc::ModeRequestType::GetUp;
    expect(qc::validate(request, 1'500).ok(), "valid ModeRequest is accepted");

    request.type = qc::ModeRequestType::StartBehavior;
    expect(
        qc::validate(request, 1'500).error == qc::ValidationError::InvalidName,
        "behavior start without behavior name is rejected");

    request.behavior_name = "bridge_drive";
    expect(
        qc::validate(request, 1'500).ok(),
        "behavior start with an explicit behavior name is accepted");

    request.behavior_name.clear();
    request.type = qc::ModeRequestType::SwitchPolicy;
    expect(
        qc::validate(request, 1'500).error == qc::ValidationError::InvalidName,
        "policy switch without policy name is rejected");
}

class CompileTimeRobotIO final : public qc::RobotIO
{
public:
    qc::RobotIOCode read_latest(qc::StateFrame&) override
    {
        return qc::RobotIOCode::NoData;
    }

    qc::RobotIOCode submit(const qc::CommandFrame&) override
    {
        return qc::RobotIOCode::Ok;
    }

    qc::RobotIOStatus status() const noexcept override
    {
        return {};
    }
};

void test_robot_io_contract()
{
    CompileTimeRobotIO io;
    qc::StateFrame frame;
    expect(io.read_latest(frame) == qc::RobotIOCode::NoData, "RobotIO implementation compiles");
}

}  // 匿名命名空间

int main()
{
    test_robot_model();
    test_state_frame();
    test_command_frame();
    test_external_commands();
    test_robot_io_contract();

    if (failures != 0)
    {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All quadruped_core tests passed\n";
    return 0;
}
