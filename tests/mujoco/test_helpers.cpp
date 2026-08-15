/**
 * @file test_helpers.cpp
 * @brief 实现 MuJoCo 测试共用辅助函数。
 */

#include "test_helpers.hpp"

#include <cmath>

namespace quadruped::backends::mujoco::test
{

int failures = 0;

void expect(const bool condition, const std::string& description)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << description << '\n';
        ++failures;
    }
}

void expect_close(
    const double actual,
    const double expected,
    const double tolerance,
    const std::string& description)
{
    if (std::abs(actual - expected) > tolerance)
    {
        std::cerr << "FAIL: " << description << "（期望 " << expected
                  << "，实际 " << actual << "）\n";
        ++failures;
    }
}

qc::RobotModel make_black_model()
{
    qc::RobotModel model;
    model.name = "black";
    model.model_id = 0x008A1E56CD69E8F4;
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

MujocoRobotIO::CreateResult create_ready(
    const std::string& scene_path,
    const std::uint64_t session_id)
{
    auto created = MujocoRobotIO::create(scene_path, make_black_model(), 7);
    if (!created.ok())
    {
        expect(false, "创建正式模型失败：" + created.error_message);
        return created;
    }
    if (!created.io->reset(session_id).ok())
    {
        expect(false, "reset 正式模型失败");
    }
    return created;
}

qc::CommandFrame make_command(
    const std::uint64_t sequence,
    const qc::ControlMode mode,
    const std::uint64_t timestamp_ns,
    const std::uint64_t expires_at_ns)
{
    qc::CommandFrame frame;
    frame.header.schema_version = qc::kFrameSchemaVersion;
    frame.header.startup_id = 7;
    frame.header.session_id = 1;
    frame.header.sequence = sequence;
    frame.header.timestamp_ns = static_cast<qc::Nanoseconds>(timestamp_ns);
    frame.header.model_id = 0x008A1E56CD69E8F4;
    frame.header.calibration_id = 0;
    frame.expires_at_ns = static_cast<qc::Nanoseconds>(expires_at_ns);
    frame.joint_count = 12;
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].mode = mode;
        frame.joints[i].target_position = 0.0;
        frame.joints[i].target_velocity = 0.0;
        frame.joints[i].kp = 0.0;
        frame.joints[i].kd = (mode == qc::ControlMode::Damping) ? 1.0 : 0.0;
        frame.joints[i].feedforward_effort = 0.0;
    }
    frame.motion_mode = qc::MotionMode::Passive;
    frame.source = qc::CommandSource::Test;
    return frame;
}

}  // namespace quadruped::backends::mujoco::test
