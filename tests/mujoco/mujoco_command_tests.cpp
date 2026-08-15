/**
 * @file mujoco_command_tests.cpp
 * @brief 测试 MujocoRobotIO 对 CommandFrame 的 submit 校验与拒绝规则。
 */

#include "test_helpers.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

namespace qc = quadruped::core;
namespace qm = quadruped::backends::mujoco;
namespace qtest = quadruped::backends::mujoco::test;

namespace
{

const std::string kBlackScenePath = QUADRUPED_BLACK_SCENE_PATH;

void test_submit_before_reset_rejected()
{
    auto created = qm::MujocoRobotIO::create(kBlackScenePath, qtest::make_black_model(), 7);
    qtest::expect(created.ok(), "创建正式模型成功");
    if (!created.ok())
    {
        return;
    }
    const auto frame = qtest::make_command(1, qc::ControlMode::Damping);
    qtest::expect(created.io->submit(frame) == qc::RobotIOCode::Rejected, "reset 前 submit 被拒绝");
    qtest::expect(created.io->status().latest_command_sequence == 0, "reset 前 latest_command_sequence 为 0");
}

void test_submit_accepts_valid_command()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    const auto frame = qtest::make_command(1, qc::ControlMode::Damping);
    qtest::expect(created.io->submit(frame) == qc::RobotIOCode::Ok, "合法命令被接受");
    qtest::expect(created.io->status().latest_command_sequence == 1, "latest_command_sequence 更新");
    qtest::expect(created.io->status().rejected_command_frames == 0, "无拒绝计数");
}

void test_submit_does_not_step_or_publish()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    qc::StateFrame before;
    qtest::expect(created.io->read_latest(before) == qc::RobotIOCode::Ok, "submit 前状态可读");
    const auto frame = qtest::make_command(1, qc::ControlMode::Damping);
    qtest::expect(created.io->submit(frame) == qc::RobotIOCode::Ok, "命令提交成功");
    qc::StateFrame after;
    qtest::expect(created.io->read_latest(after) == qc::RobotIOCode::Ok, "submit 后状态仍可读");
    qtest::expect(after.header.sequence == before.header.sequence, "submit 不增加状态序号");
    qtest::expect(after.header.timestamp_ns == before.header.timestamp_ns, "submit 不推进仿真时间");
}

void test_submit_rejects_header_mismatch()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    auto bad_schema = qtest::make_command(1, qc::ControlMode::Damping);
    bad_schema.header.schema_version = 999;
    qtest::expect(created.io->submit(bad_schema) == qc::RobotIOCode::InvalidFrame, "schema 错误返回 InvalidFrame");

    auto bad_model = qtest::make_command(2, qc::ControlMode::Damping);
    bad_model.header.model_id = 0x1234;
    qtest::expect(created.io->submit(bad_model) == qc::RobotIOCode::InvalidFrame, "model_id 错误返回 InvalidFrame");

    auto bad_calibration = qtest::make_command(3, qc::ControlMode::Damping);
    bad_calibration.header.calibration_id = 0xABCD;
    qtest::expect(created.io->submit(bad_calibration) == qc::RobotIOCode::InvalidFrame, "calibration_id 错误返回 InvalidFrame");

    auto bad_count = qtest::make_command(4, qc::ControlMode::Damping);
    bad_count.joint_count = 11;
    qtest::expect(created.io->submit(bad_count) == qc::RobotIOCode::InvalidFrame, "joint_count 错误返回 InvalidFrame");
}

void test_submit_rejects_session_and_startup_mismatch()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    auto bad_session = qtest::make_command(1, qc::ControlMode::Damping);
    bad_session.header.session_id = 2;
    qtest::expect(created.io->submit(bad_session) == qc::RobotIOCode::Rejected, "session_id 不匹配被拒绝");

    auto bad_startup = qtest::make_command(1, qc::ControlMode::Damping);
    bad_startup.header.startup_id = 8;
    qtest::expect(created.io->submit(bad_startup) == qc::RobotIOCode::Rejected, "startup_id 不匹配被拒绝");
}

void test_submit_rejects_bad_sequence()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    auto zero_seq = qtest::make_command(0, qc::ControlMode::Damping);
    qtest::expect(created.io->submit(zero_seq) == qc::RobotIOCode::Rejected, "sequence=0 被拒绝");

    auto first = qtest::make_command(1, qc::ControlMode::Damping);
    qtest::expect(created.io->submit(first) == qc::RobotIOCode::Ok, "首个命令接受");

    auto duplicate = qtest::make_command(1, qc::ControlMode::Damping);
    qtest::expect(created.io->submit(duplicate) == qc::RobotIOCode::Rejected, "重复 sequence 被拒绝");

    auto backward = qtest::make_command(0, qc::ControlMode::Damping);
    qtest::expect(created.io->submit(backward) == qc::RobotIOCode::Rejected, "倒退 sequence 被拒绝");
}

void test_submit_rejects_timing_errors()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    auto future = qtest::make_command(1, qc::ControlMode::Damping, 1'000'000'000, 2'000'000'000);
    qtest::expect(created.io->submit(future) == qc::RobotIOCode::InvalidFrame, "未来时间戳被拒绝");

    auto bad_expiry = qtest::make_command(2, qc::ControlMode::Damping, 0, 0);
    qtest::expect(created.io->submit(bad_expiry) == qc::RobotIOCode::InvalidFrame, "非法有效期被拒绝");

    auto expired = qtest::make_command(3, qc::ControlMode::Damping, 0, 0);
    expired.expires_at_ns = -1;
    qtest::expect(created.io->submit(expired) == qc::RobotIOCode::InvalidFrame, "过期命令被拒绝");
}

void test_submit_rejects_nan_and_invalid_mode()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    auto nan_command = qtest::make_command(1, qc::ControlMode::Damping);
    nan_command.joints[0].kp = std::numeric_limits<double>::quiet_NaN();
    qtest::expect(created.io->submit(nan_command) == qc::RobotIOCode::InvalidFrame, "NaN 被拒绝");

    auto velocity_command = qtest::make_command(2, qc::ControlMode::Velocity);
    qtest::expect(created.io->submit(velocity_command) == qc::RobotIOCode::Rejected, "Velocity 模式被拒绝");

    auto torque_command = qtest::make_command(3, qc::ControlMode::Torque);
    qtest::expect(created.io->submit(torque_command) == qc::RobotIOCode::Rejected, "Torque 模式被拒绝");
}

void test_rejected_command_does_not_replace_valid()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    const auto valid = qtest::make_command(1, qc::ControlMode::Damping);
    qtest::expect(created.io->submit(valid) == qc::RobotIOCode::Ok, "接受合法命令");

    const auto rejected = qtest::make_command(1, qc::ControlMode::Damping);
    qtest::expect(created.io->submit(rejected) == qc::RobotIOCode::Rejected, "重复命令被拒绝");
    qtest::expect(created.io->status().latest_command_sequence == 1, "拒绝后最新序号不变");
    qtest::expect(created.io->status().rejected_command_frames == 1, "拒绝计数加一");
}

void test_rejected_counter_accurate()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    auto bad = qtest::make_command(0, qc::ControlMode::Damping);
    for (int i = 0; i < 3; ++i)
    {
        created.io->submit(bad);
    }
    qtest::expect(created.io->status().rejected_command_frames == 3, "连续拒绝计数正确");
}

}  // namespace

int main()
{
    test_submit_before_reset_rejected();
    test_submit_accepts_valid_command();
    test_submit_does_not_step_or_publish();
    test_submit_rejects_header_mismatch();
    test_submit_rejects_session_and_startup_mismatch();
    test_submit_rejects_bad_sequence();
    test_submit_rejects_timing_errors();
    test_submit_rejects_nan_and_invalid_mode();
    test_rejected_command_does_not_replace_valid();
    test_rejected_counter_accurate();

    if (qtest::failures != 0)
    {
        std::cerr << qtest::failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All quadruped_mujoco command tests passed\n";
    return 0;
}
