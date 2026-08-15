/**
 * @file mujoco_step_state_tests.cpp
 * @brief 测试单步后的 StateFrame 字段、序号和校验。
 */

#include "test_helpers.hpp"

#include <cstdint>
#include <iostream>

namespace qc = quadruped::core;
namespace qm = quadruped::backends::mujoco;
namespace qtest = quadruped::backends::mujoco::test;

namespace
{

const std::string kBlackScenePath = QUADRUPED_BLACK_SCENE_PATH;

void test_step_advances_time_and_sequence()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;

    qc::StateFrame before;
    qtest::expect(io.read_latest(before) == qc::RobotIOCode::Ok, "步进前状态可读");
    qtest::expect(io.step() == qc::RobotIOCode::Ok, "第一次单步");

    qc::StateFrame first;
    qtest::expect(io.read_latest(first) == qc::RobotIOCode::Ok, "第一次步进后状态可读");
    qtest::expect(first.header.sequence == before.header.sequence + 1, "sequence 增加一次");
    qtest::expect(first.header.timestamp_ns == before.header.timestamp_ns + 2'000'000, "仿真时间推进 2 ms");

    qtest::expect(io.step() == qc::RobotIOCode::Ok, "第二次单步");
    qc::StateFrame second;
    qtest::expect(io.read_latest(second) == qc::RobotIOCode::Ok, "第二次步进后状态可读");
    qtest::expect(second.header.sequence == first.header.sequence + 1, "sequence 再次增加一次");
}

void test_state_frame_passes_validation_after_step()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;
    auto frame = qtest::make_command(1, qc::ControlMode::Damping);
    qtest::expect(io.submit(frame) == qc::RobotIOCode::Ok, "提交 Damping 命令");
    qtest::expect(io.step() == qc::RobotIOCode::Ok, "单步");

    qc::StateFrame state;
    qtest::expect(io.read_latest(state) == qc::RobotIOCode::Ok, "读取步进后状态");
    const auto validation = qc::validate(state, qtest::make_black_model(), state.header.timestamp_ns);
    qtest::expect(validation.ok(), "步进后 StateFrame 通过 core::validate：" + validation.message);
}

void test_step_sequence_command_numbers()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;

    qtest::expect(io.step() == qc::RobotIOCode::Ok, "无命令步进");
    qc::StateFrame state;
    qtest::expect(io.read_latest(state) == qc::RobotIOCode::Ok, "读取无命令步进状态");
    qtest::expect(state.last_accepted_command_sequence == 0, "无命令时 last_accepted=0");
    qtest::expect(state.effective_command_sequence == 0, "无命令时 effective=0");

    auto frame = qtest::make_command(1, qc::ControlMode::Damping);
    qtest::expect(io.submit(frame) == qc::RobotIOCode::Ok, "提交命令");
    qtest::expect(io.step() == qc::RobotIOCode::Ok, "有命令步进");
    qtest::expect(io.read_latest(state) == qc::RobotIOCode::Ok, "读取有命令步进状态");
    qtest::expect(state.last_accepted_command_sequence == 1, "last_accepted 等于已接受命令序号");
    qtest::expect(state.effective_command_sequence == 1, "effective 等于实际使用命令序号");
    qtest::expect(state.safety_state == qc::SafetyState::ControlEnabled, "正常步进 safety_state=ControlEnabled");
}

}  // namespace

int main()
{
    test_step_advances_time_and_sequence();
    test_state_frame_passes_validation_after_step();
    test_step_sequence_command_numbers();

    if (qtest::failures != 0)
    {
        std::cerr << qtest::failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All quadruped_mujoco step state tests passed\n";
    return 0;
}
