/**
 * @file mujoco_step_safety_tests.cpp
 * @brief 测试 MujocoRobotIO 单步的限幅、过期回退、会话隔离和故障保护。
 */

#include "test_helpers.hpp"

#include "quadruped/backends/mujoco/mujoco_model.hpp"

#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>

namespace qc = quadruped::core;
namespace qm = quadruped::backends::mujoco;
namespace qtest = quadruped::backends::mujoco::test;

namespace
{

const std::string kBlackScenePath = QUADRUPED_BLACK_SCENE_PATH;

void test_actuator_ctrlrange_clamping()
{
    auto reference = qm::MujocoModel::load(kBlackScenePath, qtest::make_black_model());
    if (!reference.ok())
    {
        qtest::expect(false, "参考模型加载失败");
        return;
    }

    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;

    // feedforward=15 超过 black 执行器 ctrlrange [-10, 10]，但不超过 RobotModel max_effort=40。
    auto frame = qtest::make_command(1, qc::ControlMode::JointImpedance);
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].feedforward_effort = 15.0;
    }
    qtest::expect(io.submit(frame) == qc::RobotIOCode::Ok, "大前馈命令提交");
    qtest::expect(io.step() == qc::RobotIOCode::Ok, "大前馈单步");

    const mjData* data = io.raw_data();
    for (std::size_t i = 0; i < qtest::make_black_model().joint_count; ++i)
    {
        const int actuator_id = reference.model->joint_mappings()[i].actuator_id;
        const double ctrl_min = reference.model->raw_model()->actuator_ctrlrange[2 * actuator_id];
        const double ctrl_max = reference.model->raw_model()->actuator_ctrlrange[2 * actuator_id + 1];
        qtest::expect(data->ctrl[actuator_id] >= ctrl_min, "控制量不低于 actuator ctrlrange 下限");
        qtest::expect(data->ctrl[actuator_id] <= ctrl_max, "控制量不高于 actuator ctrlrange 上限");
    }
}

void test_robot_model_max_effort_clamping()
{
    auto reference = qm::MujocoModel::load(kBlackScenePath, qtest::make_black_model());
    if (!reference.ok())
    {
        qtest::expect(false, "参考模型加载失败");
        return;
    }

    auto model = qtest::make_black_model();
    constexpr double small_effort = 0.5;
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        model.joints[i].limits.max_effort = small_effort;
    }

    auto created = qm::MujocoRobotIO::create(kBlackScenePath, model, 7);
    if (!created.ok() || !created.io->reset(1).ok())
    {
        qtest::expect(false, "创建并 reset 小力限模型失败");
        return;
    }
    qm::MujocoRobotIO& io = *created.io;

    // 用 kp 产生大力矩；target_position 在 [-3, 3] 内，core::validate 通过。
    auto frame = qtest::make_command(1, qc::ControlMode::JointImpedance);
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].kp = 100.0;
        frame.joints[i].target_position = 0.1;
    }
    qtest::expect(io.submit(frame) == qc::RobotIOCode::Ok, "小力限命令提交");
    qtest::expect(io.step() == qc::RobotIOCode::Ok, "小力限单步");

    const mjData* data = io.raw_data();
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        const int actuator_id = reference.model->joint_mappings()[i].actuator_id;
        qtest::expect_close(
            std::abs(data->ctrl[actuator_id]),
            small_effort,
            1e-12,
            "控制量受 RobotModel max_effort 限制");
    }
}

void test_expired_command_falls_back_to_disabled()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;

    // 有效期仅 1 ns；当前仿真时间为 0 时仍有效，步进后 2 ms 即过期。
    auto frame = qtest::make_command(1, qc::ControlMode::JointImpedance, 0, 1);
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].feedforward_effort = 5.0;
    }
    qtest::expect(io.submit(frame) == qc::RobotIOCode::Ok, "短有效期命令提交");
    qtest::expect(io.step() == qc::RobotIOCode::Ok, "有效期内单步");

    qc::StateFrame first;
    qtest::expect(io.read_latest(first) == qc::RobotIOCode::Ok, "读取第一步状态");
    qtest::expect(first.effective_command_sequence == 1, "第一步 effective 使用命令序号");

    qtest::expect(io.step() == qc::RobotIOCode::Ok, "过期后单步");
    const mjData* data = io.raw_data();
    for (int i = 0; i < 12; ++i)
    {
        qtest::expect_close(data->ctrl[i], 0.0, 0.0, "过期后控制量回零");
    }

    qc::StateFrame second;
    qtest::expect(io.read_latest(second) == qc::RobotIOCode::Ok, "读取第二步状态");
    qtest::expect(second.effective_command_sequence == 0, "过期后 effective=0");
    qtest::expect(second.last_accepted_command_sequence == 1, "last_accepted 仍保留最近接受序号");
}

void test_new_session_clears_old_command()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;

    auto frame = qtest::make_command(1, qc::ControlMode::JointImpedance);
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].feedforward_effort = 5.0;
    }
    qtest::expect(io.submit(frame) == qc::RobotIOCode::Ok, "旧会话命令提交");

    qtest::expect(io.reset(2).ok(), "新 session reset");
    qtest::expect(io.step() == qc::RobotIOCode::Ok, "新会话第一步");

    qc::StateFrame state;
    qtest::expect(io.read_latest(state) == qc::RobotIOCode::Ok, "读取新会话状态");
    qtest::expect(state.header.session_id == 2, "新会话状态 session_id 正确");
    qtest::expect(state.effective_command_sequence == 0, "新会话无旧命令 effective=0");
    qtest::expect(state.last_accepted_command_sequence == 0, "新会话无旧命令 last=0");

    const mjData* data = io.raw_data();
    for (int i = 0; i < 12; ++i)
    {
        qtest::expect_close(data->ctrl[i], 0.0, 0.0, "新会话第一步控制量为零");
    }
}

void test_compute_fault_keeps_last_valid_state()
{
    auto reference = qm::MujocoModel::load(kBlackScenePath, qtest::make_black_model());
    if (!reference.ok())
    {
        qtest::expect(false, "参考模型加载失败");
        return;
    }

    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;

    // 先提交一个命令，使 apply_joint_commands 在步进前读取关节状态，从而触发 NaN 检测。
    auto frame = qtest::make_command(1, qc::ControlMode::JointImpedance);
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].feedforward_effort = 1.0;
    }
    qtest::expect(io.submit(frame) == qc::RobotIOCode::Ok, "故障测试命令提交");

    qc::StateFrame before;
    qtest::expect(io.read_latest(before) == qc::RobotIOCode::Ok, "故障前读取有效状态");

    const int qpos_address = reference.model->joint_mappings()[0].qpos_address;
    io.raw_data()->qpos[qpos_address] = std::numeric_limits<double>::quiet_NaN();
    qtest::expect(io.step() == qc::RobotIOCode::Fault, "NaN 注入后 step 返回 Fault");
    qtest::expect(io.status().state == qc::RobotIOState::Fault, "后端进入 Fault");

    qc::StateFrame after;
    qtest::expect(io.read_latest(after) == qc::RobotIOCode::Ok, "Fault 后上一份状态仍可读");
    qtest::expect(after.header.sequence == before.header.sequence, "上一份状态序号未被覆盖");
}

}  // namespace

int main()
{
    test_actuator_ctrlrange_clamping();
    test_robot_model_max_effort_clamping();
    test_expired_command_falls_back_to_disabled();
    test_new_session_clears_old_command();
    test_compute_fault_keeps_last_valid_state();

    if (qtest::failures != 0)
    {
        std::cerr << qtest::failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All quadruped_mujoco step safety tests passed\n";
    return 0;
}
