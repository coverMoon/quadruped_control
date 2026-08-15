/**
 * @file mujoco_step_tests.cpp
 * @brief 测试 MujocoRobotIO 的显式单步推进和三种控制模式公式。
 */

#include "test_helpers.hpp"

#include "quadruped/backends/mujoco/mujoco_model.hpp"

#include <cmath>
#include <cstddef>
#include <iostream>

namespace qm = quadruped::backends::mujoco;
namespace qtest = quadruped::backends::mujoco::test;

namespace
{

const std::string kBlackScenePath = QUADRUPED_BLACK_SCENE_PATH;

void test_step_without_command_zeros_ctrl()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;
    qtest::expect(io.step() == qc::RobotIOCode::Ok, "无命令单步成功");
    const mjData* data = io.raw_data();
    for (int i = 0; i < 12; ++i)
    {
        qtest::expect_close(data->ctrl[i], 0.0, 0.0, "无命令时执行器控制量为零");
    }
}

void test_disabled_ignores_other_fields()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;
    auto frame = qtest::make_command(1, qc::ControlMode::Disabled);
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].target_position = 1.0;
        frame.joints[i].target_velocity = 2.0;
        frame.joints[i].kp = 10.0;
        frame.joints[i].kd = 5.0;
        frame.joints[i].feedforward_effort = 3.0;
    }
    qtest::expect(io.submit(frame) == qc::RobotIOCode::Ok, "Disabled 命令提交成功");
    qtest::expect(io.step() == qc::RobotIOCode::Ok, "Disabled 单步成功");
    const mjData* data = io.raw_data();
    for (int i = 0; i < 12; ++i)
    {
        qtest::expect_close(data->ctrl[i], 0.0, 0.0, "Disabled 忽略其他字段输出零");
    }
}

void test_damping_formula()
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

    auto kick = qtest::make_command(1, qc::ControlMode::JointImpedance);
    for (std::size_t i = 0; i < kick.joint_count; ++i)
    {
        kick.joints[i].feedforward_effort = 2.0;
    }
    qtest::expect(io.submit(kick) == qc::RobotIOCode::Ok, "kick 命令提交");
    qtest::expect(io.step() == qc::RobotIOCode::Ok, "kick 单步");

    // Damping 控制使用步进前的速度；先读取速度，再 step，最后验证 ctrl。
    constexpr double kd = 4.0;
    auto damping = qtest::make_command(2, qc::ControlMode::Damping);
    for (std::size_t i = 0; i < damping.joint_count; ++i)
    {
        damping.joints[i].kd = kd;
    }
    qtest::expect(io.submit(damping) == qc::RobotIOCode::Ok, "Damping 命令提交");

    double velocities[12] = {};
    const mjData* before = io.raw_data();
    for (std::size_t i = 0; i < qtest::make_black_model().joint_count; ++i)
    {
        velocities[i] = before->qvel[reference.model->joint_mappings()[i].qvel_address];
    }

    qtest::expect(io.step() == qc::RobotIOCode::Ok, "Damping 单步");
    const mjData* data = io.raw_data();
    for (std::size_t i = 0; i < qtest::make_black_model().joint_count; ++i)
    {
        const int actuator_id = reference.model->joint_mappings()[i].actuator_id;
        qtest::expect_close(
            data->ctrl[actuator_id], -kd * velocities[i], 1e-12, "Damping 力矩符合 -kd*velocity");
    }
}

void test_joint_impedance_formula()
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

    // JointImpedance 控制量按步进前的关节状态计算；先采样位置与速度，再验证 ctrl。
    constexpr double kp = 10.0;
    constexpr double kd = 2.0;
    auto frame = qtest::make_command(1, qc::ControlMode::JointImpedance);
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].target_position = 0.1 * static_cast<double>(i + 1);
        frame.joints[i].target_velocity = 0.05 * static_cast<double>(i + 1);
        frame.joints[i].kp = kp;
        frame.joints[i].kd = kd;
        frame.joints[i].feedforward_effort = 0.5 * static_cast<double>(i + 1);
    }
    qtest::expect(io.submit(frame) == qc::RobotIOCode::Ok, "JointImpedance 命令提交");

    double positions[12] = {};
    double velocities[12] = {};
    const mjData* before = io.raw_data();
    for (std::size_t i = 0; i < qtest::make_black_model().joint_count; ++i)
    {
        positions[i] = before->qpos[reference.model->joint_mappings()[i].qpos_address];
        velocities[i] = before->qvel[reference.model->joint_mappings()[i].qvel_address];
    }

    qtest::expect(io.step() == qc::RobotIOCode::Ok, "JointImpedance 单步");
    const mjData* data = io.raw_data();
    for (std::size_t i = 0; i < qtest::make_black_model().joint_count; ++i)
    {
        const int actuator_id = reference.model->joint_mappings()[i].actuator_id;
        const double expected =
            kp * (frame.joints[i].target_position - positions[i]) +
            kd * (frame.joints[i].target_velocity - velocities[i]) +
            frame.joints[i].feedforward_effort;
        qtest::expect_close(
            data->ctrl[actuator_id], expected, 1e-12, "JointImpedance 力矩符合公式");
    }
}

void test_mixed_modes_per_joint()
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

    // 混合模式：每个关节按各自模式独立计算，仍使用步进前状态。
    constexpr double kd = 3.0;
    constexpr double kp = 5.0;
    auto frame = qtest::make_command(1, qc::ControlMode::Disabled);
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        if (i % 3 == 0)
        {
            frame.joints[i].mode = qc::ControlMode::Disabled;
        }
        else if (i % 3 == 1)
        {
            frame.joints[i].mode = qc::ControlMode::Damping;
            frame.joints[i].kd = kd;
        }
        else
        {
            frame.joints[i].mode = qc::ControlMode::JointImpedance;
            frame.joints[i].kp = kp;
            frame.joints[i].target_position = 0.2;
        }
    }
    qtest::expect(io.submit(frame) == qc::RobotIOCode::Ok, "混合模式命令提交");

    double positions[12] = {};
    double velocities[12] = {};
    const mjData* before = io.raw_data();
    for (std::size_t i = 0; i < qtest::make_black_model().joint_count; ++i)
    {
        positions[i] = before->qpos[reference.model->joint_mappings()[i].qpos_address];
        velocities[i] = before->qvel[reference.model->joint_mappings()[i].qvel_address];
    }

    qtest::expect(io.step() == qc::RobotIOCode::Ok, "混合模式单步");
    const mjData* data = io.raw_data();
    for (std::size_t i = 0; i < qtest::make_black_model().joint_count; ++i)
    {
        const int actuator_id = reference.model->joint_mappings()[i].actuator_id;
        double expected = 0.0;
        if (frame.joints[i].mode == qc::ControlMode::Damping)
        {
            expected = -kd * velocities[i];
        }
        else if (frame.joints[i].mode == qc::ControlMode::JointImpedance)
        {
            expected = kp * (0.2 - positions[i]);
        }
        qtest::expect_close(
            data->ctrl[actuator_id], expected, 1e-12, "混合模式下各关节独立计算");
    }
}

}  // namespace

int main()
{
    test_step_without_command_zeros_ctrl();
    test_disabled_ignores_other_fields();
    test_damping_formula();
    test_joint_impedance_formula();
    test_mixed_modes_per_joint();

    if (qtest::failures != 0)
    {
        std::cerr << qtest::failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All quadruped_mujoco step tests passed\n";
    return 0;
}
