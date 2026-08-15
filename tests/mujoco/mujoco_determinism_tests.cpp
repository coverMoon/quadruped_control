/**
 * @file mujoco_determinism_tests.cpp
 * @brief 验证 MuJoCo 开环仿真的可重复性与端到端行为。
 */

#include "test_helpers.hpp"

#include "quadruped/backends/mujoco/mujoco_model.hpp"

#include <cmath>
#include <cstddef>
#include <iostream>

namespace qc = quadruped::core;
namespace qm = quadruped::backends::mujoco;
namespace qtest = quadruped::backends::mujoco::test;

namespace
{

const std::string kBlackScenePath = QUADRUPED_BLACK_SCENE_PATH;

// 同一机器、同一构建的 MuJoCo 单线程推进是逐值一致的；
// 使用极小容差捕获意外的非确定性来源，而不放大平台差异。
constexpr double kDeterminismTolerance = 1.0e-12;

// 驱动机器人偏离默认姿态的阻抗命令，用于确定性对比。
qc::CommandFrame make_impedance_command(const std::uint64_t sequence)
{
    auto frame = qtest::make_command(sequence, qc::ControlMode::JointImpedance);
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].kp = 20.0;
        frame.joints[i].target_position = 0.0;
    }
    return frame;
}

// 推进固定步数；返回 false 表示某一步失败。
bool run_steps(qm::MujocoRobotIO& io, std::size_t count)
{
    for (std::size_t i = 0; i < count; ++i)
    {
        if (io.step() != qc::RobotIOCode::Ok)
        {
            return false;
        }
    }
    return true;
}

// 比较两份 StateFrame 的关键字段；要求同一构建内逐值一致。
void expect_state_close(
    const qc::StateFrame& actual,
    const qc::StateFrame& expected,
    const std::string& description)
{
    qtest::expect_close(
        static_cast<double>(actual.header.sequence),
        static_cast<double>(expected.header.sequence),
        0.0,
        description + "：sequence 一致");
    qtest::expect_close(
        static_cast<double>(actual.header.timestamp_ns),
        static_cast<double>(expected.header.timestamp_ns),
        0.0,
        description + "：timestamp_ns 一致");

    for (std::size_t i = 0; i < actual.joint_count; ++i)
    {
        qtest::expect_close(
            actual.joints[i].position, expected.joints[i].position, kDeterminismTolerance,
            description + "：关节 " + std::to_string(i) + " 位置一致");
        qtest::expect_close(
            actual.joints[i].velocity, expected.joints[i].velocity, kDeterminismTolerance,
            description + "：关节 " + std::to_string(i) + " 速度一致");
        qtest::expect_close(
            actual.joints[i].effort, expected.joints[i].effort, kDeterminismTolerance,
            description + "：关节 " + std::to_string(i) + " 力矩一致");
    }

    for (std::size_t i = 0; i < 4; ++i)
    {
        qtest::expect_close(
            actual.imu.orientation[i], expected.imu.orientation[i], kDeterminismTolerance,
            description + "：IMU 四元数分量一致");
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        qtest::expect_close(
            actual.imu.angular_velocity[i], expected.imu.angular_velocity[i], kDeterminismTolerance,
            description + "：IMU 角速度一致");
        qtest::expect_close(
            actual.imu.linear_acceleration[i], expected.imu.linear_acceleration[i],
            kDeterminismTolerance,
            description + "：IMU 线加速度一致");
    }
}

void test_deterministic_run()
{
    auto first = qtest::create_ready(kBlackScenePath, 1);
    auto second = qtest::create_ready(kBlackScenePath, 1);
    if (!first.ok() || !second.ok())
    {
        return;
    }

    auto frame = make_impedance_command(1);
    qtest::expect(first.io->submit(frame) == qc::RobotIOCode::Ok, "第一次运行提交命令");
    qtest::expect(second.io->submit(frame) == qc::RobotIOCode::Ok, "第二次运行提交命令");

    constexpr std::size_t kSteps = 50;
    qtest::expect(run_steps(*first.io, kSteps), "第一次运行推进");
    qtest::expect(run_steps(*second.io, kSteps), "第二次运行推进");

    qc::StateFrame first_state;
    qc::StateFrame second_state;
    qtest::expect(first.io->read_latest(first_state) == qc::RobotIOCode::Ok, "第一次运行读取状态");
    qtest::expect(second.io->read_latest(second_state) == qc::RobotIOCode::Ok, "第二次运行读取状态");

    expect_state_close(second_state, first_state, "两次独立运行");
}

void test_damping_does_not_accelerate()
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

    // 给所有关节一个正向初始速度，再施加 Damping；主动阻尼应阻止运动，而不是同向加速。
    constexpr double kInitialVelocity = 0.5;
    constexpr double kDampingKd = 2.0;
    for (std::size_t i = 0; i < qtest::make_black_model().joint_count; ++i)
    {
        const int qvel_address = reference.model->joint_mappings()[i].qvel_address;
        io.raw_data()->qvel[qvel_address] = kInitialVelocity;
    }

    auto frame = qtest::make_command(1, qc::ControlMode::Damping);
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].kd = kDampingKd;
    }
    qtest::expect(io.submit(frame) == qc::RobotIOCode::Ok, "Damping 命令提交");
    qtest::expect(io.step() == qc::RobotIOCode::Ok, "Damping 单步");

    for (std::size_t i = 0; i < qtest::make_black_model().joint_count; ++i)
    {
        const int qvel_address = reference.model->joint_mappings()[i].qvel_address;
        const double after = io.raw_data()->qvel[qvel_address];
        qtest::expect(
            after <= kInitialVelocity + kDeterminismTolerance,
            "Damping 不会使正向速度加速");
        const int actuator_id = reference.model->joint_mappings()[i].actuator_id;
        qtest::expect_close(
            io.raw_data()->ctrl[actuator_id],
            -kDampingKd * kInitialVelocity,
            kDeterminismTolerance,
            "Damping 力矩方向与速度相反");
    }
}

void run_impedance_direction_check(
    const qm::MujocoModel& reference,
    const double delta,
    const std::string& direction)
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;

    constexpr double kKp = 50.0;
    auto frame = qtest::make_command(1, qc::ControlMode::JointImpedance);
    double before[12] = {};
    for (std::size_t i = 0; i < qtest::make_black_model().joint_count; ++i)
    {
        const int qpos_address = reference.joint_mappings()[i].qpos_address;
        before[i] = io.raw_data()->qpos[qpos_address];
        frame.joints[i].kp = kKp;
        frame.joints[i].target_position = before[i] + delta;
    }

    qtest::expect(io.submit(frame) == qc::RobotIOCode::Ok, direction + "目标命令提交");
    qtest::expect(io.step() == qc::RobotIOCode::Ok, direction + "目标单步");

    for (std::size_t i = 0; i < qtest::make_black_model().joint_count; ++i)
    {
        const int qpos_address = reference.joint_mappings()[i].qpos_address;
        const double after = io.raw_data()->qpos[qpos_address];
        const double change = after - before[i];
        qtest::expect(
            change * delta >= -1e-9,
            direction + "阻抗方向与目标偏移一致");
    }
}

void test_joint_impedance_direction()
{
    auto reference = qm::MujocoModel::load(kBlackScenePath, qtest::make_black_model());
    if (!reference.ok())
    {
        qtest::expect(false, "参考模型加载失败");
        return;
    }

    // 正负目标分别从初始姿态独立运行，避免惯性干扰单步方向判断。
    run_impedance_direction_check(*reference.model, 0.1, "正向");
    run_impedance_direction_check(*reference.model, -0.1, "负向");
}

void test_long_run_stays_finite()
{
    auto created = qtest::create_ready(kBlackScenePath, 1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;

    constexpr std::size_t kSteps = 1000;
    qtest::expect(run_steps(io, kSteps), "长时间无命令运行保持可推进");

    qc::StateFrame state;
    qtest::expect(io.read_latest(state) == qc::RobotIOCode::Ok, "长时间运行后读取状态");
    qtest::expect(io.status().state == qc::RobotIOState::Ready, "长时间运行后仍为 Ready");

    constexpr qc::Nanoseconds kExpectedTimeNs = 2'000'000'000;
    qtest::expect_close(
        static_cast<double>(state.header.timestamp_ns),
        static_cast<double>(kExpectedTimeNs),
        0.0,
        "1000 步后仿真时间为 2 s");

    for (std::size_t i = 0; i < state.joint_count; ++i)
    {
        qtest::expect(
            std::isfinite(state.joints[i].position) &&
                std::isfinite(state.joints[i].velocity) &&
                std::isfinite(state.joints[i].effort),
            "长时间运行后关节状态有限");
    }
    for (std::size_t i = 0; i < 4; ++i)
    {
        qtest::expect(std::isfinite(state.imu.orientation[i]), "长时间运行后 IMU 姿态有限");
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        qtest::expect(
            std::isfinite(state.imu.angular_velocity[i]) &&
                std::isfinite(state.imu.linear_acceleration[i]),
            "长时间运行后 IMU 速度/加速度有限");
    }
}

}  // namespace

int main()
{
    test_deterministic_run();
    test_damping_does_not_accelerate();
    test_joint_impedance_direction();
    test_long_run_stays_finite();

    if (qtest::failures != 0)
    {
        std::cerr << qtest::failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All quadruped_mujoco determinism tests passed\n";
    return 0;
}
