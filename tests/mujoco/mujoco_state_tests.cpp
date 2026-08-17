/**
 * @file mujoco_state_tests.cpp
 * @brief 测试 MujocoRobotIO 的 reset、StateFrame 生成和 RobotIO 读取语义。
 */

#include "quadruped/backends/mujoco/mujoco_model.hpp"
#include "quadruped/backends/mujoco/mujoco_robot_io.hpp"
#include "quadruped/core/core.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

namespace qc = quadruped::core;
namespace qm = quadruped::backends::mujoco;

namespace
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

const std::string kBlackScenePath = QUADRUPED_BLACK_SCENE_PATH;

// 正式 black 模型的逻辑关节顺序，必须与 configs/robots/black.yaml 一致。
qc::RobotModel make_black_model()
{
    qc::RobotModel model;
    model.name = "black";
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

// 校验 header 的关键字段。
void verify_header(
    const qc::StateFrame& frame,
    const std::uint64_t startup_id,
    const std::uint64_t session_id)
{
    expect(frame.header.startup_id == startup_id, "header.startup_id 与创建时一致");
    expect(frame.header.session_id == session_id, "header.session_id 与 reset 一致");
    expect(frame.header.sequence == 1, "当前会话第一帧 sequence=1");
    expect(frame.header.timestamp_ns == 0, "reset 后仿真时间为 0 ns");
    expect(frame.joint_count == 12, "joint_count 为 12");
    expect(frame.safety_state == qc::SafetyState::Damping, "safety_state=Damping");
    expect(frame.last_accepted_command_sequence == 0, "last_accepted_command_sequence=0");
    expect(frame.effective_command_sequence == 0, "effective_command_sequence=0");
}

// 通过映射地址读取 keyframe 期望值，验证关节顺序与 RobotModel 完全一致。
void verify_joint_states(
    const qc::StateFrame& frame,
    const qc::RobotModel& robot,
    const qm::MujocoModel& reference,
    const int key_id)
{
    const mjModel* raw = reference.raw_model();
    for (std::size_t i = 0; i < robot.joint_count; ++i)
    {
        const auto& mapping = reference.joint_mappings()[i];
        const double expected_position = raw->key_qpos[key_id * raw->nq + mapping.qpos_address];
        const double expected_effort = raw->key_ctrl[key_id * raw->nu + mapping.actuator_id];
        expect_close(
            frame.joints[i].position, expected_position, 0.0,
            robot.joints[i].name + " 位置等于 keyframe qpos");
        expect_close(frame.joints[i].velocity, 0.0, 0.0, robot.joints[i].name + " 速度为 0");
        expect_close(
            frame.joints[i].effort, expected_effort, 1e-12,
            robot.joints[i].name + " 力矩等于 keyframe ctrl");
        expect(frame.joints[i].online, robot.joints[i].name + " online=true");
        expect(frame.joints[i].valid, robot.joints[i].name + " valid=true");
        expect(frame.joints[i].age_ns == 0, robot.joints[i].name + " age_ns=0");
        expect(frame.joints[i].temperature_c == 0.0, robot.joints[i].name + " temperature_c=0");
    }
}

// 校验 IMU 四元数顺序、数值和有限性。reset 时机体姿态为单位四元数。
void verify_imu(const qc::StateFrame& frame)
{
    expect_close(frame.imu.orientation[0], 1.0, 1e-6, "imu_quat w=1");
    expect_close(frame.imu.orientation[1], 0.0, 1e-6, "imu_quat x=0");
    expect_close(frame.imu.orientation[2], 0.0, 1e-6, "imu_quat y=0");
    expect_close(frame.imu.orientation[3], 0.0, 1e-6, "imu_quat z=0");
    const double norm_sq = frame.imu.orientation[0] * frame.imu.orientation[0] +
        frame.imu.orientation[1] * frame.imu.orientation[1] +
        frame.imu.orientation[2] * frame.imu.orientation[2] +
        frame.imu.orientation[3] * frame.imu.orientation[3];
    expect_close(std::sqrt(norm_sq), 1.0, 1e-6, "IMU 四元数模长接近 1");
    for (std::size_t i = 0; i < 3; ++i)
    {
        expect(std::isfinite(frame.imu.angular_velocity[i]), "imu_gyro 分量有限");
        expect(std::isfinite(frame.imu.linear_acceleration[i]), "imu_acc 分量有限");
    }
    expect(frame.imu.age_ns == 0, "imu age_ns=0");
    expect(frame.imu.valid, "imu valid=true");
}

void test_black_reset_produces_valid_state()
{
    const qc::RobotModel robot = make_black_model();
    constexpr std::uint64_t kStartupId = 0x1234;
    constexpr std::uint64_t kSessionId = 42;

    // 独立加载一份模型，仅用于通过映射地址读取 keyframe 的期望值。
    auto reference = qm::MujocoModel::load(kBlackScenePath, robot);
    if (!reference.ok())
    {
        expect(false, "为测试加载正式模型失败：" + reference.error_message);
        return;
    }
    const int key_id = mj_name2id(reference.model->raw_model(), mjOBJ_KEY, "default_pose");
    if (key_id < 0)
    {
        expect(false, "正式模型缺少 default_pose keyframe");
        return;
    }

    auto created = qm::MujocoRobotIO::create(kBlackScenePath, robot, kStartupId);
    if (!created.ok())
    {
        expect(false, "创建正式模型失败：" + created.error_message);
        return;
    }
    qm::MujocoRobotIO& io = *created.io;

    qc::StateFrame before;
    expect(io.read_latest(before) == qc::RobotIOCode::NoData, "reset 前 read_latest 返回 NoData");

    if (!io.reset(kSessionId).ok())
    {
        expect(false, "正式 default_pose reset 失败");
        return;
    }

    qc::StateFrame frame;
    expect(io.read_latest(frame) == qc::RobotIOCode::Ok, "reset 后 read_latest 返回 Ok");
    expect(io.status().dropped_state_frames == 0, "首次生成状态不记为丢帧");

    verify_header(frame, kStartupId, kSessionId);
    verify_joint_states(frame, robot, *reference.model, key_id);
    verify_imu(frame);

    const auto validation = qc::validate(frame, robot, frame.header.timestamp_ns);
    expect(validation.ok(), "StateFrame 通过 core::validate：" + validation.message);
}

void test_repeated_read_is_stable()
{
    auto created = qm::MujocoRobotIO::create(kBlackScenePath, make_black_model(), 7);
    if (!created.ok() || !created.io->reset(1).ok())
    {
        expect(false, "创建并 reset 正式模型失败");
        return;
    }
    qm::MujocoRobotIO& io = *created.io;
    qc::StateFrame first;
    qc::StateFrame second;
    expect(io.read_latest(first) == qc::RobotIOCode::Ok, "第一次读取返回 Ok");
    const auto dropped = io.status().dropped_state_frames;
    expect(io.read_latest(second) == qc::RobotIOCode::Ok, "第二次读取返回 Ok");
    expect(second.header.sequence == first.header.sequence, "重复读取返回相同 sequence");
    expect(io.status().dropped_state_frames == dropped, "重复读取不增加丢帧计数");
}

void test_reset_new_session_restarts()
{
    auto created = qm::MujocoRobotIO::create(kBlackScenePath, make_black_model(), 7);
    if (!created.ok() || !created.io->reset(1).ok())
    {
        expect(false, "创建并 reset 正式模型失败");
        return;
    }
    qm::MujocoRobotIO& io = *created.io;
    qc::StateFrame first;
    expect(io.read_latest(first) == qc::RobotIOCode::Ok, "读取第一会话状态");
    expect(io.reset(2).ok(), "使用新 session_id 再次 reset 成功");

    qc::StateFrame second;
    expect(io.read_latest(second) == qc::RobotIOCode::Ok, "读取第二会话状态");
    expect(second.header.session_id == 2, "第二会话 session_id 正确");
    expect(second.header.sequence == 1, "新会话 sequence 重新从 1 开始");
    for (std::size_t i = 0; i < first.joint_count; ++i)
    {
        expect_close(
            second.joints[i].position, first.joints[i].position, 0.0,
            "重复 reset 后姿态恢复一致");
    }
}

void test_unread_state_dropped_on_new_reset()
{
    auto created = qm::MujocoRobotIO::create(kBlackScenePath, make_black_model(), 7);
    if (!created.ok() || !created.io->reset(1).ok() || !created.io->reset(2).ok())
    {
        expect(false, "连续 reset 正式模型失败");
        return;
    }
    expect(
        created.io->status().dropped_state_frames == 1,
        "覆盖尚未读取的旧状态时丢帧计数加一");
}

void test_submit_rejected()
{
    auto created = qm::MujocoRobotIO::create(kBlackScenePath, make_black_model(), 7);
    if (!created.ok() || !created.io->reset(1).ok())
    {
        expect(false, "创建并 reset 正式模型失败");
        return;
    }
    qc::CommandFrame command;
    expect(created.io->submit(command) == qc::RobotIOCode::Rejected, "submit 返回 Rejected");
    expect(created.io->status().rejected_command_frames == 1, "rejected_command_frames 加一");
    expect(created.io->status().latest_command_sequence == 0, "latest_command_sequence 保持 0");
    expect(created.io->status().state == qc::RobotIOState::Ready, "submit 后变为 Ready");
}

}  // 匿名命名空间

int main()
{
    test_black_reset_produces_valid_state();
    test_repeated_read_is_stable();
    test_reset_new_session_restarts();
    test_unread_state_dropped_on_new_reset();
    test_submit_rejected();

    if (failures != 0)
    {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All quadruped_mujoco state tests passed\n";
    return 0;
}
