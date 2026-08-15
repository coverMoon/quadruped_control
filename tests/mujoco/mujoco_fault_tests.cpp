/**
 * @file mujoco_fault_tests.cpp
 * @brief 测试 MujocoRobotIO 的创建、reset 拒绝路径和刷新失败进入 Fault 的行为。
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

const std::string kBlackScenePath = QUADRUPED_BLACK_SCENE_PATH;
const std::string kFixtureDir = QUADRUPED_MUJOCO_FIXTURE_DIR;

// 正式 black 模型的逻辑关节顺序，必须与 configs/robots/black.yaml 一致。
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

// fixture 模型只有两个逻辑关节，与 fixtures/ 中的小型模型对应。
qc::RobotModel make_fixture_model()
{
    qc::RobotModel model;
    model.name = "fixture";
    model.model_id = 0x1;
    model.joint_count = 2;
    constexpr const char* names[2] = {"j1", "j2"};
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        model.joints[i].name = names[i];
        model.joints[i].role = qc::JointRole::Leg;
        model.joints[i].limits = {true, -3.0, 3.0, 20.0, 40.0, 100.0, 10.0};
    }
    return model;
}

void test_create_rejects_zero_startup_id()
{
    const auto result = qm::MujocoRobotIO::create(kBlackScenePath, make_black_model(), 0);
    expect(!result.ok(), "startup_id=0 时创建被拒绝");
    expect(!result.error_message.empty(), "startup_id=0 时错误文本非空");
}

void test_reset_rejects_zero_session()
{
    auto created = qm::MujocoRobotIO::create(kBlackScenePath, make_black_model(), 7);
    if (!created.ok())
    {
        expect(false, "创建正式模型失败：" + created.error_message);
        return;
    }
    const auto result = created.io->reset(0);
    expect(result.code == qc::RobotIOCode::Rejected, "reset(session_id=0) 返回 Rejected");
    expect(!result.error_message.empty(), "reset(session_id=0) 错误文本非空");
}

void test_reset_rejects_missing_keyframe()
{
    auto created = qm::MujocoRobotIO::create(
        kFixtureDir + "/no_default_pose.xml", make_fixture_model(), 7);
    if (!created.ok())
    {
        expect(false, "创建无 keyframe fixture 失败：" + created.error_message);
        return;
    }
    const auto result = created.io->reset(1);
    expect(result.code == qc::RobotIOCode::Fault, "找不到 default_pose 返回 Fault");
    expect(
        result.error_message.find("default_pose") != std::string::npos,
        "找不到 default_pose 的错误文本包含关键字");
    expect(
        created.io->status().state == qc::RobotIOState::Fault,
        "找不到 default_pose 后状态进入 Fault");
}

void test_reset_same_session_rejected()
{
    auto created = qm::MujocoRobotIO::create(kBlackScenePath, make_black_model(), 7);
    if (!created.ok() || !created.io->reset(1).ok())
    {
        expect(false, "创建并 reset 正式模型失败");
        return;
    }
    const auto again = created.io->reset(1);
    expect(again.code == qc::RobotIOCode::Rejected, "相同 session_id 再次 reset 被拒绝");
    qc::StateFrame frame;
    expect(created.io->read_latest(frame) == qc::RobotIOCode::Ok, "拒绝后上一份状态仍可读");
    expect(frame.header.session_id == 1, "拒绝后上一份状态 session_id 未变");
}

void test_nan_injection_faults()
{
    const qc::RobotModel robot = make_black_model();
    auto reference = qm::MujocoModel::load(kBlackScenePath, robot);
    if (!reference.ok())
    {
        expect(false, "为测试加载正式模型失败");
        return;
    }
    const int qpos_address = reference.model->joint_mappings()[0].qpos_address;

    auto created = qm::MujocoRobotIO::create(kBlackScenePath, robot, 7);
    if (!created.ok() || !created.io->reset(1).ok())
    {
        expect(false, "创建并 reset 正式模型失败");
        return;
    }
    qm::MujocoRobotIO& io = *created.io;
    qc::StateFrame before;
    expect(io.read_latest(before) == qc::RobotIOCode::Ok, "注入 NaN 前读取有效状态");

    // 在测试范围内直接修改 mjData 制造 NaN，正式运行代码不得这样做。
    io.raw_data()->qpos[qpos_address] = std::nan("");
    expect(!io.refresh_for_test().empty(), "注入 NaN 后刷新失败并返回错误文本");
    expect(io.status().state == qc::RobotIOState::Fault, "刷新失败后状态进入 Fault");

    qc::StateFrame after;
    expect(io.read_latest(after) == qc::RobotIOCode::Ok, "Fault 后仍能读取上一份有效状态");
    expect(after.header.sequence == before.header.sequence, "上一份状态序号未被覆盖");
    expect(
        after.joints[0].position == before.joints[0].position,
        "上一份状态关节位置未被覆盖");
}

}  // 匿名命名空间

int main()
{
    test_create_rejects_zero_startup_id();
    test_reset_rejects_zero_session();
    test_reset_rejects_missing_keyframe();
    test_reset_same_session_rejected();
    test_nan_injection_faults();

    if (failures != 0)
    {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All quadruped_mujoco fault tests passed\n";
    return 0;
}
