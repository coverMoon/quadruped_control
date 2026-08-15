/**
 * @file mujoco_time_tests.cpp
 * @brief 测试仿真时间转换边界、时间错误故障路径和刷新入口的会话不变量。
 */

#include "quadruped/backends/mujoco/mujoco_robot_io.hpp"
#include "quadruped/core/core.hpp"

#include <cstdint>
#include <iostream>
#include <limits>
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

// 创建并成功 reset 一个后端；失败时记录断言并返回空结果。
qm::MujocoRobotIO::CreateResult create_ready(const std::uint64_t session_id)
{
    auto created = qm::MujocoRobotIO::create(kBlackScenePath, make_black_model(), 7);
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

void expect_time_rejected(
    qm::MujocoRobotIO& io,
    const double seconds,
    const std::string& keyword,
    const std::string& description)
{
    io.raw_data()->time = seconds;
    const std::string error = io.refresh_for_test();
    expect(!error.empty(), description + "：刷新被拒绝");
    expect(
        error.find(keyword) != std::string::npos,
        description + "：错误文本应包含 \"" + keyword + "\"，实际为 " + error);
    expect(io.status().state == qc::RobotIOState::Fault, description + "：进入 Fault");
}

void test_negative_time_rejected()
{
    auto created = create_ready(1);
    if (!created.ok())
    {
        return;
    }
    expect_time_rejected(*created.io, -1.0, "negative", "负仿真时间");
    expect_time_rejected(*created.io, -1.0e-9, "negative", "接近零的负仿真时间");
}

void test_nonfinite_time_rejected()
{
    auto created = create_ready(1);
    if (!created.ok())
    {
        return;
    }
    const double inf = std::numeric_limits<double>::infinity();
    expect_time_rejected(*created.io, std::numeric_limits<double>::quiet_NaN(), "not finite", "NaN");
    expect_time_rejected(*created.io, inf, "not finite", "正 Inf 仿真时间");
    expect_time_rejected(*created.io, -inf, "not finite", "负 Inf 仿真时间");
}

void test_time_overflow_rejected()
{
    auto created = create_ready(1);
    if (!created.ok())
    {
        return;
    }
    // 9223372036.854776 秒乘以 1e9 恰好舍入到 2^63，旧实现的 llround 在此溢出。
    expect_time_rejected(*created.io, 9223372036.854776, "exceeds", "2^63 边界的仿真时间");
    expect_time_rejected(*created.io, 1.0e300, "exceeds", "乘积为 Inf 的仿真时间");
}

void test_time_near_max_boundary()
{
    auto created = create_ready(1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;

    // 2^63 以下最大 double 纳秒是 2^63 - 1024，对应秒数 9223372036.854774；
    // 该值必须转换成功且结果精确等于 2^63 - 1024。
    io.raw_data()->time = 9223372036.854774;
    expect(io.refresh_for_test().empty(), "最大合法边界附近的仿真时间转换成功");
    qc::StateFrame frame;
    expect(io.read_latest(frame) == qc::RobotIOCode::Ok, "边界时间刷新后状态可读");
    expect(
        frame.header.timestamp_ns == 9223372036854774784LL,
        "边界时间转换结果等于 2^63 - 1024 纳秒");
}

void test_time_fault_preserves_last_valid_frame()
{
    auto created = create_ready(1);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;
    qc::StateFrame before;
    expect(io.read_latest(before) == qc::RobotIOCode::Ok, "注入前读取有效状态");

    io.raw_data()->time = -1.0;
    expect(!io.refresh_for_test().empty(), "时间错误刷新被拒绝");
    expect(io.status().state == qc::RobotIOState::Fault, "时间错误进入 Fault");

    qc::StateFrame after;
    expect(io.read_latest(after) == qc::RobotIOCode::Ok, "Fault 后上一份有效状态仍可读");
    expect(
        after.header.sequence == before.header.sequence,
        "上一份状态序号未被时间错误覆盖");
    expect(
        after.header.timestamp_ns == before.header.timestamp_ns,
        "上一份状态时间戳未被时间错误覆盖");
}

void test_refresh_before_reset_has_no_data()
{
    auto created = qm::MujocoRobotIO::create(kBlackScenePath, make_black_model(), 7);
    if (!created.ok())
    {
        expect(false, "创建正式模型失败：" + created.error_message);
        return;
    }
    qm::MujocoRobotIO& io = *created.io;
    expect(!io.refresh_for_test().empty(), "reset 前刷新被拒绝");
    qc::StateFrame frame;
    expect(io.read_latest(frame) == qc::RobotIOCode::NoData, "reset 前 read_latest 返回 NoData");
    expect(io.status().latest_state_sequence == 0, "reset 前没有已发布的状态序号");
}

void test_refresh_keeps_session_and_sequence()
{
    auto created = create_ready(5);
    if (!created.ok())
    {
        return;
    }
    qm::MujocoRobotIO& io = *created.io;

    // reset 后第一帧 sequence=1；同一会话内刷新只能使用已建立的会话，且序号单调。
    expect(io.refresh_for_test().empty(), "reset 后刷新成功");
    qc::StateFrame frame;
    expect(io.read_latest(frame) == qc::RobotIOCode::Ok, "刷新后状态可读");
    expect(frame.header.session_id == 5, "刷新帧使用已建立的会话");
    expect(frame.header.sequence == 2, "同会话内序号单调递增");

    // 相同会话的 reset 被拒绝后，当前会话和序号语义不受影响。
    expect(!io.reset(5).ok(), "相同 session_id 的 reset 被拒绝");
    expect(io.refresh_for_test().empty(), "拒绝 reset 后刷新仍成功");
    expect(io.read_latest(frame) == qc::RobotIOCode::Ok, "拒绝 reset 后状态可读");
    expect(frame.header.session_id == 5, "拒绝 reset 后会话不变");
    expect(frame.header.sequence == 3, "拒绝 reset 后序号继续递增");
}

}  // 匿名命名空间

int main()
{
    test_negative_time_rejected();
    test_nonfinite_time_rejected();
    test_time_overflow_rejected();
    test_time_near_max_boundary();
    test_time_fault_preserves_last_valid_frame();
    test_refresh_before_reset_has_no_data();
    test_refresh_keeps_session_and_sequence();

    if (failures != 0)
    {
        std::cerr << failures << " test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All quadruped_mujoco time tests passed\n";
    return 0;
}
