/**
 * @file motion_tests.cpp
 * @brief 测试 MotionRuntime 的状态转换、两段起立插值、Stand 和 GetDown 运动行为。
 */

#include "motion_test_helpers.hpp"

namespace
{

using motion_test::expect;
using motion_test::expect_close;
// 校验最近一条命令的所有关节目标位置等于给定数组。
void expect_command_positions(
    const motion_test::FakeRobotIO& io,
    const std::array<double, qc::kMaxJoints>& expected,
    const std::string& description)
{
    constexpr double tolerance = 1e-9;
    expect(!io.submitted.empty(), description + "（应已提交命令）");
    const auto& command = io.submitted.back();
    for (std::size_t i = 0; i < command.joint_count; ++i)
    {
        expect_close({command.joints[i].target_position, expected[i], tolerance,
            description + "（关节 " + std::to_string(i) + "）"});
    }
}

std::array<double, qc::kMaxJoints> lerp_positions(
    const std::array<double, qc::kMaxJoints>& start,
    const std::array<double, qc::kMaxJoints>& target,
    const double percent)
{
    std::array<double, qc::kMaxJoints> result{};
    for (std::size_t i = 0; i < result.size(); ++i)
    {
        result[i] = (1.0 - percent) * start[i] + percent * target[i];
    }
    return result;
}

std::array<double, qc::kMaxJoints> config_pre_pose()
{
    return motion_test::make_test_config().pre_getup_position;
}

std::array<double, qc::kMaxJoints> config_stand_pose()
{
    return motion_test::make_test_config().stand_position;
}

// 校验起立命令的公共字段：模式、控制方式、增益、速度和前馈。
void expect_getup_command_fields(const motion_test::FakeRobotIO& io)
{
    const auto& command = io.submitted.back();
    expect(command.motion_mode == qc::MotionMode::GetUp, "起立命令模式应为 GetUp");
    expect(command.joints[0].mode == qc::ControlMode::JointImpedance,
        "起立应使用 JointImpedance");
    expect(command.joints[0].kp == 80.0 && command.joints[0].kd == 3.0,
        "起立应使用配置的固定增益");
    expect(command.joints[0].target_velocity == 0.0 &&
        command.joints[0].feedforward_effort == 0.0,
        "起立目标速度和前馈力矩应为 0");
}

// 初始 Passive 应提交显式 Disabled 命令，并复制执行侧的启动与会话标识。
void test_initial_passive()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());

    const auto output = motion_test::update(*created.runtime, io);
    expect(output.read_code == qc::RobotIOCode::Ok, "初始周期应读取状态成功");
    expect(output.submitted, "初始 Passive 应提交 Disabled 命令");
    expect(output.status.mode == qc::MotionMode::Passive, "初始模式应为 Passive");
    expect(io.submitted.size() == 1, "初始周期应只提交一条命令");

    const auto& command = io.submitted.back();
    expect(command.header.startup_id == 1, "命令应复制执行侧 startup_id");
    expect(command.header.session_id == 1, "命令应复制执行侧 session_id");
    expect(command.header.sequence == 1, "新会话命令序号应从 1 开始");
    expect(command.expires_at_ns == 10'000'000, "命令有效期应为 now 加 10 ms");
    expect(command.joint_count == model.joint_count, "命令关节数应与模型一致");
    expect(command.motion_mode == qc::MotionMode::Passive, "命令模式应为 Passive");
    expect(command.source == qc::CommandSource::None, "M2 命令来源应为 None");
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        expect(command.joints[i].mode == qc::ControlMode::Disabled,
            "Passive 关节 " + std::to_string(i) + " 应为显式 Disabled");
    }
}

// 两段起立：落地姿态 → 预起立姿态 → 默认站姿，完成后进入 Stand 并保持。
void test_getup_two_stage_and_stand()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    const auto rest = motion_test::make_rest_positions();
    io.state = motion_test::make_state(model, rest);

    const auto request = motion_test::make_request(1, qc::ModeRequestType::GetUp);
    const auto accepted = motion_test::update(*created.runtime, io, &request);
    expect(accepted.has_result, "GetUp 请求应返回结果");
    expect(accepted.result.state == qc::ModeResultState::Accepted, "GetUp 应被接受");
    expect(accepted.status.mode == qc::MotionMode::GetUp, "接受后应进入 GetUp");
    expect_command_positions(io, lerp_positions(rest, config_pre_pose(), 0.5),
        "起立第一段第一周期应插值到中点");
    expect_getup_command_fields(io);

    motion_test::update(*created.runtime, io);
    expect_command_positions(io, config_pre_pose(), "第一段结束应到达预起立姿态");

    motion_test::update(*created.runtime, io);
    expect_command_positions(io, lerp_positions(config_pre_pose(), config_stand_pose(), 0.5),
        "第二段第一周期应从预起立姿态插值");

    motion_test::update(*created.runtime, io);
    expect_command_positions(io, config_stand_pose(), "第二段结束应到达默认站姿");
    expect(io.submitted.back().motion_mode == qc::MotionMode::GetUp,
        "到达站姿的周期仍处于 GetUp");

    motion_test::update(*created.runtime, io);
    expect(io.submitted.back().motion_mode == qc::MotionMode::Stand,
        "完成周期后应进入 Stand");

    const auto retried = motion_test::update(*created.runtime, io, &request);
    expect(retried.result.state == qc::ModeResultState::Completed,
        "起立完成后重试应报告 Completed");

    for (int i = 0; i < 3; ++i)
    {
        motion_test::update(*created.runtime, io);
    }
    expect_command_positions(io, config_stand_pose(), "Stand 应持续保持默认站姿");
}

// 从 Passive 请求 Stand 不允许跳过起立；尚未记录 rest_pose 时 GetDown 应拒绝。
void test_stand_and_getdown_rejections()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    motion_test::update(*created.runtime, io);

    const auto stand = motion_test::make_request(1, qc::ModeRequestType::Stand);
    const auto stand_result = motion_test::update(*created.runtime, io, &stand);
    expect(stand_result.result.state == qc::ModeResultState::Rejected,
        "Passive 中请求 Stand 应拒绝");
    expect(stand_result.status.mode == qc::MotionMode::Passive, "拒绝后应保持 Passive");

    const auto getdown = motion_test::make_request(2, qc::ModeRequestType::GetDown);
    const auto getdown_result = motion_test::update(*created.runtime, io, &getdown);
    expect(getdown_result.result.state == qc::ModeResultState::Rejected,
        "尚未记录 rest_pose 时 GetDown 应拒绝");

    motion_test::drive_getup(*created.runtime, io, 3);

    const auto stand_again = motion_test::make_request(4, qc::ModeRequestType::Stand);
    const auto stand_again_result = motion_test::update(*created.runtime, io, &stand_again);
    expect(stand_again_result.result.state == qc::ModeResultState::Rejected,
        "已在 Stand 时重复 Stand 请求应拒绝且不重新执行");

    const auto getup_again = motion_test::make_request(5, qc::ModeRequestType::GetUp);
    const auto getup_again_result = motion_test::update(*created.runtime, io, &getup_again);
    expect(getup_again_result.result.state == qc::ModeResultState::Rejected,
        "已在 Stand 时 GetUp 请求应拒绝");
    expect(io.submitted.back().motion_mode == qc::MotionMode::Stand,
        "拒绝后应保持 Stand");
}

// GetDown 应从接受时的当前姿态插值回首次 GetUp 记录的落地姿态，完成后进入 Passive。
void test_getdown_returns_to_rest()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    const auto rest = motion_test::make_rest_positions();
    io.state = motion_test::make_state(model, rest);
    motion_test::drive_getup(*created.runtime, io, 1);

    // 模拟机器人实际已经站稳，状态反馈为默认站姿。
    const auto stand = config_stand_pose();
    io.state = motion_test::make_state(model, stand);

    const auto request = motion_test::make_request(2, qc::ModeRequestType::GetDown);
    const auto accepted = motion_test::update(*created.runtime, io, &request);
    expect(accepted.result.state == qc::ModeResultState::Accepted, "GetDown 应被接受");
    expect(accepted.status.mode == qc::MotionMode::GetDown, "接受后应进入 GetDown");
    expect_command_positions(io, lerp_positions(stand, rest, 0.25),
        "趴下第一周期应从站姿向落地姿态插值");

    motion_test::update(*created.runtime, io);
    expect_command_positions(io, lerp_positions(stand, rest, 0.5), "趴下第二周期插值错误");
    motion_test::update(*created.runtime, io);
    expect_command_positions(io, lerp_positions(stand, rest, 0.75), "趴下第三周期插值错误");
    motion_test::update(*created.runtime, io);
    expect_command_positions(io, rest, "趴下结束应精确回到记录的落地姿态");

    const auto completed_cycle = motion_test::update(*created.runtime, io);
    expect(completed_cycle.status.mode == qc::MotionMode::Passive,
        "趴下完成后应进入 Passive");
    expect(io.submitted.back().joints[0].mode == qc::ControlMode::Disabled,
        "趴下完成周期应提交 Disabled");

    const auto retried = motion_test::update(*created.runtime, io, &request);
    expect(retried.result.state == qc::ModeResultState::Completed,
        "趴下完成后重试应报告 Completed");
}

// GetDown 期间重新请求 GetUp：从当时姿态重新起立，但保留首次记录的 rest_pose。
void test_getdown_interrupted_by_getup()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    const auto rest = motion_test::make_rest_positions();
    io.state = motion_test::make_state(model, rest);
    motion_test::drive_getup(*created.runtime, io, 1);

    io.state = motion_test::make_state(model, config_stand_pose());
    const auto getdown = motion_test::make_request(2, qc::ModeRequestType::GetDown);
    motion_test::update(*created.runtime, io, &getdown);
    motion_test::update(*created.runtime, io);

    // 模拟趴下到一半的实际姿态，重新起立应从该姿态开始插值。
    std::array<double, qc::kMaxJoints> middle{};
    for (std::size_t i = 0; i < middle.size(); ++i)
    {
        middle[i] = 0.3;
    }
    io.state = motion_test::make_state(model, middle);

    const auto getup = motion_test::make_request(3, qc::ModeRequestType::GetUp);
    const auto accepted = motion_test::update(*created.runtime, io, &getup);
    expect(accepted.result.state == qc::ModeResultState::Accepted,
        "GetDown 期间重新 GetUp 应被接受");
    expect_command_positions(io, lerp_positions(middle, config_pre_pose(), 0.5),
        "重新起立应从当时姿态开始插值");

    // 完成第二次起立后趴下，终点仍应是首次记录的落地姿态。
    for (int i = 0; i < 4; ++i)
    {
        motion_test::update(*created.runtime, io);
    }
    expect(io.submitted.back().motion_mode == qc::MotionMode::Stand,
        "第二次起立后应进入 Stand");
    io.state = motion_test::make_state(model, config_stand_pose());
    const auto getdown_again = motion_test::make_request(4, qc::ModeRequestType::GetDown);
    motion_test::update(*created.runtime, io, &getdown_again);
    for (int i = 0; i < 3; ++i)
    {
        motion_test::update(*created.runtime, io);
    }
    expect_command_positions(io, rest, "重新起立后趴下仍应回到首次记录的落地姿态");
}

}  // namespace

int main()
{
    test_initial_passive();
    test_getup_two_stage_and_stand();
    test_stand_and_getdown_rejections();
    test_getdown_returns_to_rest();
    test_getdown_interrupted_by_getup();

    if (motion_test::failures > 0)
    {
        std::cerr << motion_test::failures << " 个运动测试失败\n";
        return 1;
    }
    std::cout << "全部运动测试通过\n";
    return 0;
}
