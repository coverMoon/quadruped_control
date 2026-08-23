/**
 * @file motion_tests.cpp
 * @brief 测试 MotionRuntime 的状态转换、两段起立插值、Stand 和 GetDown 运动行为。
 */

#include "motion_test_helpers.hpp"

namespace
{

using motion_test::expect;
using motion_test::expect_close;

class FakePolicy final : public qm::Policy
{
public:
    qm::RlInferenceOutput forward(const qm::RlInferenceInput& input) override
    {
        ++forward_count;
        last_input = input;
        qm::RlInferenceOutput output;
        if (fail_forward)
        {
            output.error_message = "injected policy load failure";
            return output;
        }
        output.ok = true;
        output.actions.fill(action);
        return output;
    }

    int forward_count{0};
    bool fail_forward{false};
    float action{0.0F};
    qm::RlInferenceInput last_input{};
};

qc::RobotModel make_rl_model()
{
    auto model = motion_test::make_test_model();
    model.name = "black";
    return model;
}

qm::RlConfig make_rl_config(
    const std::string& name,
    const std::array<double, qc::kMaxJoints>& default_positions)
{
    qm::RlConfig config;
    config.name = name;
    config.robot_name = "black";
    config.model_path = "/tmp/" + name + ".pt";
    config.command_scale = {1.0, 1.0, 1.0};
    config.command_limits = {2.0, 2.0, 2.0};
    config.angular_velocity_scale = 1.0;
    config.joint_position_scale = 1.0;
    config.joint_velocity_scale = 1.0;
    config.observation_clip = 100.0;
    config.action_scale = 0.25;
    config.action_clip = 100.0;
    config.max_position_jump = 1.0;
    for (std::size_t i = 0; i < qm::kRlJointCount; ++i)
    {
        config.default_joint_positions[i] = default_positions[i];
        config.kp[i] = 40.0;
        config.kd[i] = 1.2;
    }
    return config;
}

qm::MotionRuntime::CreateResult make_rl_runtime()
{
    auto created = qm::MotionRuntime::create(make_rl_model(), motion_test::make_test_config());
    expect(created.ok(), "black RL 测试运行时应创建成功：" + created.error_message);
    return created;
}

qc::BaseCommand make_base_command()
{
    qc::BaseCommand command;
    command.sequence = 1;
    command.expires_at_ns = 100'000'000;
    command.source = qc::CommandSource::Test;
    command.priority = 1;
    command.vx = 0.6;
    return command;
}

qm::MotionUpdateOutput update_with_command(
    qm::MotionRuntime& runtime,
    motion_test::FakeRobotIO& io,
    const qc::BaseCommand& command,
    const qc::ModeRequest* request = nullptr)
{
    qm::MotionUpdateInput input;
    input.now_ns = 0;
    input.base_command = &command;
    input.request = request;
    return runtime.update(io, input);
}

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

// 会话变化必须中止旧请求、清空动作状态，并让新会话从 Passive 重新开始。
void test_session_change_returns_to_passive()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());

    const auto getup = motion_test::make_request(1, qc::ModeRequestType::GetUp);
    const auto accepted = motion_test::update(*created.runtime, io, &getup);
    expect(accepted.result.state == qc::ModeResultState::Accepted,
        "旧会话的 GetUp 应先被接受");
    expect(accepted.status.mode == qc::MotionMode::GetUp,
        "旧会话应进入 GetUp");

    io.state.header.session_id = 2;
    io.state.header.sequence = 1;
    const auto switched = motion_test::update(*created.runtime, io);
    expect(switched.status.mode == qc::MotionMode::Passive,
        "会话变化后必须回到 Passive");
    expect(!io.submitted.empty() &&
            io.submitted.back().header.session_id == 2 &&
            io.submitted.back().header.sequence == 1,
        "新会话的首条命令必须使用新会话号和序号 1");
    expect(io.submitted.back().joints[0].mode == qc::ControlMode::Disabled,
        "会话变化周期必须提交 Disabled 命令");
    expect(switched.result_event_count == 1,
        "会话变化应交付旧活动请求的终态");
    expect(switched.result_events[0].request_id == getup.request_id &&
            switched.result_events[0].state == qc::ModeResultState::Failed,
        "旧会话的 GetUp 必须以 Failed 结束");

    const auto restarted = motion_test::update(*created.runtime, io, &getup);
    expect(restarted.result.state == qc::ModeResultState::Accepted,
        "新会话可重新使用请求编号，旧请求不能继续执行");
    expect(restarted.status.mode == qc::MotionMode::GetUp,
        "新会话重新提交 GetUp 后才能恢复主动动作");
}

// 请求重试返回当前生命周期状态，乱序编号不能覆盖正在执行的新请求。
void test_request_lifecycle_and_ordering()
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

    auto getup = motion_test::make_request(10, qc::ModeRequestType::GetUp);
    getup.timestamp_ns = 1;
    qm::MotionUpdateInput first_input;
    first_input.now_ns = 1;
    first_input.request = &getup;
    const auto accepted = created.runtime->update(io, first_input);
    expect(accepted.result.state == qc::ModeResultState::Accepted,
        "新请求首次提交应返回 Accepted");
    expect(created.runtime->query_result(10).state == qc::ModeResultState::Running,
        "请求开始执行后查询应返回 Running");

    const std::size_t command_count = io.submitted.size();
    qm::MotionUpdateInput retry_input;
    retry_input.now_ns = 0;
    retry_input.request = &getup;
    const auto retried = created.runtime->update(io, retry_input);
    expect(retried.result.state == qc::ModeResultState::Running,
        "活动请求重试应返回 Running");
    expect(io.submitted.size() == command_count + 1,
        "活动请求重试只能继续一个控制周期，不能重新创建动作");

    const auto old_request = motion_test::make_request(9, qc::ModeRequestType::GetUp);
    const auto old_result = motion_test::update(*created.runtime, io, &old_request);
    expect(old_result.result.state == qc::ModeResultState::Rejected,
        "乱序旧 request_id 应被拒绝");
    expect(created.runtime->query_result(10).state == qc::ModeResultState::Running,
        "旧 request_id 不能覆盖活动请求");

    qm::MotionUpdateOutput completed_cycle;
    for (int i = 0; i < 3 && completed_cycle.result_event_count == 0; ++i)
    {
        completed_cycle = motion_test::update(*created.runtime, io);
    }
    expect(completed_cycle.result_event_count == 1 &&
            completed_cycle.result_events[0].request_id == getup.request_id &&
            completed_cycle.result_events[0].state == qc::ModeResultState::Completed,
        "动作完成周期必须交付 Completed 终态事件");
    expect(created.runtime->query_result(10).state == qc::ModeResultState::Completed,
        "动作完成后查询应返回 Completed");
    const auto completed = created.runtime->query_result(10);
    expect(completed.request_id == 10, "终态查询必须保留原 request_id");
    expect(completed.state == qc::ModeResultState::Completed,
        "终态查询与动作完成状态必须一致");
}

// GetUp/GetDown 可以互相打断；EnterPassive 可以打断任意主动请求。
// Passive 下的策略切换仍应拒绝。
void test_request_interruptions_and_explicit_rejections()
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
    motion_test::update(*created.runtime, io);

    const auto getup = motion_test::make_request(1, qc::ModeRequestType::GetUp);
    motion_test::update(*created.runtime, io, &getup);

    const auto getdown = motion_test::make_request(2, qc::ModeRequestType::GetDown);
    const auto getdown_result = motion_test::update(*created.runtime, io, &getdown);
    expect(getdown_result.result.state == qc::ModeResultState::Accepted,
        "GetDown 应能明确打断 GetUp");
    expect(getdown_result.result_event_count == 1 &&
            getdown_result.result_events[0].request_id == getup.request_id &&
            getdown_result.result_events[0].state == qc::ModeResultState::Failed,
        "GetDown 打断 GetUp 时旧请求必须交付 Failed");
    expect(created.runtime->query_result(getup.request_id).state == qc::ModeResultState::Failed,
        "被打断的 GetUp 查询必须返回 Failed");

    const auto getup_again = motion_test::make_request(3, qc::ModeRequestType::GetUp);
    const auto getup_result = motion_test::update(*created.runtime, io, &getup_again);
    expect(getup_result.result.state == qc::ModeResultState::Accepted,
        "GetUp 应能明确打断 GetDown");
    expect(getup_result.result_event_count == 1 &&
            getup_result.result_events[0].request_id == getdown.request_id &&
            getup_result.result_events[0].state == qc::ModeResultState::Failed,
        "GetUp 打断 GetDown 时旧请求必须交付 Failed");

    const auto passive = motion_test::make_request(4, qc::ModeRequestType::EnterPassive);
    const auto passive_result = motion_test::update(*created.runtime, io, &passive);
    expect(passive_result.result.state == qc::ModeResultState::Completed,
        "EnterPassive 应立即完成");
    expect(passive_result.status.mode == qc::MotionMode::Passive,
        "EnterPassive 后必须进入 Passive");
    expect(passive_result.result_event_count == 1 &&
            passive_result.result_events[0].request_id == getup_again.request_id &&
            passive_result.result_events[0].state == qc::ModeResultState::Failed,
        "EnterPassive 打断活动请求时必须交付 Failed");

    auto switch_policy = motion_test::make_request(5, qc::ModeRequestType::SwitchPolicy);
    switch_policy.policy_name = "obstacle";
    const auto switch_result = motion_test::update(*created.runtime, io, &switch_policy);
    expect(switch_result.result.state == qc::ModeResultState::Rejected,
        "阶段 2 不应执行策略切换，但必须返回明确 Rejected");

    const auto reset_fault = motion_test::make_request(6, qc::ModeRequestType::ResetFault);
    const auto reset_result = motion_test::update(*created.runtime, io, &reset_fault);
    expect(reset_result.result.state == qc::ModeResultState::Rejected,
        "RobotIO 尚未提供故障复位边界时 ResetFault 必须明确拒绝");
}

// 主动动作运行时发生 RobotIO fault 必须失败、回到 Passive，并拒绝新的主动请求。
void test_fault_fails_active_request()
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

    const auto getup = motion_test::make_request(1, qc::ModeRequestType::GetUp);
    motion_test::update(*created.runtime, io, &getup);

    io.read_code = qc::RobotIOCode::Fault;
    const auto failed = motion_test::update(*created.runtime, io);
    expect(failed.status.mode == qc::MotionMode::Passive,
        "RobotIO fault 后 MotionRuntime 必须回到 Passive");
    expect(failed.result_event_count == 1 &&
            failed.result_events[0].request_id == getup.request_id &&
            failed.result_events[0].state == qc::ModeResultState::Failed,
        "RobotIO fault 必须交付活动请求 Failed 终态");
    expect(!failed.status.error_message.empty(), "RobotIO fault 必须写入最近错误");
    expect(created.runtime->query_result(getup.request_id).state == qc::ModeResultState::Failed,
        "RobotIO fault 后终态查询必须返回 Failed");

    const auto new_getup = motion_test::make_request(2, qc::ModeRequestType::GetUp);
    const auto rejected = motion_test::update(*created.runtime, io, &new_getup);
    expect(rejected.result.state == qc::ModeResultState::Rejected,
        "RobotIO fault 期间新的主动请求必须拒绝");
    expect(rejected.status.mode == qc::MotionMode::Passive,
        "RobotIO fault 期间拒绝请求不能离开 Passive");
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

// rl_locomotion 必须在 Stand 且带有有效 BaseCommand 才能启动。
void test_rl_behavior_requires_base_command()
{
    auto created = make_rl_runtime();
    if (!created.ok())
    {
        return;
    }
    const auto model = make_rl_model();
    const auto stand_pose = config_stand_pose();
    FakePolicy policy;
    std::string error;
    created.runtime->attach_policy(make_rl_config("flat", stand_pose), policy, error);

    motion_test::FakeRobotIO io;
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    motion_test::drive_getup(*created.runtime, io, 1);
    io.state = motion_test::make_state(model, stand_pose);

    auto missing_command = motion_test::make_request(2, qc::ModeRequestType::StartBehavior);
    missing_command.behavior_name = "rl_locomotion";
    const auto rejected = motion_test::update(*created.runtime, io, &missing_command);
    expect(rejected.result.state == qc::ModeResultState::Rejected,
        "没有 BaseCommand 时启动 rl_locomotion 必须拒绝");
    expect(rejected.result.message == "rl_locomotion requires a valid BaseCommand",
        "缺少 BaseCommand 的拒绝原因必须明确");
    expect(rejected.status.mode == qc::MotionMode::Stand,
        "启动条件不满足时不得离开 Stand");

    const auto command = make_base_command();
    auto start = motion_test::make_request(3, qc::ModeRequestType::StartBehavior);
    start.behavior_name = "rl_locomotion";
    const auto accepted = update_with_command(*created.runtime, io, command, &start);
    expect(accepted.status.mode == qc::MotionMode::Running,
        "提供有效 BaseCommand 后应允许启动 rl_locomotion");
    expect(policy.forward_count == 1, "rl_locomotion 启动后应在首次周期推理");
    expect(created.runtime->query_result(start.request_id).state ==
            qc::ModeResultState::Completed,
        "首次推理成功后 StartBehavior 应完成");

    const auto duplicate = update_with_command(*created.runtime, io, command, &start);
    expect(duplicate.result.state == qc::ModeResultState::Completed,
        "重复 StartBehavior 请求必须返回已保存的 Completed");
    expect(policy.forward_count == 1, "重复请求不得重启策略或额外推理");

    const auto passive = motion_test::make_request(4, qc::ModeRequestType::EnterPassive);
    const auto stopped = update_with_command(*created.runtime, io, command, &passive);
    expect(stopped.result.state == qc::ModeResultState::Completed &&
            stopped.status.mode == qc::MotionMode::Passive,
        "EnterPassive 必须结束 rl_locomotion 并进入 Passive");

    motion_test::drive_getup(*created.runtime, io, 5);
    auto restart = motion_test::make_request(6, qc::ModeRequestType::StartBehavior);
    restart.behavior_name = "rl_locomotion";
    update_with_command(*created.runtime, io, command, &restart);
    io.state.header.session_id = 2;
    const auto reset = update_with_command(*created.runtime, io, command);
    expect(reset.status.mode == qc::MotionMode::Passive &&
            reset.status.behavior_name.empty(),
        "会话变化必须终止 rl_locomotion 并清空行为状态");

    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    io.state.header.session_id = 2;
    motion_test::drive_getup(*created.runtime, io, 1);
    io.state = motion_test::make_state(model, stand_pose);
    io.state.header.session_id = 2;
    policy.fail_forward = true;
    auto failing_start = motion_test::make_request(2, qc::ModeRequestType::StartBehavior);
    failing_start.behavior_name = "rl_locomotion";
    const auto failed = update_with_command(
        *created.runtime, io, command, &failing_start);
    expect(failed.status.mode == qc::MotionMode::Passive,
        "rl_locomotion 首次推理失败后必须进入 Passive");
    expect(created.runtime->query_result(failing_start.request_id).state ==
            qc::ModeResultState::Failed,
        "rl_locomotion 首次推理失败必须返回 Failed");
}

// 已加载策略可以在 Running 中直接 reload，并保留最新 BaseCommand。
void test_direct_policy_switches()
{
    auto created = make_rl_runtime();
    if (!created.ok())
    {
        return;
    }

    const auto model = make_rl_model();
    const auto flat_pose = config_stand_pose();
    auto obstacle_pose = flat_pose;
    for (std::size_t i = 0; i < qm::kRlJointCount; ++i)
    {
        obstacle_pose[i] += (i % 2 == 0) ? 0.05 : -0.05;
    }
    FakePolicy flat_policy;
    FakePolicy obstacle_policy;
    FakePolicy unloaded_policy;
    std::string error;
    expect(created.runtime->attach_policy(
        make_rl_config("flat", flat_pose), flat_policy, error),
        "flat 策略应接入：" + error);
    expect(created.runtime->register_policy(
        make_rl_config("obstacle", obstacle_pose), obstacle_policy, error),
        "obstacle 策略应注册：" + error);
    expect(created.runtime->set_policy_cycle({"flat", "obstacle"}, 2, error),
        "策略循环应按配置顺序设置：" + error);
    auto unloaded_config = make_rl_config("unloaded", obstacle_pose);
    unloaded_config.model_path.clear();
    expect(!created.runtime->register_policy(unloaded_config, unloaded_policy, error),
        "未通过加载配置校验的策略不得进入策略目录");

    motion_test::FakeRobotIO io;
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    motion_test::drive_getup(*created.runtime, io, 1);
    io.state = motion_test::make_state(model, flat_pose);

    const auto command = make_base_command();
    auto start = motion_test::make_request(2, qc::ModeRequestType::StartBehavior);
    start.behavior_name = "rl_locomotion";
    update_with_command(*created.runtime, io, command, &start);
    expect(flat_policy.forward_count == 1, "启动 RL 时 flat 应完成首次推理");

    auto obstacle = motion_test::make_request(3, qc::ModeRequestType::SwitchPolicy);
    obstacle.policy_name = "obstacle";
    const auto switched = update_with_command(*created.runtime, io, command, &obstacle);
    expect(switched.result.state == qc::ModeResultState::Accepted,
        "flat → obstacle 直接 reload 应接受");
    expect(switched.status.policy_name == "obstacle", "直接 reload 后应报告 obstacle");
    expect(obstacle_policy.forward_count == 1, "obstacle 应在直接 reload 周期首次推理");
    expect(flat_policy.forward_count == 1, "直接 reload 周期不得继续调用旧 flat 策略");
    expect(switched.status.active_source == qc::CommandSource::Test,
        "直接 reload 后同周期 BaseCommand 仍应有效");
    expect_close({obstacle_policy.last_input.observation[0], 0.6, 1.0e-6,
        "切换后新策略应收到最新 vx 命令"});
    for (std::size_t i = qm::kRlObservationDim; i < qm::kRlInputDim; ++i)
    {
        expect_close({obstacle_policy.last_input.observation[i], 0.0, 1.0e-9,
            "切换后旧 RL 历史必须清零"});
    }
    expect(created.runtime->query_result(obstacle.request_id).state ==
            qc::ModeResultState::Completed,
        "直接 reload 应在新策略首次推理后完成请求");

    io.state = motion_test::make_state(model, obstacle_pose);
    auto flat = motion_test::make_request(4, qc::ModeRequestType::SwitchPolicy);
    flat.policy_name = "flat";
    update_with_command(*created.runtime, io, command, &flat);
    expect(flat_policy.forward_count == 2, "obstacle → flat 应直接 reload 并重新推理");
    expect(obstacle_policy.forward_count == 1, "切回 flat 时不得再次调用 obstacle");

    auto toggle = motion_test::make_request(5, qc::ModeRequestType::SwitchPolicy);
    toggle.policy_name = "toggle";
    const auto toggled = update_with_command(*created.runtime, io, command, &toggle);
    expect(toggled.result.state == qc::ModeResultState::Accepted,
        "toggle 应按策略循环切换到 obstacle");
    expect(toggled.status.policy_name == "obstacle",
        "flat 状态的 toggle 目标应为循环下一项 obstacle");

    auto unknown = motion_test::make_request(6, qc::ModeRequestType::SwitchPolicy);
    unknown.policy_name = "missing";
    const auto rejected = update_with_command(*created.runtime, io, command, &unknown);
    expect(rejected.result.state == qc::ModeResultState::Rejected,
        "未知或未加载策略必须拒绝");
    expect(rejected.status.mode == qc::MotionMode::Running,
        "拒绝未知策略后应保持当前 RL 行为");

    auto unloaded = motion_test::make_request(7, qc::ModeRequestType::SwitchPolicy);
    unloaded.policy_name = "unloaded";
    const auto unloaded_result =
        update_with_command(*created.runtime, io, command, &unloaded);
    expect(unloaded_result.result.state == qc::ModeResultState::Rejected,
        "未成功加载和注册的策略必须拒绝切换");
}

// 姿态不匹配时先执行固定周期阻抗过渡，过渡完成后才允许目标策略推理。
void test_policy_transition_and_failure()
{
    auto created = make_rl_runtime();
    if (!created.ok())
    {
        return;
    }

    const auto model = make_rl_model();
    const auto flat_pose = config_stand_pose();
    auto far_pose = flat_pose;
    for (std::size_t i = 0; i < qm::kRlJointCount; ++i)
    {
        far_pose[i] += (i % 2 == 0) ? 0.5 : -0.5;
    }
    FakePolicy flat_policy;
    FakePolicy far_policy;
    FakePolicy failing_policy;
    failing_policy.fail_forward = true;
    std::string error;
    expect(created.runtime->attach_policy(
        make_rl_config("flat", flat_pose), flat_policy, error),
        "flat 策略应接入：" + error);
    expect(created.runtime->register_policy(
        make_rl_config("far", far_pose), far_policy, error),
        "far 策略应注册：" + error);
    expect(created.runtime->register_policy(
        make_rl_config("failing", far_pose), failing_policy, error),
        "失败注入策略应注册：" + error);

    motion_test::FakeRobotIO io;
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    motion_test::drive_getup(*created.runtime, io, 1);
    io.state = motion_test::make_state(model, flat_pose);
    const auto command = make_base_command();
    auto start = motion_test::make_request(2, qc::ModeRequestType::StartBehavior);
    start.behavior_name = "rl_locomotion";
    update_with_command(*created.runtime, io, command, &start);

    auto switch_far = motion_test::make_request(3, qc::ModeRequestType::SwitchPolicy);
    switch_far.policy_name = "far";
    const auto accepted = update_with_command(*created.runtime, io, command, &switch_far);
    expect(accepted.result.state == qc::ModeResultState::Accepted,
        "姿态不匹配的策略切换应接受并进入 transition");
    expect(accepted.status.behavior_phase == "policy_transition",
        "姿态不匹配时应报告 policy_transition");
    expect(io.submitted.back().joints[0].mode == qc::ControlMode::JointImpedance,
        "策略过渡必须输出位置阻抗命令");
    expect(flat_policy.forward_count == 1 && far_policy.forward_count == 0,
        "策略过渡首周期不得执行旧策略或目标策略推理");

    for (int cycle = 1; cycle < 20; ++cycle)
    {
        update_with_command(*created.runtime, io, command);
    }
    expect(created.runtime->query_result(switch_far.request_id).state ==
            qc::ModeResultState::Completed,
        "固定周期过渡完成后切换请求应 Completed");
    expect(far_policy.forward_count == 0,
        "完成过渡的命令周期仍不应执行目标策略推理");
    expect(io.submitted.back().joints[0].target_position == far_pose[0],
        "过渡最后一周期应到达目标策略默认姿态");

    io.state = motion_test::make_state(model, far_pose);
    const auto resumed = update_with_command(*created.runtime, io, command);
    expect(far_policy.forward_count == 1, "过渡完成后下一周期应恢复目标策略推理");
    expect(resumed.status.policy_name == "far", "过渡完成后当前策略应更新为 far");
    expect(resumed.status.active_source == qc::CommandSource::Test,
        "过渡期间持续接收的 BaseCommand 应在恢复 RL 后生效");

    auto switch_failing = motion_test::make_request(4, qc::ModeRequestType::SwitchPolicy);
    switch_failing.policy_name = "failing";
    const auto failed = update_with_command(*created.runtime, io, command, &switch_failing);
    expect(failed.status.mode == qc::MotionMode::Passive,
        "目标策略首次推理失败后必须进入 Passive");
    expect(created.runtime->query_result(switch_failing.request_id).state ==
            qc::ModeResultState::Failed,
        "目标策略加载后的首次推理失败必须返回 Failed");
    expect(!failed.status.error_message.empty(), "策略切换失败必须保留错误说明");
    expect(io.submitted.back().joints[0].mode == qc::ControlMode::Disabled,
        "策略切换失败后必须清空旧 RL 命令并提交 Disabled");
}

// EnterPassive 在策略过渡期间拥有最高打断优先级。
void test_policy_transition_interrupted_by_passive()
{
    auto created = make_rl_runtime();
    if (!created.ok())
    {
        return;
    }
    const auto model = make_rl_model();
    const auto flat_pose = config_stand_pose();
    auto far_pose = flat_pose;
    far_pose[0] += 0.5;
    FakePolicy flat_policy;
    FakePolicy far_policy;
    std::string error;
    created.runtime->attach_policy(make_rl_config("flat", flat_pose), flat_policy, error);
    created.runtime->register_policy(make_rl_config("far", far_pose), far_policy, error);

    motion_test::FakeRobotIO io;
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    motion_test::drive_getup(*created.runtime, io, 1);
    io.state = motion_test::make_state(model, flat_pose);
    auto start = motion_test::make_request(2, qc::ModeRequestType::StartBehavior);
    start.behavior_name = "rl_locomotion";
    const auto command = make_base_command();
    update_with_command(*created.runtime, io, command, &start);
    auto switch_far = motion_test::make_request(3, qc::ModeRequestType::SwitchPolicy);
    switch_far.policy_name = "far";
    update_with_command(*created.runtime, io, command, &switch_far);

    const auto passive = motion_test::make_request(4, qc::ModeRequestType::EnterPassive);
    const auto interrupted = motion_test::update(*created.runtime, io, &passive);
    expect(interrupted.result.state == qc::ModeResultState::Completed,
        "策略过渡期间 EnterPassive 应立即完成");
    expect(interrupted.status.mode == qc::MotionMode::Passive,
        "EnterPassive 应立即结束策略过渡并进入 Passive");
    expect(created.runtime->query_result(switch_far.request_id).state ==
            qc::ModeResultState::Failed,
        "被 EnterPassive 打断的策略切换应返回 Failed");
    expect(far_policy.forward_count == 0, "被打断后目标策略不得推理");
}

}  // namespace

int main()
{
    test_initial_passive();
    test_getup_two_stage_and_stand();
    test_stand_and_getdown_rejections();
    test_getdown_returns_to_rest();
    test_getdown_interrupted_by_getup();
    test_session_change_returns_to_passive();
    test_request_lifecycle_and_ordering();
    test_rl_behavior_requires_base_command();
    test_request_interruptions_and_explicit_rejections();
    test_fault_fails_active_request();
    test_direct_policy_switches();
    test_policy_transition_and_failure();
    test_policy_transition_interrupted_by_passive();

    if (motion_test::failures > 0)
    {
        std::cerr << motion_test::failures << " 个运动测试失败\n";
        return 1;
    }
    std::cout << "全部运动测试通过\n";
    return 0;
}
