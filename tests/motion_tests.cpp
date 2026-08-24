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
        output.elapsed_ns = elapsed_ns;
        if (fail_forward)
        {
            output.error_message = "injected policy load failure";
            return output;
        }
        output.ok = true;
        output.action_dimension = output_dimension;
        output.actions.fill(action);
        return output;
    }

    int forward_count{0};
    bool fail_forward{false};
    float action{0.0F};
    std::size_t output_dimension{qm::kRlActionDim};
    qc::Nanoseconds elapsed_ns{0};
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
    config.observation_dimension = qm::kRlObservationDim;
    config.history_frame_count = qm::kRlHistoryFrames;
    config.inference_input_dimension = qm::kRlInputDim;
    config.action_dimension = qm::kRlActionDim;
    config.joint_count = qm::kRlJointCount;
    config.command_scale = {1.0, 1.0, 1.0};
    config.command_limits = {2.0, 2.0, 2.0};
    config.angular_velocity_scale = 1.0;
    config.joint_position_scale = 1.0;
    config.joint_velocity_scale = 1.0;
    config.observation_clip = 100.0;
    config.action_clip = 100.0;
    config.max_position_jump = 1.0;
    for (std::size_t i = 0; i < qm::kRlHistoryFrames; ++i)
    {
        config.history_frames[i] = i;
    }
    const auto model = make_rl_model();
    for (std::size_t i = 0; i < qm::kRlJointCount; ++i)
    {
        config.joint_names[i] = model.joints[i].name;
        config.policy_dof_indices[i] = i;
        config.default_joint_positions[i] = default_positions[i];
        config.kp[i] = 40.0;
        config.kd[i] = 1.2;
        config.action_scale[i] = 0.25;
    }
    return config;
}

qm::MotionRuntime::CreateResult make_rl_runtime()
{
    auto created = qm::MotionRuntime::create(make_rl_model(), motion_test::make_test_config());
    expect(created.ok(), "black RL 测试运行时应创建成功：" + created.error_message);
    return created;
}

qm::RlConfig make_wheel_rl_config(const qc::RobotModel& model)
{
    qm::RlConfig config;
    config.name = "flat";
    config.robot_name = model.name;
    config.model_path = "/tmp/blackw_flat.pt";
    config.observation_dimension = 57;
    config.history_frame_count = 6;
    config.inference_input_dimension = 342;
    config.action_dimension = model.joint_count;
    config.joint_count = model.joint_count;
    config.wheel_count = 4;
    config.wheel_indices = {3, 7, 11, 15};
    config.command_scale = {2.0, 2.0, 0.25};
    config.command_limits = {2.0, 1.0, 3.0};
    config.angular_velocity_scale = 0.25;
    config.joint_position_scale = 1.0;
    config.joint_velocity_scale = 0.05;
    config.observation_clip = 100.0;
    config.action_clip = 100.0;
    config.max_position_jump = 1.0;
    for (std::size_t i = 0; i < config.history_frame_count; ++i)
    {
        config.history_frames[i] = i;
    }
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        const bool is_wheel = model.joints[i].role == qc::JointRole::Wheel;
        config.joint_names[i] = model.joints[i].name;
        config.policy_dof_indices[i] = i;
        config.default_joint_positions[i] = is_wheel ? 0.0 : 0.4;
        config.kp[i] = is_wheel ? 0.0 : 50.0;
        config.kd[i] = is_wheel ? 1.0 : 1.2;
        config.action_scale[i] = is_wheel
            ? ((i == 7 || i == 15) ? -10.0 : 10.0)
            : 0.25;
    }
    return config;
}

qm::FixedDriveConfig make_fixed_drive_config(
    const qc::RobotModel& model,
    const std::string& behavior_name,
    const double target)
{
    qm::FixedDriveConfig config;
    config.behavior_name = behavior_name;
    config.robot_name = model.name;
    config.joint_count = model.joint_count;
    config.prepare_cycles = 2;
    config.exit_to_rl_cycles = 2;
    config.max_x = 2.0;
    config.max_yaw = 3.0;
    config.wheel_velocity_scale = 10.0;
    config.yaw_to_wheel_velocity = 5.0;
    config.wheel_count = 4;
    config.wheel_velocity_sign = {1.0, -1.0, 1.0, -1.0};
    config.wheel_sides = {
        qm::WheelSide::Left,
        qm::WheelSide::Right,
        qm::WheelSide::Left,
        qm::WheelSide::Right,
    };
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        const bool is_wheel = model.joints[i].role == qc::JointRole::Wheel;
        config.joint_names[i] = model.joints[i].name;
        config.target_positions[i] = is_wheel ? 0.0 : target;
        config.kp[i] = is_wheel ? 0.0 : 80.0;
        config.kd[i] = is_wheel ? 2.0 : 3.0;
    }
    return config;
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

    io.read_code = qc::RobotIOCode::Ok;
    const auto recovered_state = motion_test::update(*created.runtime, io);
    expect(!recovered_state.status.error_message.empty(),
        "进入 Passive 的故障原因必须锁存，不能被下一周期安全命令清除");
}

// 测试专用 RobotIO 注入统一验证故障终态、安全命令和显式恢复条件。
void test_robot_io_fault_injection_matrix()
{
    struct FaultCase
    {
        motion_test::RobotIOFaultInjection injection;
        qc::RobotIOCode read_code;
        qc::RobotIOState io_state;
        const char* name;
    };
    const std::array<FaultCase, 7> cases{{
        {motion_test::RobotIOFaultInjection::NoData,
            qc::RobotIOCode::NoData, qc::RobotIOState::Paused, "状态无数据"},
        {motion_test::RobotIOFaultInjection::Disconnected,
            qc::RobotIOCode::Disconnected, qc::RobotIOState::Disconnected, "IPC 断开"},
        {motion_test::RobotIOFaultInjection::BackendFault,
            qc::RobotIOCode::Fault, qc::RobotIOState::Fault, "backend fault"},
        {motion_test::RobotIOFaultInjection::FutureTimestamp,
            qc::RobotIOCode::Ok, qc::RobotIOState::Ready, "时间异常"},
        {motion_test::RobotIOFaultInjection::NonFiniteJoint,
            qc::RobotIOCode::Ok, qc::RobotIOState::Ready, "关节 NaN"},
        {motion_test::RobotIOFaultInjection::InfiniteJoint,
            qc::RobotIOCode::Ok, qc::RobotIOState::Ready, "关节 Inf"},
        {motion_test::RobotIOFaultInjection::ExpiredCommand,
            qc::RobotIOCode::Ok, qc::RobotIOState::Ready, "命令过期"},
    }};

    for (const auto& test : cases)
    {
        auto created = motion_test::make_runtime();
        if (!created.ok())
        {
            continue;
        }
        motion_test::FakeRobotIO io;
        io.state = motion_test::make_state(
            motion_test::make_test_model(), motion_test::make_rest_positions());
        motion_test::update(*created.runtime, io);
        const auto getup = motion_test::make_request(1, qc::ModeRequestType::GetUp);
        motion_test::update(*created.runtime, io, &getup);
        const std::size_t command_count = io.submitted.size();

        io.inject(test.injection);
        const auto failed = motion_test::update(*created.runtime, io);
        expect(failed.read_code == test.read_code,
            std::string(test.name) + " 应报告预期 read_code");
        expect(failed.status.mode == qc::MotionMode::Passive,
            std::string(test.name) + " 应使主动动作回到 Passive");
        expect(failed.result_event_count == 1 &&
                failed.result_events[0].request_id == getup.request_id &&
                failed.result_events[0].state == qc::ModeResultState::Failed,
            std::string(test.name) + " 应交付活动请求 Failed");
        expect(io.status().state == test.io_state,
            std::string(test.name) + " 应保留明确 RobotIO 状态");
        expect(io.submitted.size() == command_count,
            std::string(test.name) + " 失败后不得留下新的主动命令");

        const auto blocked = motion_test::make_request(2, qc::ModeRequestType::GetUp);
        const auto rejected = motion_test::update(*created.runtime, io, &blocked);
        expect(rejected.result.state == qc::ModeResultState::Rejected ||
                created.runtime->query_result(blocked.request_id).state ==
                    qc::ModeResultState::Failed,
            std::string(test.name) + " 未清除时不得恢复主动控制");

        io.clear_injection();
        const auto recovery = motion_test::make_request(3, qc::ModeRequestType::GetUp);
        const auto recovered = motion_test::update(*created.runtime, io, &recovery);
        expect(recovered.result.state == qc::ModeResultState::Accepted,
            std::string(test.name) + " 清除后应允许显式新请求恢复");
    }
}

// IMU 无效、策略 forward 失败和推理超时只在依赖 IMU/策略的 RL 模式触发安全回退。
void test_rl_fault_injection_and_diagnostics()
{
    const auto run_case = [](const motion_test::RobotIOFaultInjection io_fault,
                              const bool fail_forward,
                              const qc::Nanoseconds elapsed_ns,
                              const char* name) {
        auto created = make_rl_runtime();
        if (!created.ok())
        {
            return;
        }
        const auto model = make_rl_model();
        const auto stand_pose = config_stand_pose();
        FakePolicy policy;
        policy.fail_forward = fail_forward;
        policy.elapsed_ns = elapsed_ns;
        std::string error;
        created.runtime->attach_policy(make_rl_config("flat", stand_pose), policy, error);
        motion_test::FakeRobotIO io;
        io.state = motion_test::make_state(model, motion_test::make_rest_positions());
        motion_test::drive_getup(*created.runtime, io, 1);
        io.state = motion_test::make_state(model, stand_pose);
        io.inject(io_fault);
        const auto command = make_base_command();
        auto start = motion_test::make_request(2, qc::ModeRequestType::StartBehavior);
        start.behavior_name = "rl_locomotion";
        const auto failed = update_with_command(*created.runtime, io, command, &start);
        expect(failed.status.mode == qc::MotionMode::Passive,
            std::string(name) + " 应使 RL 回到 Passive");
        expect(created.runtime->query_result(start.request_id).state ==
                qc::ModeResultState::Failed,
            std::string(name) + " 应使策略启动请求 Failed");
        expect(!io.submitted.empty() &&
                io.submitted.back().joints[0].mode == qc::ControlMode::Disabled,
            std::string(name) + " 应提交 Disabled 最终命令");
        expect(failed.diagnostics.inference_elapsed_ns == elapsed_ns,
            std::string(name) + " 应记录最近推理耗时");
    };

    run_case(motion_test::RobotIOFaultInjection::InvalidImu, false, 0, "IMU 非法");
    run_case(motion_test::RobotIOFaultInjection::None, true, 2'000'000,
        "policy forward 失败");
    run_case(motion_test::RobotIOFaultInjection::None, false,
        qm::kRlInferenceDeadlineNs + 1, "policy inference 超时");
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

// 与 rl_sar 对齐：RL 中的 GetUp 应停止策略，并从当前姿态平滑回到 Stand。
void test_rl_getup_returns_to_stand()
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

    const auto command = make_base_command();
    auto start = motion_test::make_request(2, qc::ModeRequestType::StartBehavior);
    start.behavior_name = "rl_locomotion";
    const auto running = update_with_command(*created.runtime, io, command, &start);
    expect(running.status.mode == qc::MotionMode::Running &&
            running.status.behavior_name == "rl_locomotion",
        "RL 应先进入 Running");

    auto near_stand = stand_pose;
    near_stand[1] += 0.04;
    io.state = motion_test::make_state(model, near_stand);
    const auto getup = motion_test::make_request(3, qc::ModeRequestType::GetUp);
    const auto accepted = update_with_command(*created.runtime, io, command, &getup);
    expect(accepted.result.state == qc::ModeResultState::Accepted &&
            accepted.status.mode == qc::MotionMode::GetUp,
        "RL 中的 GetUp 应被接受");
    expect(accepted.status.behavior_name.empty(),
        "离开 RL 后应清空行为名称");
    expect_command_positions(io, lerp_positions(near_stand, stand_pose, 0.5),
        "接近默认姿态时应跳过预起立阶段并平滑返回站姿");

    motion_test::update(*created.runtime, io);
    const auto standing = motion_test::update(*created.runtime, io);
    expect(standing.status.mode == qc::MotionMode::Stand,
        "RL 的 GetUp 完成后应进入 Stand");
    expect(created.runtime->query_result(getup.request_id).state ==
            qc::ModeResultState::Completed,
        "RL 返回 Stand 后 GetUp 请求应完成");
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
    const auto started = update_with_command(*created.runtime, io, command, &start);
    expect(flat_policy.forward_count == 1, "启动 RL 时 flat 应完成首次推理");
    expect(started.status.command_limits == std::array<double, 3>{2.0, 2.0, 2.0},
        "MotionStatus 应公开当前策略的三轴 command_limits");

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

// black 与 blackW 共用同一 Retry 生命周期；轮关节使用当前角度、零速和零 KP。
void test_retry_and_mixed_wheel_commands()
{
    const auto leg_model = motion_test::make_test_model();
    auto leg_runtime = motion_test::make_runtime();
    if (leg_runtime.ok())
    {
        std::string leg_error;
        expect(leg_runtime.runtime->configure_retry(
                   motion_test::make_retry_config(leg_model), leg_error),
            "black Retry 配置应接受：" + leg_error);
        motion_test::FakeRobotIO leg_io;
        leg_io.state = motion_test::make_state(
            leg_model, motion_test::make_rest_positions());
        auto leg_retry = motion_test::make_request(1, qc::ModeRequestType::StartBehavior);
        leg_retry.behavior_name = "retry";
        motion_test::update(*leg_runtime.runtime, leg_io, &leg_retry);
        const auto leg_locked = motion_test::update(*leg_runtime.runtime, leg_io);
        expect(leg_locked.status.behavior_name == "retry" &&
                leg_locked.status.behavior_phase == "locked",
            "black 应使用共用 Retry 生命周期进入 locked");
    }

    const auto model = motion_test::make_wheel_test_model();
    auto created = qm::MotionRuntime::create(model, motion_test::make_wheel_test_config());
    expect(created.ok(), "16 关节腿轮运行时应创建成功：" + created.error_message);
    if (!created.ok())
    {
        return;
    }
    std::string error;
    expect(created.runtime->configure_retry(
               motion_test::make_retry_config(model), error),
        "blackW Retry 配置应接受：" + error);

    auto positions = motion_test::make_rest_positions();
    positions[0] = 10.0;
    positions[3] = 1.25;
    positions[7] = -2.5;
    positions[11] = 3.75;
    positions[15] = -5.0;
    motion_test::FakeRobotIO io;
    io.state = motion_test::make_state(model, positions);

    const auto getup = motion_test::make_request(1, qc::ModeRequestType::GetUp);
    const auto first = motion_test::update(*created.runtime, io, &getup);
    expect(first.result.state == qc::ModeResultState::Accepted,
        "blackW GetUp 应被接受");
    expect_close({io.submitted.back().joints[0].target_position, 3.0, 1e-12,
        "越界腿姿态生成的命令应夹到显式位置上限"});
    for (const std::size_t index : {3U, 7U, 11U, 15U})
    {
        const auto& wheel = io.submitted.back().joints[index];
        expect_close({wheel.target_position, positions[index], 1e-12,
            "基础动作轮关节应保持当前角度"});
        expect(wheel.target_velocity == 0.0 && wheel.kp == 0.0 && wheel.kd == 0.5,
            "基础动作轮关节应使用零速、零 KP 和配置 KD");
    }

    auto retry = motion_test::make_request(2, qc::ModeRequestType::StartBehavior);
    retry.behavior_name = "retry";
    const auto accepted = motion_test::update(*created.runtime, io, &retry);
    expect(accepted.result.state == qc::ModeResultState::Accepted,
        "Retry 应能中断当前基础动作");
    expect(accepted.status.behavior_name == "retry" &&
            accepted.status.behavior_phase == "preparing",
        "Retry 应进入 preparing 阶段");
    const auto locked = motion_test::update(*created.runtime, io);
    expect(locked.status.mode == qc::MotionMode::Running &&
            locked.status.behavior_phase == "locked",
        "Retry 插值完成后应保持 Running/locked");
    expect(created.runtime->query_result(retry.request_id).state ==
            qc::ModeResultState::Completed,
        "Retry 锁定后请求应 Completed");
    for (const std::size_t index : {3U, 7U, 11U, 15U})
    {
        const auto& wheel = io.submitted.back().joints[index];
        expect_close({wheel.target_position, positions[index], 1e-12,
            "Retry 轮关节应保持当前角度"});
        expect(wheel.target_velocity == 0.0 && wheel.kp == 0.0 && wheel.kd == 0.7,
            "Retry 轮关节应使用行为配置 KD");
    }

    const auto switch_policy =
        motion_test::make_request(3, qc::ModeRequestType::SwitchPolicy);
    const auto rejected = motion_test::update(*created.runtime, io, &switch_policy);
    expect(rejected.result.state == qc::ModeResultState::Rejected,
        "Retry 锁定期间应拒绝策略切换");

    const auto retry_getup = motion_test::make_request(4, qc::ModeRequestType::GetUp);
    const auto interrupted = motion_test::update(*created.runtime, io, &retry_getup);
    expect(interrupted.result.state == qc::ModeResultState::Accepted &&
            interrupted.status.mode == qc::MotionMode::GetUp,
        "Retry 应允许 GetUp 打断");

    const auto passive = motion_test::make_request(5, qc::ModeRequestType::EnterPassive);
    const auto stopped = motion_test::update(*created.runtime, io, &passive);
    expect(stopped.result.state == qc::ModeResultState::Completed &&
            stopped.status.mode == qc::MotionMode::Passive,
        "Retry 后的动作应可由 EnterPassive 立即停止");
}

// Car、Bridge、Low-bar 共用固定姿态差速控制，并可在模式间切换和返回 RL。
void test_fixed_drive_behaviors()
{
    const auto model = motion_test::make_wheel_test_model();
    auto created = qm::MotionRuntime::create(
        model, motion_test::make_wheel_test_config());
    expect(created.ok(), "固定姿态轮驱测试运行时应创建成功");
    if (!created.ok())
    {
        return;
    }
    std::string error;
    auto car_config = make_fixed_drive_config(model, "car_drive", 0.6);
    car_config.wheel_sides = {
        qm::WheelSide::Right,
        qm::WheelSide::Left,
        qm::WheelSide::Right,
        qm::WheelSide::Left,
    };
    expect(created.runtime->configure_fixed_drive(
        car_config, error),
        "Car drive 配置应接受：" + error);
    expect(created.runtime->configure_fixed_drive(
        make_fixed_drive_config(model, "bridge_drive", 0.5), error),
        "Bridge drive 配置应接受：" + error);
    expect(created.runtime->configure_fixed_drive(
        make_fixed_drive_config(model, "low_bar_drive", 0.7), error),
        "Low-bar drive 配置应接受：" + error);
    auto duplicate = make_fixed_drive_config(model, "car_drive", 0.6);
    expect(!created.runtime->configure_fixed_drive(duplicate, error),
        "同名固定姿态轮驱配置必须拒绝重复注册");

    FakePolicy policy;
    policy.output_dimension = 16;
    expect(created.runtime->attach_policy(
        make_wheel_rl_config(model), policy, error),
        "固定姿态轮驱返回测试策略应接入：" + error);
    motion_test::FakeRobotIO io;
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    motion_test::drive_getup(*created.runtime, io, 1);

    auto car = motion_test::make_request(2, qc::ModeRequestType::StartBehavior);
    car.behavior_name = "car_drive";
    const auto no_command = motion_test::update(*created.runtime, io, &car);
    expect(no_command.result.state == qc::ModeResultState::Rejected,
        "固定姿态轮驱启动必须要求有效 BaseCommand");
    car.request_id = 3;
    auto command = make_base_command();
    command.vx = 0.5;
    command.wz = 0.2;
    const auto accepted = update_with_command(*created.runtime, io, command, &car);
    expect(accepted.result.state == qc::ModeResultState::Accepted &&
            accepted.status.behavior_name == "car_drive" &&
            accepted.status.behavior_phase == "preparing",
        "Car drive 应从 Stand 进入姿态准备阶段");
    expect_close({io.submitted.back().joints[0].target_position, 0.3, 1e-12,
        "Car drive 第一周期应线性插值腿姿态"});
    expect_close({io.submitted.back().joints[3].target_velocity, 6.0, 1e-12,
        "轮侧映射必须来自显式配置而不是关节下标"});
    expect_close({io.submitted.back().joints[7].target_velocity, -4.0, 1e-12,
        "轮速应同时应用显式轮侧和独立方向符号"});
    const auto ready = update_with_command(*created.runtime, io, command);
    expect(ready.status.behavior_phase == "driving" &&
            created.runtime->query_result(3).state == qc::ModeResultState::Completed,
        "固定姿态准备完成后应完成启动请求并持续 Driving");

    auto stale_command = command;
    stale_command.expires_at_ns = -1;
    update_with_command(*created.runtime, io, stale_command);
    expect(io.submitted.back().joints[3].target_velocity == 0.0 &&
            io.submitted.back().joints[7].target_velocity == 0.0,
        "运行中 BaseCommand 过期必须将固定轮驱速度归零");

    auto bridge = motion_test::make_request(4, qc::ModeRequestType::StartBehavior);
    bridge.behavior_name = "bridge_drive";
    const auto switched = update_with_command(*created.runtime, io, command, &bridge);
    expect(switched.result.state == qc::ModeResultState::Accepted &&
            switched.status.behavior_name == "bridge_drive",
        "固定姿态轮驱之间应允许直接切换");
    update_with_command(*created.runtime, io, command);

    auto return_rl = motion_test::make_request(5, qc::ModeRequestType::StartBehavior);
    return_rl.behavior_name = "rl_locomotion";
    const auto transitioning = update_with_command(
        *created.runtime, io, command, &return_rl);
    expect(transitioning.result.state == qc::ModeResultState::Accepted &&
            transitioning.status.behavior_phase == "policy_transition" &&
            policy.forward_count == 0,
        "Bridge drive 应以配置周期和固定增益进入返回 RL 过渡");
    update_with_command(*created.runtime, io, command);
    expect(created.runtime->query_result(5).state == qc::ModeResultState::Completed,
        "固定姿态轮驱返回 RL 过渡完成后请求应 Completed");
    update_with_command(*created.runtime, io, command);
    expect(policy.forward_count == 1,
        "返回固定姿态轮驱过渡后的下一周期应恢复 RL 推理");
}

// Event chain 对照 rl_sar 执行 pose、drive、pose_drive，并以编码器位移完成轮驱动。
void test_event_chain_capability_and_request_interface()
{
    const auto model = motion_test::make_test_model();
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    qm::EventChainConfig config;
    config.robot_name = model.name;
    config.joint_count = model.joint_count;
    config.exit_to_rl_cycles = 1;
    config.interpolation = "linear";
    config.event_count = 1;
    config.events[0].name = "drive";
    config.events[0].type = qm::EventType::Drive;
    config.events[0].wheel_group = qm::WheelGroup::All;
    config.events[0].distance_m = 0.1;
    config.events[0].speed_mps = 0.1;
    config.events[0].timeout_cycles = 10;
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        config.joint_names[i] = model.joints[i].name;
        config.kp[i] = 10.0;
        config.kd[i] = 1.0;
    }
    std::string error;
    expect(!created.runtime->configure_event_chain(config, error),
        "无 Wheel 角色的 black 模型必须拒绝 drive 事件");
    expect(error.find("Wheel") != std::string::npos ||
            error.find("wheel") != std::string::npos,
        "能力拒绝应明确说明 Wheel 原因");

    motion_test::FakeRobotIO io;
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    auto request = motion_test::make_request(1, qc::ModeRequestType::StartBehavior);
    request.behavior_name = "event_chain";
    const auto result = motion_test::update(*created.runtime, io, &request);
    expect(result.result.state == qc::ModeResultState::Rejected,
        "未配置 Event chain 的请求应明确拒绝");
    expect(result.status.mode == qc::MotionMode::Passive,
        "Event chain 拒绝不得改变当前模式");

    const auto wheel_model = motion_test::make_wheel_test_model();
    auto wheel_runtime = qm::MotionRuntime::create(
        wheel_model, motion_test::make_wheel_test_config());
    expect(wheel_runtime.ok(), "Event chain 腿轮运行时应创建成功");
    if (!wheel_runtime.ok())
    {
        return;
    }
    FakePolicy wheel_policy;
    wheel_policy.output_dimension = 16;
    expect(wheel_runtime.runtime->attach_policy(
        make_wheel_rl_config(wheel_model), wheel_policy, error),
        "blackW 测试策略应接入：" + error);
    config.robot_name = wheel_model.name;
    config.joint_count = wheel_model.joint_count;
    config.wheel_count = 4;
    config.wheel_radius = 0.1;
    for (std::size_t i = 0; i < config.wheel_count; ++i)
    {
        config.wheel_velocity_sign[i] = 1.0;
    }
    for (std::size_t i = 0; i < wheel_model.joint_count; ++i)
    {
        config.joint_names[i] = wheel_model.joints[i].name;
        if (wheel_model.joints[i].role == qc::JointRole::Wheel)
        {
            config.kp[i] = 0.0;
        }
    }
    expect(wheel_runtime.runtime->configure_event_chain(config, error),
        "显式 Wheel 角色模型应通过 drive 能力检查：" + error);
    motion_test::FakeRobotIO wheel_io;
    wheel_io.state = motion_test::make_state(
        wheel_model, motion_test::make_rest_positions());
    motion_test::drive_getup(*wheel_runtime.runtime, wheel_io, 1);

    config.interpolation = "smoothstep";
    config.exit_to_rl_cycles = 2;
    config.event_count = 3;
    config.events[0] = {};
    config.events[0].name = "pose";
    config.events[0].type = qm::EventType::Pose;
    config.events[0].transition_cycles = 2;
    config.events[1] = {};
    config.events[1].name = "front_drive";
    config.events[1].type = qm::EventType::Drive;
    config.events[1].wheel_group = qm::WheelGroup::Front;
    config.events[1].distance_m = 0.02;
    config.events[1].speed_mps = 0.1;
    config.events[1].timeout_cycles = 4;
    config.events[2] = {};
    config.events[2].name = "rear_pose_drive";
    config.events[2].type = qm::EventType::PoseDrive;
    config.events[2].transition_cycles = 2;
    config.events[2].wheel_group = qm::WheelGroup::Rear;
    config.events[2].distance_m = -0.02;
    config.events[2].speed_mps = 0.1;
    config.events[2].timeout_cycles = 4;
    for (std::size_t i = 0; i < wheel_model.joint_count; ++i)
    {
        if (wheel_model.joints[i].role != qc::JointRole::Wheel)
        {
            config.events[0].dof_positions[i] = 0.6;
            config.events[2].dof_positions[i] = 0.8;
        }
    }
    config.wheel_velocity_sign = {1.0, -1.0, 1.0, -1.0};
    expect(wheel_runtime.runtime->configure_event_chain(config, error),
        "组合 Event chain 应通过配置检查：" + error);

    request.request_id = 2;
    const auto accepted = motion_test::update(*wheel_runtime.runtime, wheel_io, &request);
    expect(accepted.result.state == qc::ModeResultState::Accepted &&
            accepted.status.behavior_name == "event_chain",
        "Stand 应接受 Event chain 请求");
    expect_close({wheel_io.submitted.back().joints[0].target_position, 0.3, 1e-12,
        "smoothstep 中点应按 rl_sar 插值"});
    motion_test::update(*wheel_runtime.runtime, wheel_io);
    motion_test::update(*wheel_runtime.runtime, wheel_io);
    expect_close({wheel_io.submitted.back().joints[3].target_velocity, 1.0, 1e-12,
        "前左轮应按正向符号驱动"});
    expect_close({wheel_io.submitted.back().joints[7].target_velocity, -1.0, 1e-12,
        "前右轮应按负向符号驱动"});
    expect(wheel_io.submitted.back().joints[11].target_velocity == 0.0,
        "front 轮组不得驱动后轮");
    wheel_io.state.joints[3].position += 0.2;
    wheel_io.state.joints[7].position -= 0.2;
    motion_test::update(*wheel_runtime.runtime, wheel_io);
    motion_test::update(*wheel_runtime.runtime, wheel_io);
    expect_close({wheel_io.submitted.back().joints[11].target_velocity, -1.0, 1e-12,
        "负距离 pose_drive 应反向驱动后左轮"});
    expect_close({wheel_io.submitted.back().joints[15].target_velocity, 1.0, 1e-12,
        "负距离 pose_drive 应反向驱动后右轮"});
    wheel_io.state.joints[11].position -= 0.2;
    wheel_io.state.joints[15].position += 0.2;
    const auto completed = motion_test::update(*wheel_runtime.runtime, wheel_io);
    expect(completed.status.behavior_phase == "completed" &&
            wheel_runtime.runtime->query_result(2).state ==
                qc::ModeResultState::Completed,
        "姿态和位移均完成后 Event chain 应进入最终保持并完成请求");

    auto return_rl = motion_test::make_request(3, qc::ModeRequestType::StartBehavior);
    return_rl.behavior_name = "rl_locomotion";
    const auto base_command = make_base_command();
    const auto returning = update_with_command(
        *wheel_runtime.runtime, wheel_io, base_command, &return_rl);
    expect(returning.result.state == qc::ModeResultState::Accepted &&
            returning.status.behavior_phase == "policy_transition",
        "Event chain 应通过显式过渡返回当前 RL 策略");
    expect(wheel_policy.forward_count == 0,
        "返回 RL 的姿态过渡周期不得提前执行策略推理");
    const auto returned = update_with_command(
        *wheel_runtime.runtime, wheel_io, base_command);
    expect(returned.status.policy_name == "flat" &&
            wheel_runtime.runtime->query_result(3).state ==
                qc::ModeResultState::Completed,
        "exit_to_rl_cycles 完成后应激活原策略并完成返回请求");
    update_with_command(*wheel_runtime.runtime, wheel_io, base_command);
    expect(wheel_policy.forward_count == 1,
        "返回过渡后的下一周期应恢复 RL 推理");

    auto timeout_runtime = qm::MotionRuntime::create(
        wheel_model, motion_test::make_wheel_test_config());
    expect(timeout_runtime.ok(), "Event chain 超时测试运行时应创建成功");
    if (!timeout_runtime.ok())
    {
        return;
    }
    config.event_count = 1;
    config.events[0] = config.events[1];
    config.events[0].timeout_cycles = 1;
    expect(timeout_runtime.runtime->configure_event_chain(config, error),
        "Event chain 超时配置应通过：" + error);
    motion_test::FakeRobotIO timeout_io;
    timeout_io.state = motion_test::make_state(
        wheel_model, motion_test::make_rest_positions());
    motion_test::drive_getup(*timeout_runtime.runtime, timeout_io, 1);
    request.request_id = 2;
    motion_test::update(*timeout_runtime.runtime, timeout_io, &request);
    const auto timed_out = motion_test::update(*timeout_runtime.runtime, timeout_io);
    expect(timed_out.status.mode == qc::MotionMode::Passive &&
            timeout_runtime.runtime->query_result(2).state ==
                qc::ModeResultState::Failed,
        "轮编码器无进展达到 timeout 后应失败并回到 Passive");
    expect(timeout_io.submitted.back().joints[0].mode == qc::ControlMode::Disabled,
        "Event chain 超时周期应提交显式 Disabled");
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
    test_rl_getup_returns_to_stand();
    test_request_interruptions_and_explicit_rejections();
    test_fault_fails_active_request();
    test_robot_io_fault_injection_matrix();
    test_rl_fault_injection_and_diagnostics();
    test_direct_policy_switches();
    test_policy_transition_and_failure();
    test_policy_transition_interrupted_by_passive();
    test_retry_and_mixed_wheel_commands();
    test_fixed_drive_behaviors();
    test_event_chain_capability_and_request_interface();

    if (motion_test::failures > 0)
    {
        std::cerr << motion_test::failures << " 个运动测试失败\n";
        return 1;
    }
    std::cout << "全部运动测试通过\n";
    return 0;
}
