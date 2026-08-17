/**
 * @file motion_session_tests.cpp
 * @brief 测试 MotionRuntime 的会话建立/切换、无状态周期和配置/时间边界路径。
 */

#include "motion_test_helpers.hpp"

#include <limits>

namespace
{

using motion_test::expect;

// 没有任何可用状态时：不提交命令，GetUp 请求被拒绝，模式保持 Passive。
void test_no_state()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    io.read_code = qc::RobotIOCode::NoData;

    const auto request = motion_test::make_request(1, qc::ModeRequestType::GetUp);
    const auto output = motion_test::update(*created.runtime, io, &request);
    expect(output.read_code == qc::RobotIOCode::NoData, "无状态时应报告 NoData");
    expect(!output.submitted, "无状态时不应提交命令");
    expect(output.result.state == qc::ModeResultState::Rejected, "无状态时 GetUp 应拒绝");
    expect(output.status.mode == qc::MotionMode::Passive, "无状态时应保持 Passive");
}

// 会话变化：立即回到 Passive，清除 rest_pose 和未完成请求，命令序号从 1 重新开始。
void test_session_change_resets()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    motion_test::drive_getup(*created.runtime, io, 1);

    // 执行侧建立新会话（例如 reset 后）。
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    io.state.header.session_id = 2;
    const auto output = motion_test::update(*created.runtime, io);
    expect(output.status.mode == qc::MotionMode::Passive, "会话变化后应立即回到 Passive");
    expect(io.submitted.back().header.session_id == 2, "新会话命令应使用新 session_id");
    expect(io.submitted.back().header.sequence == 1, "新会话命令序号应从 1 重新开始");
    expect(io.submitted.back().joints[0].mode == qc::ControlMode::Disabled,
        "新会话首条命令应为 Disabled");

    // 上一会话的 GetUp 使用了编号 1；新会话中相同编号应作为新请求处理，
    // 同时因 rest_pose 已随会话清除而被拒绝。
    const auto getdown = motion_test::make_request(1, qc::ModeRequestType::GetDown);
    const auto getdown_result = motion_test::update(*created.runtime, io, &getdown);
    expect(getdown_result.result.state == qc::ModeResultState::Rejected,
        "新会话已清除 rest_pose，GetDown 应拒绝");

    const auto getup = motion_test::make_request(2, qc::ModeRequestType::GetUp);
    const auto getup_result = motion_test::update(*created.runtime, io, &getup);
    expect(getup_result.result.state == qc::ModeResultState::Accepted,
        "新会话中 GetUp 应可重新接受");
}

// 会话切换与活动请求重试发生在同一周期时：只交付旧请求 Failed 终态，
// 拒绝输入请求且不自动重启到新会话；调用方在后续周期重新提交。
void test_session_change_defers_request()
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
    motion_test::update(*created.runtime, io, &getup);

    // 会话 1 中 GetUp #1 正在 Running；执行侧切换会话 2，本周期仍重试 #1。
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    io.state.header.session_id = 2;
    const auto output = motion_test::update(*created.runtime, io, &getup);
    expect(output.result.state == qc::ModeResultState::Rejected,
        "会话切换周期应拒绝输入请求");
    expect(output.result.message.rfind("session changed", 0) == 0,
        "拒绝原因应说明会话已切换");
    expect(output.result_event_count == 1 && output.result_events[0].request_id == 1 &&
        output.result_events[0].state == qc::ModeResultState::Failed,
        "旧活动请求应交付 Failed 终态");
    expect(output.status.mode == qc::MotionMode::Passive,
        "会话切换后应保持 Passive，不自动重新 GetUp");
    expect(io.submitted.back().motion_mode == qc::MotionMode::Passive,
        "会话切换周期提交的命令应为 Passive");

    // 后续周期重新提交同一编号：按新会话的新请求处理。
    const auto resubmitted = motion_test::update(*created.runtime, io, &getup);
    expect(resubmitted.result.state == qc::ModeResultState::Accepted,
        "后续周期重新提交应作为新会话请求接受");
}

// 非法的模型或控制器配置必须拒绝创建 MotionRuntime。
void test_invalid_config_rejected()
{
    auto bad_cycles = qm::MotionRuntime::create(
        motion_test::make_test_model(), qc::ControllerConfig{});
    expect(!bad_cycles.ok(), "零周期配置应拒绝创建");

    auto bad_model = qm::MotionRuntime::create(
        qc::RobotModel{}, motion_test::make_test_config());
    expect(!bad_model.ok(), "空模型应拒绝创建");

    auto config = motion_test::make_test_config();
    config.fixed_kp[2] = 200.0;
    auto bad_gain = qm::MotionRuntime::create(motion_test::make_test_model(), config);
    expect(!bad_gain.ok(), "越过模型上限的增益应拒绝创建");
}

// 单调时间加命令有效期溢出时，过期时间应钳位到可表示最大值，不触发未定义行为。
void test_expiry_overflow_clamped()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    const qc::Nanoseconds near_max = INT64_MAX - 5'000'000;
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    io.state.header.timestamp_ns = near_max;

    qm::MotionUpdateInput input;
    input.now_ns = near_max;
    const auto output = created.runtime->update(io, input);
    expect(output.submitted, "接近上限的时间应正常提交命令");
    expect(io.submitted.back().expires_at_ns == INT64_MAX,
        "加法溢出时过期时间应钳位到 INT64_MAX");
}

}  // namespace

int main()
{
    test_no_state();
    test_session_change_resets();
    test_session_change_defers_request();
    test_invalid_config_rejected();
    test_expiry_overflow_clamped();

    if (motion_test::failures > 0)
    {
        std::cerr << motion_test::failures << " 个运动会话测试失败\n";
        return 1;
    }
    std::cout << "全部运动会话测试通过\n";
    return 0;
}
