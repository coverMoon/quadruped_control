/**
 * @file motion_error_tests.cpp
 * @brief 测试 MotionRuntime 的会话变化、失效状态、安全状态下降和 RobotIO 错误路径。
 */

#include "motion_test_helpers.hpp"

#include <limits>

namespace
{

using motion_test::expect;



// 主动动作中关节掉线：请求转为 Failed，回到 Passive 并提交 Disabled。
void test_joint_offline_fails_active()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());

    const auto request = motion_test::make_request(1, qc::ModeRequestType::GetUp);
    motion_test::update(*created.runtime, io, &request);

    io.state.joints[3].online = false;
    const auto failed_cycle = motion_test::update(*created.runtime, io);
    expect(failed_cycle.status.mode == qc::MotionMode::Passive,
        "关节掉线后应回到 Passive");
    expect(io.submitted.back().joints[0].mode == qc::ControlMode::Disabled,
        "失败周期应提交 Disabled");
    expect(!failed_cycle.status.error_message.empty(), "失败后应记录错误说明");
    expect(failed_cycle.result_event_count == 1 &&
        failed_cycle.result_events[0].request_id == 1 &&
        failed_cycle.result_events[0].state == qc::ModeResultState::Failed,
        "失败终态应通过事件交付");

    const auto retried = motion_test::update(*created.runtime, io, &request);
    expect(retried.result.state == qc::ModeResultState::Failed,
        "关节掉线后请求应转为 Failed");
}

// 主动动作中执行侧安全状态下降：请求转为 Failed 并回到 Passive。
void test_safety_drop_fails_active()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());

    const auto request = motion_test::make_request(1, qc::ModeRequestType::GetUp);
    motion_test::update(*created.runtime, io, &request);

    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    io.state.safety_state = qc::SafetyState::Damping;
    motion_test::update(*created.runtime, io);
    const auto retried = motion_test::update(*created.runtime, io, &request);
    expect(retried.result.state == qc::ModeResultState::Failed,
        "安全状态下降后请求应转为 Failed");
    expect(retried.status.mode == qc::MotionMode::Passive, "安全状态下降后应回到 Passive");
}

// 主动动作中状态包含非有限值：核心校验失败，请求转为 Failed 且不再提交新命令。
void test_invalid_state_fails_active()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());

    const auto request = motion_test::make_request(1, qc::ModeRequestType::GetUp);
    motion_test::update(*created.runtime, io, &request);

    const std::size_t commands_before = io.submitted.size();
    io.state.joints[0].position = std::numeric_limits<double>::quiet_NaN();
    const auto failed_cycle = motion_test::update(*created.runtime, io);
    expect(failed_cycle.status.mode == qc::MotionMode::Passive,
        "非法状态后应回到 Passive");
    expect(io.submitted.size() == commands_before, "非法状态周期不应提交新命令");

    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    const auto retried = motion_test::update(*created.runtime, io, &request);
    expect(retried.result.state == qc::ModeResultState::Failed,
        "非法状态后请求应转为 Failed");
}

// 主动动作中 RobotIO 读取失败：请求转为 Failed，依赖旧命令过期进入安全退路。
void test_read_failure_fails_active()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());

    const auto request = motion_test::make_request(1, qc::ModeRequestType::GetUp);
    motion_test::update(*created.runtime, io, &request);

    const std::size_t commands_before = io.submitted.size();
    io.read_code = qc::RobotIOCode::Disconnected;
    const auto failed_cycle = motion_test::update(*created.runtime, io);
    expect(failed_cycle.status.mode == qc::MotionMode::Passive, "读取失败后应回到 Passive");
    expect(io.submitted.size() == commands_before, "读取失败周期不应提交新命令");

    io.read_code = qc::RobotIOCode::Ok;
    const auto retried = motion_test::update(*created.runtime, io, &request);
    expect(retried.result.state == qc::ModeResultState::Failed, "读取失败后请求应转为 Failed");
}

// 主动动作中 submit 失败：请求转为 Failed 并回到 Passive。
void test_submit_failure_fails_active()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());

    const auto request = motion_test::make_request(1, qc::ModeRequestType::GetUp);
    motion_test::update(*created.runtime, io, &request);

    io.submit_code = qc::RobotIOCode::Rejected;
    const auto failed_cycle = motion_test::update(*created.runtime, io);
    expect(failed_cycle.status.mode == qc::MotionMode::Passive, "submit 失败后应回到 Passive");

    io.submit_code = qc::RobotIOCode::Ok;
    const auto retried = motion_test::update(*created.runtime, io, &request);
    expect(retried.result.state == qc::ModeResultState::Failed, "submit 失败后请求应转为 Failed");
}

// 主动动作前的安全状态不满足时，GetUp 请求应被拒绝而不是失败。
void test_getup_rejected_without_control_enabled()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());
    io.state.safety_state = qc::SafetyState::Damping;
    motion_test::update(*created.runtime, io);

    const auto request = motion_test::make_request(1, qc::ModeRequestType::GetUp);
    const auto output = motion_test::update(*created.runtime, io, &request);
    expect(output.result.state == qc::ModeResultState::Rejected,
        "安全状态不允许时 GetUp 应拒绝");
    expect(output.status.mode == qc::MotionMode::Passive, "拒绝后应保持 Passive");
}




}  // namespace

int main()
{
    test_joint_offline_fails_active();
    test_safety_drop_fails_active();
    test_invalid_state_fails_active();
    test_read_failure_fails_active();
    test_submit_failure_fails_active();
    test_getup_rejected_without_control_enabled();

    if (motion_test::failures > 0)
    {
        std::cerr << motion_test::failures << " 个运动错误路径测试失败\n";
        return 1;
    }
    std::cout << "全部运动错误路径测试通过\n";
    return 0;
}
