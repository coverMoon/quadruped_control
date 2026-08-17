/**
 * @file motion_request_tests.cpp
 * @brief 测试 MotionRuntime 的请求编号、结果终态、中断查询和未实现请求路径。
 */

#include "motion_test_helpers.hpp"

namespace
{

using motion_test::expect;

// 相同编号返回当前结果；未知旧编号拒绝；被拒绝的新请求可以按编号重试查询。
void test_request_dedup_and_stale()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    motion_test::FakeRobotIO io;
    const auto model = motion_test::make_test_model();
    io.state = motion_test::make_state(model, motion_test::make_rest_positions());

    // 从编号 5 开始，保留未使用过的编号验证旧编号拒绝。
    const auto getup = motion_test::make_request(5, qc::ModeRequestType::GetUp);
    const auto accepted = motion_test::update(*created.runtime, io, &getup);
    expect(accepted.result.state == qc::ModeResultState::Accepted, "GetUp 应被接受");

    const auto retried = motion_test::update(*created.runtime, io, &getup);
    expect(retried.result.state == qc::ModeResultState::Running,
        "执行中重试相同编号应返回 Running 且不重复执行");
    expect(retried.result.request_id == 5, "重试结果编号应与请求一致");

    const auto stale = motion_test::make_request(3, qc::ModeRequestType::EnterPassive);
    const auto stale_result = motion_test::update(*created.runtime, io, &stale);
    expect(stale_result.result.state == qc::ModeResultState::Rejected,
        "未知的旧编号应拒绝");
    expect(stale_result.status.mode == qc::MotionMode::GetUp,
        "未知旧编号拒绝不应改变当前运动阶段");

    for (int i = 0; i < 4; ++i)
    {
        motion_test::update(*created.runtime, io);
    }
    const auto completed = motion_test::update(*created.runtime, io, &getup);
    expect(completed.result.state == qc::ModeResultState::Completed,
        "起立完成后重试应报告 Completed");

    // 最新被拒绝请求的终态同样可以按编号重试查询。
    const auto stand = motion_test::make_request(6, qc::ModeRequestType::Stand);
    const auto rejected = motion_test::update(*created.runtime, io, &stand);
    expect(rejected.result.state == qc::ModeResultState::Rejected,
        "已在 Stand 时 Stand 请求应拒绝");
    const auto rejected_again = motion_test::update(*created.runtime, io, &stand);
    expect(rejected_again.result.state == qc::ModeResultState::Rejected,
        "被拒绝请求重试应返回相同终态");
}

// EnterPassive 中止主动动作时，被中止请求获得 Failed 终态并可查询。
void test_enter_passive_aborts_motion()
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

    const auto passive = motion_test::make_request(2, qc::ModeRequestType::EnterPassive);
    const auto result = motion_test::update(*created.runtime, io, &passive);
    expect(result.result.state == qc::ModeResultState::Completed, "EnterPassive 应立即完成");
    expect(result.status.mode == qc::MotionMode::Passive, "EnterPassive 后应进入 Passive");
    expect(io.submitted.back().joints[0].mode == qc::ControlMode::Disabled,
        "EnterPassive 周期应提交 Disabled");

    // 被中止的 GetUp 必须获得明确终态，而不是静默消失或退回 stale。
    const auto aborted = created.runtime->query_result(1);
    expect(aborted.state == qc::ModeResultState::Failed,
        "被中止的 GetUp 应查询到 Failed 终态");
    const auto retried = motion_test::update(*created.runtime, io, &getup);
    expect(retried.result.state == qc::ModeResultState::Failed,
        "被中止请求按旧编号重试应返回 Failed 终态");
}

// 执行中被拒绝的新请求不能覆盖活动请求的结果，原动作完成后仍可查询终态。
void test_rejected_request_preserves_active_result()
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

    // GetUp 执行中收到被拒绝的 Stand 和 GetDown，运动阶段不得改变。
    const auto stand = motion_test::make_request(2, qc::ModeRequestType::Stand);
    const auto stand_result = motion_test::update(*created.runtime, io, &stand);
    expect(stand_result.result.state == qc::ModeResultState::Rejected,
        "GetUp 执行中 Stand 应拒绝");
    expect(stand_result.status.mode == qc::MotionMode::GetUp, "拒绝后应保持 GetUp");

    const auto getdown = motion_test::make_request(3, qc::ModeRequestType::GetDown);
    const auto getdown_result = motion_test::update(*created.runtime, io, &getdown);
    expect(getdown_result.result.state == qc::ModeResultState::Rejected,
        "GetUp 执行中 GetDown 应拒绝");

    // 原 GetUp 不受影响，完成后仍可查询到 Completed。
    for (int i = 0; i < 4; ++i)
    {
        motion_test::update(*created.runtime, io);
    }
    expect(created.runtime->query_result(1).state == qc::ModeResultState::Completed,
        "被拒绝请求出现后原 GetUp 仍应查询到 Completed");
    expect(created.runtime->query_result(2).state == qc::ModeResultState::Rejected,
        "被拒绝的 Stand 应保留自身 Rejected 终态");
    expect(created.runtime->query_result(3).state == qc::ModeResultState::Rejected,
        "被拒绝的 GetDown 应保留自身 Rejected 终态");
}

// GetDown 被 GetUp 中断时：旧 GetDown 获得 Failed 终态，新 GetUp 正常推进到完成。
void test_getdown_interrupted_by_getup_terminal_states()
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

    io.state = motion_test::make_state(model, motion_test::make_test_config().stand_position);
    const auto getdown = motion_test::make_request(2, qc::ModeRequestType::GetDown);
    motion_test::update(*created.runtime, io, &getdown);
    motion_test::update(*created.runtime, io);

    // 重新起立：旧 GetDown 获得 Failed 终态，新 GetUp 获得 Accepted。
    const auto getup = motion_test::make_request(3, qc::ModeRequestType::GetUp);
    const auto accepted = motion_test::update(*created.runtime, io, &getup);
    expect(accepted.result.state == qc::ModeResultState::Accepted,
        "中断趴下的 GetUp 应被接受");

    const auto old_result = created.runtime->query_result(2);
    expect(old_result.state == qc::ModeResultState::Failed,
        "被中断的 GetDown 应查询到 Failed 终态");

    const auto old_retried = motion_test::update(*created.runtime, io, &getdown);
    expect(old_retried.result.state == qc::ModeResultState::Failed,
        "被中断的 GetDown 按旧编号重试应返回 Failed 终态");

    const auto running = motion_test::update(*created.runtime, io, &getup);
    expect(running.result.state == qc::ModeResultState::Running,
        "新 GetUp 执行中重试应返回 Running");

    for (int i = 0; i < 4; ++i)
    {
        motion_test::update(*created.runtime, io);
    }
    const auto completed = motion_test::update(*created.runtime, io, &getup);
    expect(completed.result.state == qc::ModeResultState::Completed,
        "新 GetUp 完成后应报告 Completed");
    expect(created.runtime->query_result(2).state == qc::ModeResultState::Failed,
        "新动作完成后旧 GetDown 终态不应被覆盖");
}

// StartBehavior、SwitchPolicy 和 ResetFault 在 M2 应明确拒绝且不改变模式。
void test_unimplemented_requests()
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

    auto behavior = motion_test::make_request(1, qc::ModeRequestType::StartBehavior);
    behavior.behavior_name = "rl_locomotion";
    const auto behavior_result = motion_test::update(*created.runtime, io, &behavior);
    expect(behavior_result.result.state == qc::ModeResultState::Rejected,
        "StartBehavior 应明确拒绝");

    auto policy = motion_test::make_request(2, qc::ModeRequestType::SwitchPolicy);
    policy.policy_name = "himloco_flat";
    const auto policy_result = motion_test::update(*created.runtime, io, &policy);
    expect(policy_result.result.state == qc::ModeResultState::Rejected,
        "SwitchPolicy 应明确拒绝");

    const auto fault = motion_test::make_request(3, qc::ModeRequestType::ResetFault);
    const auto fault_result = motion_test::update(*created.runtime, io, &fault);
    expect(fault_result.result.state == qc::ModeResultState::Rejected,
        "ResetFault 应明确拒绝");
    expect(fault_result.status.mode == qc::MotionMode::Passive,
        "未实现请求不应改变当前模式");
}

}  // namespace

int main()
{
    test_request_dedup_and_stale();
    test_enter_passive_aborts_motion();
    test_rejected_request_preserves_active_result();
    test_getdown_interrupted_by_getup_terminal_states();
    test_unimplemented_requests();

    if (motion_test::failures > 0)
    {
        std::cerr << motion_test::failures << " 个运动请求测试失败\n";
        return 1;
    }
    std::cout << "全部运动请求测试通过\n";
    return 0;
}
