/**
 * @file motion_result_events_tests.cpp
 * @brief 测试 MotionRuntime 的终态事件交付、查询接口和固定容量历史淘汰语义。
 */

#include "motion_test_helpers.hpp"

namespace
{

using motion_test::expect;

// 从未出现过的编号通过 query_result 查询时明确拒绝。
void test_query_result_unknown()
{
    auto created = motion_test::make_runtime();
    if (!created.ok())
    {
        return;
    }
    const auto result = created.runtime->query_result(7);
    expect(result.state == qc::ModeResultState::Rejected, "未知编号查询应拒绝");
    expect(result.request_id == 7, "未知编号查询结果应保留原编号");
}

// 被中断的终态必须在产生周期通过 result_events 交付；固定容量历史淘汰后查询
// 返回 unknown，但已交付的事件副本不受后续请求影响。
void test_terminated_results_delivered()
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
    const auto abort_cycle = motion_test::update(*created.runtime, io, &passive);
    expect(abort_cycle.result.state == qc::ModeResultState::Completed,
        "EnterPassive 自身结果应直接交付");
    expect(abort_cycle.result_event_count == 1, "中止周期应交付一条终态事件");
    expect(abort_cycle.result_events[0].request_id == 1,
        "终态事件应指向被中止的 GetUp");
    expect(abort_cycle.result_events[0].state == qc::ModeResultState::Failed,
        "被中止的 GetUp 终态应为 Failed");
    const qc::ModeResult delivered = abort_cycle.result_events[0];

    // 后续大量被拒绝请求不改变已交付的事件副本，也不会在非产生周期重发。
    // 使用合法但被拒绝的 Stand 请求（Passive 中不允许跳过起立）填充历史。
    for (std::uint64_t id = 3; id <= 12; ++id)
    {
        const auto stand = motion_test::make_request(id, qc::ModeRequestType::Stand);
        const auto output = motion_test::update(*created.runtime, io, &stand);
        expect(output.result.state == qc::ModeResultState::Rejected,
            "Passive 中的 Stand 请求应被拒绝");
        expect(output.result_event_count == 0, "拒绝周期不应产生额外终态事件");
    }
    expect(delivered.state == qc::ModeResultState::Failed && delivered.request_id == 1,
        "已交付的事件副本不受后续请求影响");

    // 历史容量只有 8 条，最早的终态已被淘汰；淘汰后查询按约定返回 unknown。
    const auto evicted = created.runtime->query_result(1);
    expect(evicted.state == qc::ModeResultState::Rejected,
        "淘汰后的历史查询应按约定返回 Rejected");
}

// 无输入请求的周期产生完成/失败终态时，也必须通过 result_events 交付。
void test_completion_delivered_without_retry()
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

    // 不再重试请求，完成周期应通过事件交付 Completed 终态。
    qc::ModeResult delivered{};
    bool found = false;
    for (int i = 0; i < 4 && !found; ++i)
    {
        const auto output = motion_test::update(*created.runtime, io);
        for (std::size_t e = 0; e < output.result_event_count; ++e)
        {
            if (output.result_events[e].request_id == 1)
            {
                delivered = output.result_events[e];
                found = true;
            }
        }
    }
    expect(found, "完成周期应通过事件交付终态");
    expect(delivered.state == qc::ModeResultState::Completed,
        "事件交付的起立终态应为 Completed");
}

// 完成周期同时重试同一请求：result 是请求处理时刻快照（Running），
// 同一请求随后的 Completed 终态经 result_events 交付。
void test_retry_in_completion_cycle()
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

    // 推进到完成前的最后一周，并在完成周期重试 #1。
    for (int i = 0; i < 3; ++i)
    {
        motion_test::update(*created.runtime, io);
    }
    const auto completion_cycle = motion_test::update(*created.runtime, io, &getup);
    expect(completion_cycle.result.state == qc::ModeResultState::Running,
        "完成周期的 result 是请求处理时刻快照");
    expect(completion_cycle.result_event_count == 1 &&
        completion_cycle.result_events[0].request_id == 1 &&
        completion_cycle.result_events[0].state == qc::ModeResultState::Completed,
        "同一请求的完成终态应经 result_events 交付");

    // 下一周期重试：返回保存的 Completed 终态。
    const auto retried = motion_test::update(*created.runtime, io, &getup);
    expect(retried.result.state == qc::ModeResultState::Completed,
        "完成后重试应返回 Completed 终态");
}

}  // namespace

int main()
{
    test_query_result_unknown();
    test_terminated_results_delivered();
    test_completion_delivered_without_retry();
    test_retry_in_completion_cycle();

    if (motion_test::failures > 0)
    {
        std::cerr << motion_test::failures << " 个运动结果事件测试失败\n";
        return 1;
    }
    std::cout << "全部运动结果事件测试通过\n";
    return 0;
}
