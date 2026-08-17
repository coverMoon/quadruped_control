/**
 * @file motion_results.cpp
 * @brief 实现 MotionRuntime 的请求结果生命周期：完成、中止、终态存储和查询。
 */

#include "quadruped/motion/motion_runtime.hpp"

#include <cstddef>
#include <string>

namespace quadruped::motion
{

void MotionRuntime::complete_active_request(const char* message)
{
    if (!has_active_request_)
    {
        return;
    }
    active_result_.state = core::ModeResultState::Completed;
    active_result_.message = message;
    store_terminal_result(active_result_);
    append_result_event(active_result_);
    has_active_request_ = false;
}

void MotionRuntime::abort_active_request(const std::string& reason)
{
    if (!has_active_request_)
    {
        return;
    }
    active_result_.state = core::ModeResultState::Failed;
    active_result_.message = reason;
    store_terminal_result(active_result_);
    append_result_event(active_result_);
    has_active_request_ = false;
}

void MotionRuntime::store_terminal_result(const core::ModeResult& result)
{
    // 覆盖前的结果已在产生周期通过更新输出交付，这里只服务重试查询。
    terminal_results_[terminal_next_] = result;
    terminal_next_ = (terminal_next_ + 1) % terminal_results_.size();
    if (terminal_count_ < terminal_results_.size())
    {
        ++terminal_count_;
    }
}

bool MotionRuntime::find_terminal_result(
    const std::uint64_t request_id,
    core::ModeResult& result) const
{
    // 环形存储：有效条目是最近写入的 terminal_count_ 个，从最旧开始查找。
    for (std::size_t i = 0; i < terminal_count_; ++i)
    {
        const std::size_t index =
            (terminal_next_ + terminal_results_.size() - terminal_count_ + i) %
            terminal_results_.size();
        if (terminal_results_[index].request_id == request_id)
        {
            result = terminal_results_[index];
            return true;
        }
    }
    return false;
}

void MotionRuntime::append_result_event(const core::ModeResult& result)
{
    // 单周期至多产生一项终态事件，容量 2 留有裕量；越界说明内部状态错误。
    if (pending_result_event_count_ < pending_result_events_.size())
    {
        pending_result_events_[pending_result_event_count_++] = result;
    }
}

core::ModeResult MotionRuntime::query_result(const std::uint64_t request_id) const
{
    if (has_active_request_ && request_id == active_request_id_)
    {
        return active_result_;
    }
    core::ModeResult stored{};
    if (find_terminal_result(request_id, stored))
    {
        return stored;
    }
    return core::ModeResult{request_id, core::ModeResultState::Rejected,
        "unknown request id"};
}

}  // 命名空间 quadruped::motion
