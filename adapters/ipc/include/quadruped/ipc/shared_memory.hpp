/**
 * @file shared_memory.hpp
 * @brief 声明共享内存生命周期、数据通道和 futex 事件操作。
 */

#pragma once

#include "quadruped/ipc/wire_protocol.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace quadruped::ipc
{

enum class EventWaitResult
{
    Changed,
    TimedOut,
    Interrupted,
    Failed,
};

class SharedMemory
{
public:
    struct OpenResult
    {
        std::unique_ptr<SharedMemory> memory{};
        std::string error_message{};

        [[nodiscard]] bool ok() const noexcept
        {
            return memory != nullptr;
        }
    };

    static OpenResult create_owner(const std::string& name, const WireIdentity& identity);
    static OpenResult open_existing(const std::string& name);

    SharedMemory(const SharedMemory&) = delete;
    SharedMemory& operator=(const SharedMemory&) = delete;
    ~SharedMemory();

    SharedLayout& layout() noexcept
    {
        return *layout_;
    }

    const SharedLayout& layout() const noexcept
    {
        return *layout_;
    }

private:
    SharedMemory(std::string name, int fd, SharedLayout* layout, bool owner)
        : name_(std::move(name)), fd_(fd), layout_(layout), owner_(owner)
    {
    }

    std::string name_{};
    int fd_{-1};
    SharedLayout* layout_{nullptr};
    bool owner_{false};
};

[[nodiscard]] std::int64_t monotonic_now_ns() noexcept;

// 返回当前序列快照。处理数据后应再次比较快照，再决定是否等待。
[[nodiscard]] std::uint32_t event_sequence(const WireEvent& event) noexcept;

// 先发布递增后的序列，再唤醒所有进程共享等待者。
void notify_event(WireEvent& event) noexcept;

// 使用相对单调时钟超时等待；序列已变化时立即返回 Changed。
[[nodiscard]] EventWaitResult wait_event(
    WireEvent& event,
    std::uint32_t expected_sequence,
    std::int64_t timeout_ns) noexcept;

template<typename T>
void publish_latest(LatestSlot<T>& slot, const T& value) noexcept
{
    while (slot.lock.exchange(1, std::memory_order_acquire) != 0)
    {
    }
    slot.value = value;
    slot.version.fetch_add(1, std::memory_order_release);
    slot.lock.store(0, std::memory_order_release);
}

template<typename T>
void publish_latest_and_notify(
    LatestSlot<T>& slot,
    const T& value,
    WireEvent& event) noexcept
{
    publish_latest(slot, value);
    notify_event(event);
}

template<typename T>
bool read_latest(const LatestSlot<T>& slot, T& value, std::uint64_t* version = nullptr) noexcept
{
    // 连续四次立即重试很容易全部落在 500 Hz/1 kHz 发布窗口内，进而把
    // 正常锁竞争误报为“无数据”。仍保持有界非阻塞，并避免失败读者反复写锁缓存行。
    constexpr int kReadAttempts = 64;
    for (int attempt = 0; attempt < kReadAttempts; ++attempt)
    {
        std::uint32_t expected = 0;
        if (!slot.lock.compare_exchange_weak(
                expected,
                1,
                std::memory_order_acquire,
                std::memory_order_relaxed))
        {
            continue;
        }
        const std::uint64_t current = slot.version.load(std::memory_order_acquire);
        if (current != 0)
        {
            value = slot.value;
        }
        slot.lock.store(0, std::memory_order_release);
        if (current == 0)
        {
            return false;
        }
        if (version != nullptr)
        {
            *version = current;
        }
        return true;
    }
    return false;
}

template<typename T, std::size_t Capacity>
bool queue_push(SpscQueue<T, Capacity>& queue, const T& value) noexcept
{
    const std::uint64_t write = queue.write_index.load(std::memory_order_relaxed);
    const std::uint64_t read = queue.read_index.load(std::memory_order_acquire);
    if (write - read >= Capacity)
    {
        return false;
    }
    queue.entries[write % Capacity] = value;
    queue.write_index.store(write + 1, std::memory_order_release);
    return true;
}

template<typename T, std::size_t Capacity>
bool queue_push_and_notify(
    SpscQueue<T, Capacity>& queue,
    const T& value,
    WireEvent& event) noexcept
{
    if (!queue_push(queue, value))
    {
        return false;
    }
    notify_event(event);
    return true;
}

template<typename T, std::size_t Capacity>
bool queue_pop(SpscQueue<T, Capacity>& queue, T& value) noexcept
{
    const std::uint64_t read = queue.read_index.load(std::memory_order_relaxed);
    const std::uint64_t write = queue.write_index.load(std::memory_order_acquire);
    if (read == write)
    {
        return false;
    }
    value = queue.entries[read % Capacity];
    queue.read_index.store(read + 1, std::memory_order_release);
    return true;
}

}  // 命名空间 quadruped::ipc
