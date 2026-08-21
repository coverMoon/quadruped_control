/**
 * @file ipc_control.cpp
 * @brief 提供不依赖 ROS 2 的本机共享内存后端复位控制入口。
 */

#include "quadruped/core/constants.hpp"
#include "quadruped/ipc/shared_memory.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

namespace qi = quadruped::ipc;
namespace
{

constexpr auto kWaitTimeout = std::chrono::seconds(5);

std::uint64_t make_request_id()
{
    const auto value = static_cast<std::uint64_t>(qi::monotonic_now_ns());
    return value == 0 ? 1 : value;
}

std::string read_message(const std::array<char, qi::kWireMessageCapacity>& message)
{
    const auto end = std::find(message.begin(), message.end(), '\0');
    return std::string(message.begin(), end);
}

}  // 匿名命名空间

int main(int argc, char** argv)
{
    if (argc != 3 || std::string(argv[2]) != "reset")
    {
        std::cerr << "用法: " << argv[0] << " <共享内存名称> reset\n";
        return 2;
    }

    const auto opened = qi::SharedMemory::open_existing(argv[1]);
    if (!opened.ok())
    {
        std::cerr << "打开共享内存失败: " << opened.error_message << '\n';
        return 1;
    }
    qi::SharedLayout& layout = opened.memory->layout();
    qi::WireHeartbeat heartbeat;
    if (!qi::read_latest(layout.backend_heartbeat, heartbeat) || heartbeat.online == 0)
    {
        std::cerr << "MuJoCo 后端尚未在线\n";
        return 1;
    }

    qi::WireControlRequest request;
    request.schema_version = quadruped::core::kFrameSchemaVersion;
    request.startup_id = heartbeat.startup_id;
    request.session_id = heartbeat.session_id;
    request.request_id = make_request_id();
    request.type = static_cast<std::uint8_t>(qi::WireControlType::Reset);
    if (!qi::queue_push(layout.control_requests, request))
    {
        std::cerr << "后端控制请求队列已满\n";
        return 1;
    }

    const auto deadline = std::chrono::steady_clock::now() + kWaitTimeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        qi::WireControlResult result;
        while (qi::queue_pop(layout.control_results, result))
        {
            if (result.request_id != request.request_id)
            {
                continue;
            }
            std::cout << read_message(result.message) << " session=" << result.session_id << '\n';
            return result.success == 1 ? 0 : 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cerr << "等待后端复位结果超时\n";
    return 1;
}
