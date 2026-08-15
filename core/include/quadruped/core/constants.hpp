/**
 * @file constants.hpp
 * @brief 定义核心模块共用的容量、版本和时间类型。
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace quadruped::core
{

// 公共帧采用固定容量，避免高频控制周期中因机器人型号不同而动态分配内存。
inline constexpr std::size_t kMaxJoints = 16;

// 修改公共帧的字段含义或二进制传输约定时必须增加该版本。
inline constexpr std::uint32_t kFrameSchemaVersion = 1;

// 所有控制时间均使用同一单调时钟的纳秒值，不能填入系统日期时间。
using Nanoseconds = std::int64_t;

}  // 命名空间 quadruped::core
