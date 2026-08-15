/**
 * @file sim_time.cpp
 * @brief 实现 MuJoCo 仿真秒到 int64 单调纳秒的安全转换。
 */

#include "sim_time.hpp"

#include <cmath>

namespace quadruped::backends::mujoco
{
namespace
{

// 秒到纳秒的换算比例。
constexpr double kNanosecondsPerSecond = 1.0e9;

// int64 上界 2^63 的精确 double 表示。换算乘积达到或超过该值时，llround 的结果
// 无法由 int64 表示，必须提前拒绝；小于 2^63 的最大 double 是 2^63 - 1024
// （该量级下 double 间距为 1024），其取整结果仍落在 int64 范围内。
constexpr double kInt64ExclusiveBoundNs = 9223372036854775808.0;

}  // 匿名命名空间

std::string sim_time_error_message(const SimTimeError error)
{
    switch (error)
    {
    case SimTimeError::NonFinite:
        return "MuJoCo simulation time is not finite";
    case SimTimeError::Negative:
        return "MuJoCo simulation time is negative";
    case SimTimeError::Overflow:
        return "MuJoCo simulation time exceeds the int64 nanosecond range";
    case SimTimeError::None:
        break;
    }
    return {};
}

SimTimeError seconds_to_nanoseconds(const double seconds, core::Nanoseconds& out_ns)
{
    // NaN 与任何比较都为 false，因此必须先显式拒绝非有限输入，
    // 否则 NaN 会漏过溢出检查，使 llround(NaN) 产生未定义行为。
    if (!std::isfinite(seconds))
    {
        return SimTimeError::NonFinite;
    }
    if (seconds < 0.0)
    {
        return SimTimeError::Negative;
    }
    // 先在 double 域完成乘法再做边界比较：乘积是精确的 double 运算，
    // 与 2^63 的比较不存在舍入歧义；超大输入得到 Inf 时同样按溢出拒绝。
    const double nanoseconds = seconds * kNanosecondsPerSecond;
    if (nanoseconds >= kInt64ExclusiveBoundNs)
    {
        return SimTimeError::Overflow;
    }
    // 此处 nanoseconds < 2^63，故 llround 结果最大为 2^63 - 1024，必然可表示。
    out_ns = static_cast<core::Nanoseconds>(std::llround(nanoseconds));
    return SimTimeError::None;
}

}  // 命名空间 quadruped::backends::mujoco
