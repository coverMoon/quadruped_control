/**
 * @file sim_time.hpp
 * @brief 声明 MuJoCo 仿真秒到 int64 单调纳秒的安全转换。
 */

#pragma once

#include "quadruped/core/types.hpp"

namespace quadruped::backends::mujoco
{

// 仿真时间转换的失败原因分类，便于错误信息和测试区分具体问题。
enum class SimTimeError
{
    None = 0,  // 转换成功。
    NonFinite, // 输入为 NaN 或 Inf。
    Negative,  // 输入为负的有限值。
    Overflow,  // 换算结果超出 int64 纳秒的可表示范围。
};

// 把 MuJoCo 的 double 仿真时间（秒）转换为 int64 单调纳秒，四舍五入到最近纳秒。
// 返回 SimTimeError::None 时 out_ns 被写入；任何失败都不会修改 out_ns，
// 也保证不会发生越界 llround 或未定义行为。
SimTimeError seconds_to_nanoseconds(double seconds, core::Nanoseconds& out_ns);

}  // 命名空间 quadruped::backends::mujoco
