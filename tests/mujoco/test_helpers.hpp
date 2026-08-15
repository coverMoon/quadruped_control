/**
 * @file test_helpers.hpp
 * @brief MuJoCo 测试共用的机器人模型、命令构造和断言辅助函数。
 */

#pragma once

#include "quadruped/backends/mujoco/mujoco_robot_io.hpp"
#include "quadruped/core/core.hpp"

#include <cstdint>
#include <iostream>
#include <string>

namespace qc = quadruped::core;

namespace quadruped::backends::mujoco::test
{

extern int failures;

void expect(bool condition, const std::string& description);
void expect_close(double actual, double expected, double tolerance, const std::string& description);

qc::RobotModel make_black_model();

MujocoRobotIO::CreateResult create_ready(const std::string& scene_path, std::uint64_t session_id);

qc::CommandFrame make_command(
    std::uint64_t sequence,
    qc::ControlMode mode,
    std::uint64_t timestamp_ns = 0,
    std::uint64_t expires_at_ns = 1'000'000'000);

}  // namespace quadruped::backends::mujoco::test
