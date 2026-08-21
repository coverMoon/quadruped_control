/**
 * @file conversions.hpp
 * @brief 声明 core 数据结构与固定容量 wire schema 之间的显式转换。
 */

#pragma once

#include "quadruped/core/robot_io.hpp"
#include "quadruped/core/robot_model.hpp"
#include "quadruped/core/types.hpp"
#include "quadruped/ipc/wire_protocol.hpp"

#include <string>

namespace quadruped::ipc
{

WireIdentity make_identity(const core::RobotModel& model);
bool identity_matches(
    const WireIdentity& identity,
    const core::RobotModel& model,
    std::string& error_message);

WireStateFrame to_wire(const core::StateFrame& frame);
bool from_wire(const WireStateFrame& wire, core::StateFrame& frame);
WireCommandFrame to_wire(const core::CommandFrame& frame);
bool from_wire(const WireCommandFrame& wire, core::CommandFrame& frame);
WireBaseCommand to_wire(const core::BaseCommand& command);
bool from_wire(const WireBaseCommand& wire, core::BaseCommand& command);
WireModeRequest to_wire(const core::ModeRequest& request);
bool from_wire(const WireModeRequest& wire, core::ModeRequest& request);
WireModeResult to_wire(const core::ModeResult& result);
bool from_wire(const WireModeResult& wire, core::ModeResult& result);
WireMotionStatus to_wire(const core::MotionStatus& status);
bool from_wire(const WireMotionStatus& wire, core::MotionStatus& status);
WireRobotIOStatus to_wire(const core::RobotIOStatus& status);
bool from_wire(const WireRobotIOStatus& wire, core::RobotIOStatus& status);

}  // 命名空间 quadruped::ipc
