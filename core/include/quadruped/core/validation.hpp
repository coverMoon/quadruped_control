/**
 * @file validation.hpp
 * @brief 声明机器人模型和控制数据的合法性检查接口。
 */

#pragma once

#include "quadruped/core/robot_model.hpp"
#include "quadruped/core/types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace quadruped::core
{

// 校验失败的机器可读分类；详细原因保存在 ValidationResult::message。
enum class ValidationError : std::uint8_t
{
    None = 0,
    InvalidName,
    InvalidIdentifier,
    InvalidJointCount,
    DuplicateJointName,
    InvalidJointRole,
    InvalidJointLimits,
    SchemaMismatch,
    ModelMismatch,
    CalibrationMismatch,
    InvalidTimestamp,
    Expired,
    InvalidControlMode,
    InvalidCommandSource,
    InvalidMotionMode,
    InvalidModeRequest,
    NonFiniteValue,
    NegativeValue,
};

// 所有 validate 重载共用的返回结果。
struct ValidationResult
{
    // None 表示校验通过，其他枚举表示第一个发现的问题。
    ValidationError error{ValidationError::None};

    // 问题所属的关节下标；错误不对应具体关节时等于 kMaxJoints。
    std::size_t joint_index{kMaxJoints};

    // 面向日志和开发者的错误说明，不能作为程序分支的判断依据。
    std::string message{};

    [[nodiscard]] bool ok() const noexcept
    {
        return error == ValidationError::None;
    }

    explicit operator bool() const noexcept
    {
        return ok();
    }
};

// RobotModel 校验不依赖当前时间，主要检查名称、维度、重复关节和限制。
[[nodiscard]] ValidationResult validate(const RobotModel& model);

// frame 校验假定 model 已在启动时通过 validate(model)，不会重复检查稳定模型。
// now_ns 与 frame 必须来自同一单调时钟，用于拒绝未来时间戳。
[[nodiscard]] ValidationResult validate(
    const StateFrame& frame,
    const RobotModel& model,
    Nanoseconds now_ns);

// 除公共帧字段外，还检查命令有效期、控制模式和已校验 RobotModel 的限制。
[[nodiscard]] ValidationResult validate(
    const CommandFrame& frame,
    const RobotModel& model,
    Nanoseconds now_ns);

// BaseCommand 和 ModeRequest 的时间戳同样必须使用调用方的单调时钟。
[[nodiscard]] ValidationResult validate(const BaseCommand& command, Nanoseconds now_ns);
[[nodiscard]] ValidationResult validate(const ModeRequest& request, Nanoseconds now_ns);

}  // 命名空间 quadruped::core
