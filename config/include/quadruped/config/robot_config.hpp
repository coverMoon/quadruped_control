/**
 * @file robot_config.hpp
 * @brief 声明启动期 YAML 配置加载接口，输出已校验的 RobotModel 和 ControllerConfig。
 */

#pragma once

#include "quadruped/core/controller_config.hpp"
#include "quadruped/core/robot_model.hpp"

#include <string>

namespace quadruped::config
{

// load_robot_model() 的返回结果：成功时 model 已通过核心校验，失败时 error_message 说明原因。
struct RobotModelLoadResult
{
    core::RobotModel model{};
    std::string error_message{};

    [[nodiscard]] bool ok() const noexcept
    {
        return error_message.empty();
    }

    explicit operator bool() const noexcept
    {
        return ok();
    }
};

// load_controller_config() 的返回结果：成功时 config 已通过核心校验。
struct ControllerConfigLoadResult
{
    core::ControllerConfig config{};
    std::string error_message{};

    [[nodiscard]] bool ok() const noexcept
    {
        return error_message.empty();
    }

    explicit operator bool() const noexcept
    {
        return ok();
    }
};

// 从 YAML 文件加载 RobotModel 并执行核心校验；缺字段、非法枚举、非法限制都会失败。
// 只在启动期调用，高频控制路径不得使用。
[[nodiscard]] RobotModelLoadResult load_robot_model(const std::string& path);

// 从 YAML 文件加载 ControllerConfig，并按已校验的 RobotModel 核对名称、关节顺序、
// 数组维度和数值范围；任何不一致都会拒绝加载。
[[nodiscard]] ControllerConfigLoadResult load_controller_config(
    const std::string& path,
    const core::RobotModel& model);

}  // 命名空间 quadruped::config
