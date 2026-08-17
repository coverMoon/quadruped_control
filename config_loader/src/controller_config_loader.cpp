/**
 * @file controller_config_loader.cpp
 * @brief 实现 ControllerConfig 的 YAML 启动期加载、关节顺序核对和核心校验。
 */

#include "quadruped/config/robot_config.hpp"

#include "yaml_helpers.hpp"

#include "quadruped/core/validation.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <string>

namespace quadruped::config
{
namespace
{

// 1 ms 对应的纳秒数，用于把 YAML 中的毫秒配置转换为内部纳秒表示。
inline constexpr std::int64_t kNanosecondsPerMillisecond = 1'000'000;

// 读取以 ms 为单位的正时间字段并转换为 ns；非正值直接拒绝。
bool read_duration_ms(
    detail::FieldReader& reader,
    const char* key,
    core::Nanoseconds& out_ns)
{
    std::int64_t ms = 0;
    if (!reader.required(key, ms))
    {
        return false;
    }
    if (ms <= 0)
    {
        return reader.fail(std::string("key \"") + key + "\" must be positive");
    }
    // 换算前检查乘法溢出，避免极大但可解析的值触发有符号溢出未定义行为。
    if (ms > INT64_MAX / kNanosecondsPerMillisecond)
    {
        return reader.fail(
            std::string("key \"") + key + "\" is out of representable range");
    }
    out_ns = static_cast<core::Nanoseconds>(ms * kNanosecondsPerMillisecond);
    return true;
}

// 读取正的控制周期数字段；0、负值和超出 32 位范围的值直接拒绝。
bool read_cycle_count(
    detail::FieldReader& reader,
    const char* key,
    std::uint32_t& out)
{
    std::int64_t value = 0;
    if (!reader.required(key, value))
    {
        return false;
    }
    if (value <= 0 || value > static_cast<std::int64_t>(UINT32_MAX))
    {
        return reader.fail(
            std::string("key \"") + key + "\" must be a positive 32-bit cycle count");
    }
    out = static_cast<std::uint32_t>(value);
    return true;
}

// 核对控制器配置中的关节名称顺序与 RobotModel 完全一致，防止数组与模型错位。
bool check_joint_order(
    const YAML::Node& root,
    const core::RobotModel& model,
    std::string& error)
{
    const YAML::Node names = root["joint_names"];
    if (!names || !names.IsSequence() || names.size() != model.joint_count)
    {
        error = "\"joint_names\" must be a sequence matching the RobotModel joint count";
        return false;
    }
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        std::string name;
        try
        {
            name = names[i].as<std::string>();
        }
        catch (const YAML::Exception& e)
        {
            // 非字符串条目必须作为配置错误返回，不能把异常抛出模块边界。
            error = "joint_names[" + std::to_string(i) + "] must be a string: " + e.what();
            return false;
        }
        if (name != model.joints[i].name)
        {
            error = "joint order mismatch at index " + std::to_string(i) + ": config has \"" +
                name + "\", RobotModel expects \"" + model.joints[i].name + "\"";
            return false;
        }
    }
    return true;
}

// 读取周期、周期数、关节顺序等标量字段；任何缺失或非法都会失败。
bool read_controller_scalars(
    detail::FieldReader& reader,
    const core::RobotModel& model,
    core::ControllerConfig& config)
{
    return read_duration_ms(reader, "control_period_ms", config.control_period_ns) &&
        read_duration_ms(reader, "command_validity_ms", config.command_validity_ns) &&
        read_cycle_count(reader, "getup_pre_cycles", config.getup_pre_cycles) &&
        read_cycle_count(reader, "getup_cycles", config.getup_cycles) &&
        read_cycle_count(reader, "getdown_cycles", config.getdown_cycles) &&
        check_joint_order(reader.node(), model, reader.error_message());
}

// 读取姿态和增益数组；维度或数值错误都会失败。
bool read_controller_arrays(
    detail::FieldReader& reader,
    const core::RobotModel& model,
    core::ControllerConfig& config)
{
    const std::size_t count = model.joint_count;
    return reader.double_array("pre_getup_position", config.pre_getup_position.data(), count) &&
        reader.double_array("stand_position", config.stand_position.data(), count) &&
        reader.double_array("fixed_kp", config.fixed_kp.data(), count) &&
        reader.double_array("fixed_kd", config.fixed_kd.data(), count);
}

}  // 匿名命名空间

ControllerConfigLoadResult load_controller_config(
    const std::string& path,
    const core::RobotModel& model)
{
    ControllerConfigLoadResult result;

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const YAML::Exception& e)
    {
        result.error_message = "cannot load controller config file \"" + path + "\": " + e.what();
        return result;
    }
    // 根节点必须是映射；标量或序列根节点会让后续按键索引抛异常。
    if (!root.IsMap())
    {
        result.error_message = "controller config root must be a mapping";
        return result;
    }

    std::string& error = result.error_message;
    detail::FieldReader reader(root, error);
    std::string name;
    if (!reader.required("name", name))
    {
        return result;
    }
    if (name != model.name)
    {
        error = "controller config is for robot \"" + name +
            "\", but RobotModel is \"" + model.name + "\"";
        return result;
    }

    auto& config = result.config;
    if (!read_controller_scalars(reader, model, config) ||
        !read_controller_arrays(reader, model, config))
    {
        return result;
    }

    if (const auto validation = core::validate(config, model); !validation)
    {
        error = "controller config validation failed: " + validation.message;
        return result;
    }
    return result;
}

}  // 命名空间 quadruped::config
