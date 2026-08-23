/**
 * @file behavior_config_loader.cpp
 * @brief 实现 Retry 和 Event chain 配置加载及机器人能力校验。
 */

#include "quadruped/config/behavior_config_loader.hpp"

#include "yaml_helpers.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <string>

namespace quadruped::config
{
namespace
{

bool load_root(const std::string& path, YAML::Node& root, std::string& error)
{
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const YAML::Exception& exception)
    {
        error = "cannot load behavior config file \"" + path + "\": " + exception.what();
        return false;
    }
    if (!root.IsMap())
    {
        error = "behavior config root must be a mapping";
        return false;
    }
    return true;
}

bool read_positive_cycles(
    detail::FieldReader& reader,
    const char* key,
    std::uint32_t& output)
{
    std::int64_t value = 0;
    if (!reader.required(key, value))
    {
        return false;
    }
    if (value <= 0 || value > static_cast<std::int64_t>(UINT32_MAX))
    {
        return reader.fail(std::string("key \"") + key +
            "\" must be a positive 32-bit cycle count");
    }
    output = static_cast<std::uint32_t>(value);
    return true;
}

bool read_nonnegative_cycles(
    detail::FieldReader& reader,
    const char* key,
    std::uint32_t& output)
{
    std::int64_t value = 0;
    if (!reader.required(key, value))
    {
        return false;
    }
    if (value < 0 || value > static_cast<std::int64_t>(UINT32_MAX))
    {
        return reader.fail(std::string("key \"") + key +
            "\" must be a nonnegative 32-bit cycle count");
    }
    output = static_cast<std::uint32_t>(value);
    return true;
}

bool read_optional_nonnegative_cycles(
    detail::FieldReader& reader,
    const char* key,
    std::uint32_t& output)
{
    if (!reader.node()[key])
    {
        output = 0;
        return true;
    }
    return read_nonnegative_cycles(reader, key, output);
}

bool check_robot_name(
    detail::FieldReader& reader,
    const core::RobotModel& model,
    std::string& output)
{
    if (!reader.required("robot_name", output))
    {
        return false;
    }
    if (output != model.name)
    {
        return reader.fail("behavior config is for robot \"" + output +
            "\", but RobotModel is \"" + model.name + "\"");
    }
    return true;
}

bool read_joint_names(
    const YAML::Node& root,
    const core::RobotModel& model,
    std::array<std::string, core::kMaxJoints>& output,
    std::string& error)
{
    const YAML::Node names = root["joint_names"];
    if (!names || !names.IsSequence() || names.size() != model.joint_count)
    {
        error = "\"joint_names\" must match the RobotModel joint count";
        return false;
    }
    try
    {
        for (std::size_t i = 0; i < model.joint_count; ++i)
        {
            output[i] = names[i].as<std::string>();
            if (output[i] != model.joints[i].name)
            {
                error = "joint order mismatch at index " + std::to_string(i);
                return false;
            }
        }
    }
    catch (const YAML::Exception& exception)
    {
        error = "invalid joint_names: " + std::string(exception.what());
        return false;
    }
    return true;
}

bool validate_gains_and_pose(
    const core::RobotModel& model,
    const std::array<double, core::kMaxJoints>& positions,
    const std::array<double, core::kMaxJoints>& kp,
    const std::array<double, core::kMaxJoints>& kd,
    std::string& error)
{
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        const auto& joint = model.joints[i];
        if (!std::isfinite(positions[i]) || !std::isfinite(kp[i]) ||
            !std::isfinite(kd[i]) || kp[i] < 0.0 || kd[i] < 0.0 ||
            kp[i] > joint.limits.max_kp || kd[i] > joint.limits.max_kd)
        {
            error = "behavior joint values are invalid at index " + std::to_string(i);
            return false;
        }
        if (joint.limits.position_limited &&
            (positions[i] < joint.limits.min_position ||
                positions[i] > joint.limits.max_position))
        {
            error = "behavior pose exceeds position limits at index " + std::to_string(i);
            return false;
        }
        if (joint.role == core::JointRole::Wheel && kp[i] != 0.0)
        {
            error = "Wheel KP must be zero at index " + std::to_string(i);
            return false;
        }
    }
    return true;
}

bool parse_event_type(const std::string& text, motion::EventType& output)
{
    if (text == "pose")
    {
        output = motion::EventType::Pose;
        return true;
    }
    if (text == "drive")
    {
        output = motion::EventType::Drive;
        return true;
    }
    if (text == "pose_drive")
    {
        output = motion::EventType::PoseDrive;
        return true;
    }
    return false;
}

bool parse_wheel_group(const std::string& text, motion::WheelGroup& output)
{
    if (text == "front")
    {
        output = motion::WheelGroup::Front;
        return true;
    }
    if (text == "rear")
    {
        output = motion::WheelGroup::Rear;
        return true;
    }
    if (text == "all")
    {
        output = motion::WheelGroup::All;
        return true;
    }
    return false;
}

bool parse_wheel_side(const std::string& text, motion::WheelSide& output)
{
    if (text == "left")
    {
        output = motion::WheelSide::Left;
        return true;
    }
    if (text == "right")
    {
        output = motion::WheelSide::Right;
        return true;
    }
    return false;
}

bool read_event(
    const YAML::Node& node,
    const core::RobotModel& model,
    motion::EventChainEvent& event,
    std::string& error)
{
    if (!node.IsMap())
    {
        error = "each Event chain event must be a mapping";
        return false;
    }
    detail::FieldReader reader(node, error);
    std::string type;
    if (!reader.required("name", event.name) || event.name.empty() ||
        !reader.required("type", type) || !parse_event_type(type, event.type) ||
        !read_optional_nonnegative_cycles(reader, "hold_cycles", event.hold_cycles))
    {
        if (error.empty())
        {
            error = "event name or type is invalid";
        }
        return false;
    }

    const bool uses_pose = event.type == motion::EventType::Pose ||
        event.type == motion::EventType::PoseDrive;
    const bool uses_wheel = event.type == motion::EventType::Drive ||
        event.type == motion::EventType::PoseDrive;
    if (uses_pose &&
        (!read_positive_cycles(reader, "transition_cycles", event.transition_cycles) ||
            !reader.double_array(
                "dof_pos", event.dof_positions.data(), model.joint_count)))
    {
        return false;
    }
    if (!uses_wheel)
    {
        event.wheel_group = motion::WheelGroup::None;
        return true;
    }

    std::string wheel_group;
    if (!reader.required("wheel_group", wheel_group) ||
        !reader.required("distance_m", event.distance_m) ||
        !reader.required("speed_mps", event.speed_mps) ||
        !read_positive_cycles(reader, "timeout_cycles", event.timeout_cycles))
    {
        return false;
    }
    if (!parse_wheel_group(wheel_group, event.wheel_group))
    {
        error = "wheel event requires wheel_group front, rear, or all";
        return false;
    }
    if (!std::isfinite(event.distance_m) || !std::isfinite(event.speed_mps) ||
        event.distance_m == 0.0 || event.speed_mps <= 0.0)
    {
        error = "wheel event distance must be nonzero; speed and timeout must be positive";
        return false;
    }
    if (event.type == motion::EventType::PoseDrive &&
        event.timeout_cycles < event.transition_cycles)
    {
        error = "pose_drive timeout must cover its pose transition";
        return false;
    }
    return true;
}

}  // 匿名命名空间

RetryConfigLoadResult load_retry_config(
    const std::string& path,
    const core::RobotModel& model)
{
    RetryConfigLoadResult result;
    YAML::Node root;
    if (!load_root(path, root, result.error_message))
    {
        return result;
    }
    detail::FieldReader reader(root, result.error_message);
    auto& config = result.config;
    config.joint_count = model.joint_count;
    if (!check_robot_name(reader, model, config.robot_name) ||
        !read_positive_cycles(reader, "prepare_cycles", config.prepare_cycles) ||
        !read_joint_names(root, model, config.joint_names, result.error_message) ||
        !reader.double_array(
            "retry_default_joint_positions", config.target_positions.data(), model.joint_count) ||
        !reader.double_array("kp", config.kp.data(), model.joint_count) ||
        !reader.double_array("kd", config.kd.data(), model.joint_count))
    {
        return result;
    }
    validate_gains_and_pose(
        model, config.target_positions, config.kp, config.kd, result.error_message);
    return result;
}

FixedDriveConfigLoadResult load_fixed_drive_config(
    const std::string& path,
    const core::RobotModel& model)
{
    FixedDriveConfigLoadResult result;
    YAML::Node root;
    if (!load_root(path, root, result.error_message))
    {
        return result;
    }
    detail::FieldReader reader(root, result.error_message);
    auto& config = result.config;
    config.joint_count = model.joint_count;
    if (!reader.required("behavior_name", config.behavior_name) ||
        (config.behavior_name != "car_drive" &&
            config.behavior_name != "bridge_drive" &&
            config.behavior_name != "low_bar_drive") ||
        !check_robot_name(reader, model, config.robot_name) ||
        !read_joint_names(root, model, config.joint_names, result.error_message) ||
        !read_positive_cycles(reader, "prepare_cycles", config.prepare_cycles) ||
        !read_positive_cycles(reader, "exit_to_rl_cycles", config.exit_to_rl_cycles) ||
        !reader.required("max_x", config.max_x) ||
        !reader.required("max_yaw", config.max_yaw) ||
        !reader.required("wheel_velocity_scale", config.wheel_velocity_scale) ||
        !reader.required("yaw_to_wheel_velocity", config.yaw_to_wheel_velocity) ||
        !reader.double_array(
            "target_joint_positions", config.target_positions.data(), model.joint_count) ||
        !reader.double_array("kp", config.kp.data(), model.joint_count) ||
        !reader.double_array("kd", config.kd.data(), model.joint_count))
    {
        if (result.error_message.empty())
        {
            result.error_message = "fixed drive behavior_name is invalid";
        }
        return result;
    }
    if (!std::isfinite(config.max_x) || config.max_x <= 0.0 ||
        !std::isfinite(config.max_yaw) || config.max_yaw <= 0.0 ||
        !std::isfinite(config.wheel_velocity_scale) ||
        config.wheel_velocity_scale <= 0.0 ||
        !std::isfinite(config.yaw_to_wheel_velocity) ||
        config.yaw_to_wheel_velocity <= 0.0)
    {
        result.error_message = "fixed drive limits and velocity scales must be positive";
        return result;
    }
    if (!validate_gains_and_pose(model,
            config.target_positions,
            config.kp,
            config.kd,
            result.error_message))
    {
        return result;
    }

    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        if (model.joints[i].role == core::JointRole::Wheel)
        {
            ++config.wheel_count;
        }
    }
    const YAML::Node signs = root["wheel_velocity_sign"];
    const YAML::Node sides = root["wheel_sides"];
    if (config.wheel_count == 0 || !signs || !signs.IsSequence() ||
        signs.size() != config.wheel_count || !sides || !sides.IsSequence() ||
        sides.size() != config.wheel_count)
    {
        result.error_message =
            "fixed drive requires one velocity sign and side for every Wheel joint";
        return result;
    }
    bool has_left = false;
    bool has_right = false;
    try
    {
        for (std::size_t wheel_id = 0; wheel_id < config.wheel_count; ++wheel_id)
        {
            config.wheel_velocity_sign[wheel_id] = signs[wheel_id].as<double>();
            const std::string side = sides[wheel_id].as<std::string>();
            if (!std::isfinite(config.wheel_velocity_sign[wheel_id]) ||
                config.wheel_velocity_sign[wheel_id] == 0.0 ||
                !parse_wheel_side(side, config.wheel_sides[wheel_id]))
            {
                result.error_message = "fixed drive wheel sign or side is invalid";
                return result;
            }
            has_left = has_left || config.wheel_sides[wheel_id] == motion::WheelSide::Left;
            has_right = has_right || config.wheel_sides[wheel_id] == motion::WheelSide::Right;
        }
    }
    catch (const YAML::Exception& exception)
    {
        result.error_message = "invalid fixed drive wheel mapping: " +
            std::string(exception.what());
        return result;
    }
    if (!has_left || !has_right)
    {
        result.error_message = "fixed drive requires both left and right Wheel joints";
        return result;
    }

    const double worst_velocity =
        config.wheel_velocity_scale * config.max_x +
        config.yaw_to_wheel_velocity * config.max_yaw;
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        if (model.joints[i].role == core::JointRole::Wheel &&
            worst_velocity > model.joints[i].limits.max_velocity)
        {
            result.error_message = "fixed drive maximum wheel velocity exceeds RobotModel";
            return result;
        }
    }
    return result;
}

EventChainConfigLoadResult load_event_chain_config(
    const std::string& path,
    const core::RobotModel& model)
{
    EventChainConfigLoadResult result;
    YAML::Node root;
    if (!load_root(path, root, result.error_message))
    {
        return result;
    }
    detail::FieldReader reader(root, result.error_message);
    auto& config = result.config;
    config.joint_count = model.joint_count;
    if (!check_robot_name(reader, model, config.robot_name) ||
        !read_positive_cycles(reader, "exit_to_rl_cycles", config.exit_to_rl_cycles) ||
        !read_joint_names(root, model, config.joint_names, result.error_message) ||
        !reader.required("interpolation", config.interpolation) ||
        (config.interpolation != "linear" && config.interpolation != "smoothstep") ||
        !reader.double_array("kp", config.kp.data(), model.joint_count) ||
        !reader.double_array("kd", config.kd.data(), model.joint_count))
    {
        if (result.error_message.empty())
        {
            result.error_message = "Event chain interpolation must be linear or smoothstep";
        }
        return result;
    }

    const YAML::Node events = root["events"];
    if (!events || !events.IsSequence() || events.size() == 0 ||
        events.size() > motion::kMaxEventChainEvents)
    {
        result.error_message = "Event chain events must be a nonempty bounded sequence";
        return result;
    }

    bool has_wheel = false;
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        has_wheel = has_wheel || model.joints[i].role == core::JointRole::Wheel;
        if (!std::isfinite(config.kp[i]) || !std::isfinite(config.kd[i]) ||
            config.kp[i] < 0.0 || config.kd[i] < 0.0 ||
            config.kp[i] > model.joints[i].limits.max_kp ||
            config.kd[i] > model.joints[i].limits.max_kd ||
            (model.joints[i].role == core::JointRole::Wheel && config.kp[i] != 0.0))
        {
            result.error_message = "Event chain gains are invalid at index " +
                std::to_string(i);
            return result;
        }
    }

    const YAML::Node signs = root["wheel_velocity_sign"];
    if (has_wheel)
    {
        if (!reader.required("wheel_radius", config.wheel_radius) ||
            !std::isfinite(config.wheel_radius) || config.wheel_radius <= 0.0 ||
            !signs || !signs.IsSequence())
        {
            result.error_message = "wheel model requires positive radius and velocity signs";
            return result;
        }
        for (std::size_t i = 0; i < model.joint_count; ++i)
        {
            if (model.joints[i].role == core::JointRole::Wheel)
            {
                ++config.wheel_count;
            }
        }
        if (signs.size() != config.wheel_count)
        {
            result.error_message = "wheel_velocity_sign count must match Wheel joints";
            return result;
        }
        try
        {
            for (std::size_t i = 0; i < config.wheel_count; ++i)
            {
                config.wheel_velocity_sign[i] = signs[i].as<double>();
                if (!std::isfinite(config.wheel_velocity_sign[i]) ||
                    config.wheel_velocity_sign[i] == 0.0)
                {
                    result.error_message = "wheel_velocity_sign values must be finite and nonzero";
                    return result;
                }
            }
        }
        catch (const YAML::Exception& exception)
        {
            result.error_message = "invalid wheel_velocity_sign: " +
                std::string(exception.what());
            return result;
        }
    }
    else if ((root["wheel_radius"] && !root["wheel_radius"].IsNull()) ||
        (signs && (!signs.IsSequence() || signs.size() != 0)))
    {
        result.error_message = "RobotModel without Wheel joints must not define wheel parameters";
        return result;
    }

    config.event_count = events.size();
    for (std::size_t i = 0; i < config.event_count; ++i)
    {
        if (!read_event(events[i], model, config.events[i], result.error_message))
        {
            result.error_message = "event[" + std::to_string(i) + "]: " + result.error_message;
            return result;
        }
        const bool uses_pose = config.events[i].type == motion::EventType::Pose ||
            config.events[i].type == motion::EventType::PoseDrive;
        if (uses_pose)
        {
            for (std::size_t joint_index = 0; joint_index < model.joint_count; ++joint_index)
            {
                const double position = config.events[i].dof_positions[joint_index];
                const auto& limits = model.joints[joint_index].limits;
                if (!std::isfinite(position) ||
                    (limits.position_limited &&
                        (position < limits.min_position || position > limits.max_position)))
                {
                    result.error_message = "event[" + std::to_string(i) +
                        "] pose is invalid at joint index " + std::to_string(joint_index);
                    return result;
                }
            }
        }
        const auto type = config.events[i].type;
        if (!has_wheel && (type == motion::EventType::Drive ||
                              type == motion::EventType::PoseDrive))
        {
            result.error_message = "event[" + std::to_string(i) +
                "] requires Wheel joints unavailable in RobotModel";
            return result;
        }
    }
    return result;
}

}  // 命名空间 quadruped::config
