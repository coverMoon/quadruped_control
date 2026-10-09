/**
 * @file rl_config_loader.cpp
 * @brief 实现 black/blackW RL 固定容量配置加载和显式关节映射校验。
 */

#include "quadruped/config/rl_config_loader.hpp"

#include "yaml_helpers.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <filesystem>
#include <string>

namespace quadruped::config
{
namespace
{

bool read_size(
    detail::FieldReader& reader,
    const char* key,
    std::size_t& output,
    const std::size_t maximum)
{
    std::int64_t value = 0;
    if (!reader.required(key, value))
    {
        return false;
    }
    if (value <= 0 || static_cast<std::uint64_t>(value) > maximum)
    {
        return reader.fail(std::string("key \"") + key + "\" is out of range");
    }
    output = static_cast<std::size_t>(value);
    return true;
}

bool read_index_sequence(
    const YAML::Node& root,
    const char* key,
    std::size_t* output,
    const std::size_t expected_size,
    std::string& error)
{
    const YAML::Node values = root[key];
    if (!values || !values.IsSequence() || values.size() != expected_size)
    {
        error = std::string("key \"") + key + "\" must contain " +
            std::to_string(expected_size) + " indices";
        return false;
    }
    try
    {
        for (std::size_t i = 0; i < expected_size; ++i)
        {
            const std::int64_t value = values[i].as<std::int64_t>();
            if (value < 0)
            {
                error = std::string("key \"") + key + "\" contains a negative index";
                return false;
            }
            output[i] = static_cast<std::size_t>(value);
        }
    }
    catch (const YAML::Exception& exception)
    {
        error = std::string("invalid index in key \"") + key + "\": " + exception.what();
        return false;
    }
    return true;
}

bool read_joint_names(
    const YAML::Node& root,
    const core::RobotModel& model,
    motion::RlConfig& config,
    std::string& error)
{
    const YAML::Node values = root["joint_names"];
    if (!values || !values.IsSequence() || values.size() != model.joint_count)
    {
        error = "joint_names must match the RobotModel joint count";
        return false;
    }
    try
    {
        for (std::size_t i = 0; i < model.joint_count; ++i)
        {
            config.joint_names[i] = values[i].as<std::string>();
        }
    }
    catch (const YAML::Exception& exception)
    {
        error = "invalid joint_names: " + std::string(exception.what());
        return false;
    }
    return true;
}

bool read_observation_layout(
    const YAML::Node& root,
    motion::RlObservationLayout& layout,
    std::string& error)
{
    const YAML::Node value = root["observation_layout"];
    if (!value)
    {
        // 缺省保持 black / blackW 原有布局，旧 YAML 无需新增字段。
        layout = motion::RlObservationLayout::AllJoints;
        return true;
    }
    std::string text;
    try
    {
        text = value.as<std::string>();
    }
    catch (const YAML::Exception& exception)
    {
        error = "invalid observation_layout: " + std::string(exception.what());
        return false;
    }
    if (text == "all_joints")
    {
        layout = motion::RlObservationLayout::AllJoints;
        return true;
    }
    if (text == "leg_wheel_split_v1")
    {
        layout = motion::RlObservationLayout::LegWheelSplit;
        return true;
    }
    error = "unsupported observation_layout: " + text;
    return false;
}

bool read_observation_order(
    const YAML::Node& root,
    const motion::RlObservationLayout layout,
    std::string& error)
{
    constexpr const char* kAllJoints[6] = {
        "commands", "angular_velocity", "projected_gravity",
        "joint_position_error", "joint_velocity", "previous_action"};
    constexpr const char* kLegWheelSplit[7] = {
        "commands", "angular_velocity", "projected_gravity",
        "joint_position_error", "joint_velocity", "wheel_velocity", "previous_action"};
    const bool split = layout == motion::RlObservationLayout::LegWheelSplit;
    const char* const* expected = split ? kLegWheelSplit : kAllJoints;
    const std::size_t expected_size = split ? 7U : 6U;
    const YAML::Node values = root["observation_order"];
    if (!values || !values.IsSequence() || values.size() != expected_size)
    {
        error = std::string("observation_order must contain the ") +
            std::to_string(expected_size) + " fields of the configured layout";
        return false;
    }
    try
    {
        for (std::size_t i = 0; i < expected_size; ++i)
        {
            if (values[i].as<std::string>() != expected[i])
            {
                error = "observation_order does not match the supported RL layout";
                return false;
            }
        }
    }
    catch (const YAML::Exception& exception)
    {
        error = "invalid observation_order: " + std::string(exception.what());
        return false;
    }
    return true;
}

// 腿轮分离布局额外的轮速比例和 forward sign；其他布局不读取这两个字段。
bool read_split_layout_fields(
    detail::FieldReader& reader,
    motion::RlConfig& config)
{
    if (config.observation_layout != motion::RlObservationLayout::LegWheelSplit)
    {
        return true;
    }
    return reader.required("wheel_velocity_scale", config.wheel_velocity_scale) &&
        reader.double_array(
            "wheel_velocity_signs", config.wheel_velocity_signs.data(), config.wheel_count);
}

bool read_action_modes(detail::FieldReader& reader, motion::RlConfig& config)
{
    std::string leg_mode;
    std::string wheel_mode;
    if (!reader.required("leg_action_mode", leg_mode) ||
        !reader.required("wheel_action_mode", wheel_mode))
    {
        return false;
    }
    if (leg_mode != "position_residual" || wheel_mode != "target_velocity")
    {
        return reader.fail("unsupported leg_action_mode or wheel_action_mode");
    }
    config.leg_action_mode = motion::RlJointActionMode::PositionResidual;
    config.wheel_action_mode = motion::RlJointActionMode::TargetVelocity;
    return true;
}

bool read_config(
    detail::FieldReader& reader,
    const core::RobotModel& model,
    motion::RlConfig& config)
{
    config.joint_count = model.joint_count;
    if (!reader.required("name", config.name) ||
        !reader.required("robot_name", config.robot_name) ||
        !reader.required("model_path", config.model_path))
    {
        return false;
    }
    if (!read_size(reader, "observation_dimension", config.observation_dimension,
            motion::kMaxRlObservationDim) ||
        !read_size(reader, "inference_input_dimension", config.inference_input_dimension,
            motion::kMaxRlInputDim) ||
        !read_size(reader, "action_dimension", config.action_dimension,
            motion::kMaxRlActionDim))
    {
        return false;
    }
    const YAML::Node root = reader.node();
    const YAML::Node history = root["history_frames"];
    if (!history || !history.IsSequence() || history.size() == 0 ||
        history.size() > motion::kMaxRlHistoryFrames)
    {
        return reader.fail("history_frames must be a nonempty bounded sequence");
    }
    config.history_frame_count = history.size();
    std::size_t model_wheel_count = 0;
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        model_wheel_count += model.joints[i].role == core::JointRole::Wheel ? 1U : 0U;
    }
    config.wheel_count = model_wheel_count;
    if (!read_observation_layout(root, config.observation_layout, reader.error_message()) ||
        !read_index_sequence(root, "history_frames", config.history_frames.data(),
            config.history_frame_count, reader.error_message()) ||
        !read_joint_names(root, model, config, reader.error_message()) ||
        !read_index_sequence(root, "policy_dof_indices", config.policy_dof_indices.data(),
            config.action_dimension, reader.error_message()) ||
        !read_index_sequence(root, "wheel_indices", config.wheel_indices.data(),
            config.wheel_count, reader.error_message()) ||
        !read_observation_order(root, config.observation_layout, reader.error_message()) ||
        !read_action_modes(reader, config) ||
        !read_split_layout_fields(reader, config))
    {
        return false;
    }
    return reader.double_array("command_scale", config.command_scale.data(), 3) &&
        reader.double_array("command_limits", config.command_limits.data(), 3) &&
        reader.required("angular_velocity_scale", config.angular_velocity_scale) &&
        reader.required("joint_position_scale", config.joint_position_scale) &&
        reader.required("joint_velocity_scale", config.joint_velocity_scale) &&
        reader.required("observation_clip", config.observation_clip) &&
        reader.double_array("default_joint_positions",
            config.default_joint_positions.data(), config.action_dimension) &&
        reader.double_array("kp", config.kp.data(), config.action_dimension) &&
        reader.double_array("kd", config.kd.data(), config.action_dimension) &&
        reader.double_array("action_scale", config.action_scale.data(), config.action_dimension) &&
        reader.required("action_clip", config.action_clip) &&
        reader.required("max_position_jump", config.max_position_jump);
}

}  // namespace

RlConfigLoadResult load_rl_config(
    const std::string& path,
    const std::string& asset_root,
    const core::RobotModel& model)
{
    RlConfigLoadResult result;
    const std::filesystem::path root_path(asset_root);
    if (asset_root.empty() || !root_path.is_absolute())
    {
        result.error_message = "RL asset_root must be an absolute path";
        return result;
    }

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const YAML::Exception& error)
    {
        result.error_message = "cannot load RL config \"" + path + "\": " + error.what();
        return result;
    }
    if (!root.IsMap())
    {
        result.error_message = "RL config root must be a mapping";
        return result;
    }

    detail::FieldReader reader(root, result.error_message);
    if (!read_config(reader, model, result.config))
    {
        return result;
    }
    const std::filesystem::path model_path(result.config.model_path);
    result.config.model_path =
        (model_path.is_absolute() ? model_path : root_path / model_path).string();
    if (!motion::validate_rl_config(result.config, model, result.error_message))
    {
        result.error_message = "invalid RL config: " + result.error_message;
    }
    return result;
}

}  // namespace quadruped::config
