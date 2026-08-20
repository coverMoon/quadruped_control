/**
 * @file rl_config_loader.cpp
 * @brief 实现 black RL 策略所需最小 YAML 配置加载。
 */

#include "quadruped/config/rl_config_loader.hpp"

#include "yaml_helpers.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>

namespace quadruped::config
{
namespace
{

bool read_config(detail::FieldReader& reader, motion::RlConfig& config)
{
    return reader.required("name", config.name) &&
        reader.required("robot_name", config.robot_name) &&
        reader.required("model_path", config.model_path) &&
        reader.double_array("command_scale", config.command_scale.data(), 3) &&
        reader.double_array("command_limits", config.command_limits.data(), 3) &&
        reader.required("angular_velocity_scale", config.angular_velocity_scale) &&
        reader.required("joint_position_scale", config.joint_position_scale) &&
        reader.required("joint_velocity_scale", config.joint_velocity_scale) &&
        reader.required("observation_clip", config.observation_clip) &&
        reader.double_array("default_joint_positions",
            config.default_joint_positions.data(), motion::kRlJointCount) &&
        reader.double_array("kp", config.kp.data(), motion::kRlJointCount) &&
        reader.double_array("kd", config.kd.data(), motion::kRlJointCount) &&
        reader.required("action_scale", config.action_scale) &&
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
    if (!read_config(reader, result.config))
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
