/**
 * @file policy_switch_loader.cpp
 * @brief 实现按循环顺序加载策略切换 YAML 配置。
 */

#include "quadruped/config/policy_switch_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>

namespace quadruped::config
{
namespace
{

bool valid_policy_name(const std::string& name)
{
    if (name.empty())
    {
        return false;
    }
    for (const unsigned char character : name)
    {
        if (!std::isalnum(character) && character != '_' && character != '-')
        {
            return false;
        }
    }
    return true;
}

YAML::Node select_robot_config(const YAML::Node& root, const std::string& robot_name)
{
    if (root[robot_name] && root[robot_name].IsMap())
    {
        return root[robot_name];
    }
    return root;
}

}  // 匿名命名空间

PolicySwitchLoadResult load_policy_switch_config(
    const std::string& path,
    const std::string& robot_name,
    const std::string& policy_root)
{
    PolicySwitchLoadResult result;
    const std::filesystem::path policy_root_path(policy_root);
    if (robot_name.empty())
    {
        result.error_message = "policy switch robot name is empty";
        return result;
    }
    if (policy_root.empty() || !policy_root_path.is_absolute())
    {
        result.error_message = "policy switch policy_root must be an absolute path";
        return result;
    }

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const YAML::Exception& error)
    {
        result.error_message = "cannot load policy switch config \"" + path + "\": " + error.what();
        return result;
    }
    if (!root.IsMap())
    {
        result.error_message = "policy switch config root must be a mapping";
        return result;
    }

    const YAML::Node config = select_robot_config(root, robot_name);
    if (!config.IsMap())
    {
        result.error_message = "policy switch config for robot must be a mapping";
        return result;
    }

    if (config["posture_transition_cycles"])
    {
        try
        {
            const int cycles = config["posture_transition_cycles"].as<int>();
            if (cycles <= 0)
            {
                result.error_message = "posture_transition_cycles must be positive";
                return result;
            }
            result.config.posture_transition_cycles = static_cast<std::uint32_t>(cycles);
        }
        catch (const YAML::Exception& error)
        {
            result.error_message = "invalid posture_transition_cycles: " + std::string(error.what());
            return result;
        }
    }

    const YAML::Node cycle = config["policy_config_cycle"];
    if (!cycle || !cycle.IsSequence())
    {
        result.error_message =
            "policy switch config missing sequence field 'policy_config_cycle'";
        return result;
    }

    for (const YAML::Node& node : cycle)
    {
        std::string policy_name;
        try
        {
            policy_name = node.as<std::string>();
        }
        catch (const YAML::Exception& error)
        {
            result.warnings.emplace_back(
                "ignoring non-string policy_config_cycle item: " + std::string(error.what()));
            continue;
        }
        if (!valid_policy_name(policy_name))
        {
            result.warnings.emplace_back(
                "ignoring invalid policy_config_cycle item: " + policy_name);
            continue;
        }
        if (std::find(
                result.config.policy_names.begin(), result.config.policy_names.end(), policy_name) !=
            result.config.policy_names.end())
        {
            result.warnings.emplace_back("ignoring duplicate policy_config_cycle item: " + policy_name);
            continue;
        }

        const std::filesystem::path policy_path = policy_root_path / (policy_name + ".yaml");
        if (!std::filesystem::is_regular_file(policy_path))
        {
            result.warnings.emplace_back(
                "ignoring unavailable policy config: " + policy_path.string());
            continue;
        }
        result.config.policy_names.push_back(policy_name);
    }

    if (result.config.policy_names.empty())
    {
        result.error_message = "policy switch config has no available policy config";
    }
    return result;
}

}  // namespace quadruped::config
