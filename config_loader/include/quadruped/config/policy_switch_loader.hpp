/**
 * @file policy_switch_loader.hpp
 * @brief 声明按循环顺序加载策略切换配置的接口。
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace quadruped::config
{

struct PolicySwitchConfig
{
    std::vector<std::string> policy_names{};
    std::uint32_t posture_transition_cycles{20};
};

struct PolicySwitchLoadResult
{
    PolicySwitchConfig config{};
    std::vector<std::string> warnings{};
    std::string error_message{};

    [[nodiscard]] bool ok() const noexcept
    {
        return error_message.empty();
    }
};

// policy_root 必须是绝对路径；循环项对应 policy_root/<name>.yaml。
[[nodiscard]] PolicySwitchLoadResult load_policy_switch_config(
    const std::string& path,
    const std::string& robot_name,
    const std::string& policy_root);

}  // namespace quadruped::config
