/**
 * @file simulation_config_loader.cpp
 * @brief 实现交互式 MuJoCo 仿真运行参数的 YAML 加载和范围校验。
 */

#include "quadruped/config/simulation_config.hpp"

#include "yaml_helpers.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <string>

namespace quadruped::config
{
namespace
{

inline constexpr double kMinRealTimeFactor = 0.01;
inline constexpr double kMaxRealTimeFactor = 100.0;
inline constexpr double kMinVisualSyncHz = 1.0;
inline constexpr double kMaxVisualSyncHz = 1000.0;

}  // 匿名命名空间

SimulationConfigLoadResult load_simulation_config(const std::string& path)
{
    SimulationConfigLoadResult result;
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const YAML::Exception& error)
    {
        result.error_message =
            "cannot load simulation config file \"" + path + "\": " + error.what();
        return result;
    }
    if (!root.IsMap())
    {
        result.error_message = "simulation config root must be a mapping";
        return result;
    }

    detail::FieldReader reader(root, result.error_message);
    auto& config = result.config;
    if (!reader.required("real_time_factor", config.real_time_factor) ||
        !reader.required("visual_sync_hz", config.visual_sync_hz) ||
        !reader.required("vsync", config.vsync))
    {
        return result;
    }
    if (!std::isfinite(config.real_time_factor) ||
        config.real_time_factor < kMinRealTimeFactor ||
        config.real_time_factor > kMaxRealTimeFactor)
    {
        result.error_message = "key \"real_time_factor\" must be finite and in [0.01, 100]";
        return result;
    }
    if (!std::isfinite(config.visual_sync_hz) ||
        config.visual_sync_hz < kMinVisualSyncHz ||
        config.visual_sync_hz > kMaxVisualSyncHz)
    {
        result.error_message = "key \"visual_sync_hz\" must be finite and in [1, 1000]";
        return result;
    }
    return result;
}

}  // 命名空间 quadruped::config
