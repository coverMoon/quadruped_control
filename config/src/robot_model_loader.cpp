/**
 * @file robot_model_loader.cpp
 * @brief 实现 RobotModel 的 YAML 启动期加载和核心校验。
 */

#include "quadruped/config/robot_config.hpp"

#include "yaml_helpers.hpp"

#include "quadruped/core/validation.hpp"

#include <yaml-cpp/yaml.h>

#include <string>

namespace quadruped::config
{
namespace
{

// 把 YAML 中的 role 字符串映射为核心关节角色；未知角色明确拒绝。
bool parse_joint_role(const std::string& text, core::JointRole& role)
{
    if (text == "leg")
    {
        role = core::JointRole::Leg;
        return true;
    }
    if (text == "wheel")
    {
        role = core::JointRole::Wheel;
        return true;
    }
    if (text == "auxiliary")
    {
        role = core::JointRole::Auxiliary;
        return true;
    }
    return false;
}

// 加载单个关节条目；任何字段缺失或非法都会给出带关节下标的错误说明。
bool load_joint(
    detail::FieldReader& reader,
    const std::size_t index,
    core::JointDescription& joint)
{
    std::string role_text;
    if (!reader.required("name", joint.name) ||
        !reader.required("role", role_text) ||
        !reader.required("position_limited", joint.limits.position_limited) ||
        !reader.required("min_position", joint.limits.min_position) ||
        !reader.required("max_position", joint.limits.max_position) ||
        !reader.required("max_velocity", joint.limits.max_velocity) ||
        !reader.required("max_effort", joint.limits.max_effort) ||
        !reader.required("max_kp", joint.limits.max_kp) ||
        !reader.required("max_kd", joint.limits.max_kd))
    {
        return reader.fail("joints[" + std::to_string(index) + "]: " + reader.error_message());
    }
    if (!parse_joint_role(role_text, joint.role))
    {
        return reader.fail("joints[" + std::to_string(index) + "]: unknown joint role \"" +
            role_text + "\"");
    }
    return true;
}

// 读取并逐项校验 joints 序列；失败时写出可读错误。
bool load_joints(
    const YAML::Node& root,
    core::RobotModel& model,
    std::string& error)
{
    const YAML::Node joints = root["joints"];
    if (!joints || !joints.IsSequence() || joints.size() == 0 ||
        joints.size() > core::kMaxJoints)
    {
        error = "\"joints\" must be a non-empty sequence with at most " +
            std::to_string(core::kMaxJoints) + " entries";
        return false;
    }
    model.joint_count = joints.size();
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        if (!joints[i].IsMap())
        {
            error = "joints[" + std::to_string(i) + "]: joint entry must be a mapping";
            return false;
        }
        detail::FieldReader joint_reader(joints[i], error);
        if (!load_joint(joint_reader, i, model.joints[i]))
        {
            return false;
        }
    }
    return true;
}

}  // 匿名命名空间

RobotModelLoadResult load_robot_model(const std::string& path)
{
    RobotModelLoadResult result;

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const YAML::Exception& e)
    {
        result.error_message = "cannot load robot model file \"" + path + "\": " + e.what();
        return result;
    }
    // 根节点必须是映射；标量或序列根节点会让后续按键索引抛异常。
    if (!root.IsMap())
    {
        result.error_message = "robot config root must be a mapping";
        return result;
    }

    auto& model = result.model;
    std::string& error = result.error_message;
    detail::FieldReader reader(root, error);
    if (!reader.required("name", model.name) ||
        !reader.uint64("model_id", model.model_id) ||
        !reader.uint64("calibration_id", model.calibration_id) ||
        !load_joints(root, model, error))
    {
        return result;
    }

    if (const auto validation = core::validate(model); !validation)
    {
        error = "robot model validation failed: " + validation.message;
        return result;
    }
    return result;
}

}  // 命名空间 quadruped::config
