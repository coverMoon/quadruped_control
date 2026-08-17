/**
 * @file config_test_helpers.hpp
 * @brief 配置测试共用的临时文件写入、模板配置和拒绝断言辅助函数。
 */

#pragma once

#include "quadruped/config/robot_config.hpp"
#include "quadruped/core/core.hpp"

#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace qc = quadruped::core;

namespace config_test
{

// 简单测试程序累计失败断言，全部用例运行后统一返回非零退出码。
inline int failures = 0;

inline void expect(bool condition, const std::string& description)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << description << '\n';
        ++failures;
    }
}

// 把测试 YAML 内容写入临时文件；每次调用使用不同文件名，返回完整路径。
inline std::string write_temp_config(const std::string& name, const std::string& content)
{
    const auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream file(path);
    file << content;
    return path.string();
}

// 一份最小合法的 RobotModel YAML，各用例在此基础上只修改待验证部分。
inline std::string make_robot_yaml()
{
    std::string yaml =
        "name: test_quadruped\n"
        "model_id: 0x1234\n"
        "calibration_id: 0\n"
        "joints:\n";
    constexpr const char* names[2] = {"joint_a", "joint_b"};
    for (const char* name : names)
    {
        yaml += "  - {name: ";
        yaml += name;
        yaml +=
            ", role: leg, position_limited: true, min_position: -3.0, max_position: 3.0,"
            " max_velocity: 20.0, max_effort: 40.0, max_kp: 100.0, max_kd: 10.0}\n";
    }
    return yaml;
}

// 一份与 make_robot_yaml 匹配的最小合法控制器配置 YAML。
inline std::string make_controller_yaml()
{
    return
        "name: test_quadruped\n"
        "control_period_ms: 5\n"
        "command_validity_ms: 10\n"
        "getup_pre_cycles: 100\n"
        "getup_cycles: 100\n"
        "getdown_cycles: 500\n"
        "joint_names: [joint_a, joint_b]\n"
        "pre_getup_position: [0.0, 1.4]\n"
        "stand_position: [0.0, 0.82]\n"
        "fixed_kp: [80.0, 80.0]\n"
        "fixed_kd: [3.0, 3.0]\n";
}

inline qc::RobotModel load_test_model()
{
    const auto result = quadruped::config::load_robot_model(
        write_temp_config("qc_test_robot.yaml", make_robot_yaml()));
    expect(result.ok(), "最小合法机器人配置应加载成功");
    return result.model;
}

// 一次“替换模板并拒绝加载”用例的全部输入。
struct RejectionCase
{
    const char* file;
    const char* from;
    const char* to;
    const char* description;
};

// 用替换生成非法配置，验证加载被拒绝且错误信息可读；
// 加载接口承诺不抛异常，非法配置必须以 error_message 返回。
inline void expect_robot_rejected(const RejectionCase& c)
{
    std::string yaml = make_robot_yaml();
    const auto pos = yaml.find(c.from);
    expect(pos != std::string::npos,
        std::string(c.description) + "（测试配置模板缺少待替换内容）");
    if (pos != std::string::npos)
    {
        yaml.replace(pos, std::strlen(c.from), c.to);
    }
    try
    {
        const auto result =
            quadruped::config::load_robot_model(write_temp_config(c.file, yaml));
        expect(!result.ok(), c.description);
        expect(!result.error_message.empty(),
            std::string(c.description) + "（应给出可读原因）");
    }
    catch (const std::exception& e)
    {
        expect(false, std::string(c.description) + "（异常越过模块边界：" + e.what() + "）");
    }
}

inline void expect_controller_rejected(const qc::RobotModel& model, const RejectionCase& c)
{
    std::string yaml = make_controller_yaml();
    const auto pos = yaml.find(c.from);
    expect(pos != std::string::npos,
        std::string(c.description) + "（测试配置模板缺少待替换内容）");
    if (pos != std::string::npos)
    {
        yaml.replace(pos, std::strlen(c.from), c.to);
    }
    try
    {
        const auto result = quadruped::config::load_controller_config(
            write_temp_config(c.file, yaml), model);
        expect(!result.ok(), c.description);
        expect(!result.error_message.empty(),
            std::string(c.description) + "（应给出可读原因）");
    }
    catch (const std::exception& e)
    {
        expect(false, std::string(c.description) + "（异常越过模块边界：" + e.what() + "）");
    }
}

// 非法根节点回归用例。
struct RootCase
{
    const char* file;
    const char* content;
    const char* description;
};

// 根节点不是映射时加载接口应拒绝并返回错误，而不是抛出 YAML 异常。
inline void expect_robot_root_rejected(const RootCase& c)
{
    const auto path = write_temp_config(c.file, c.content);
    try
    {
        const auto result = quadruped::config::load_robot_model(path);
        expect(!result.ok(), c.description);
    }
    catch (const std::exception& e)
    {
        expect(false, std::string(c.description) + "（异常越过模块边界：" + e.what() + "）");
    }
}

inline void expect_controller_root_rejected(const qc::RobotModel& model, const RootCase& c)
{
    const auto path = write_temp_config(c.file, c.content);
    try
    {
        const auto result = quadruped::config::load_controller_config(path, model);
        expect(!result.ok(), c.description);
    }
    catch (const std::exception& e)
    {
        expect(false, std::string(c.description) + "（异常越过模块边界：" + e.what() + "）");
    }
}

}  // namespace config_test
