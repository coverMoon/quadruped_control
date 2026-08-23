/**
 * @file config_tests.cpp
 * @brief 验证仓库 black 机器人和控制器配置可以加载并匹配。
 */

#include "quadruped/config/robot_config.hpp"
#include "quadruped/config/policy_switch_loader.hpp"
#include "quadruped/config/simulation_config.hpp"

#include <iostream>
#include <filesystem>
#include <string>

namespace
{

int failures = 0;

void expect(const bool condition, const std::string& message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

}  // namespace

int main()
{
    const auto robot = quadruped::config::load_robot_model(QUADRUPED_ROBOT_CONFIG_PATH);
    expect(robot.ok(), "black RobotModel 应加载成功：" + robot.error_message);
    if (robot.ok())
    {
        expect(robot.model.name == "black", "机器人名称应为 black");
        expect(robot.model.joint_count == 12, "black 应有 12 个有序关节");
        expect(robot.model.joints.front().name == "FL_hip_joint", "首关节顺序应正确");
        expect(robot.model.joints[11].name == "RR_calf_joint", "末关节顺序应正确");
        const auto controller = quadruped::config::load_controller_config(
            QUADRUPED_CONTROLLER_CONFIG_PATH, robot.model);
        expect(controller.ok(), "black ControllerConfig 应加载成功：" + controller.error_message);
        if (controller.ok())
        {
            expect(controller.config.control_period_ns == 5'000'000,
                "控制周期应为 5 ms");
        }
    }
    expect(!quadruped::config::load_robot_model("/missing/quadruped.yaml").ok(),
        "缺失配置文件应拒绝");

    const std::filesystem::path policy_switch_path(QUADRUPED_POLICY_SWITCH_CONFIG_PATH);
    const auto policy_switch = quadruped::config::load_policy_switch_config(
        policy_switch_path.string(), "black", policy_switch_path.parent_path().string());
    expect(policy_switch.ok(), "策略循环配置应加载成功：" + policy_switch.error_message);
    if (policy_switch.ok())
    {
        expect(policy_switch.config.policy_names.size() == 2,
            "black 策略循环应包含两个策略");
        expect(policy_switch.config.policy_names[0] == "flat",
            "策略循环第一项应为 flat");
        expect(policy_switch.config.policy_names[1] == "obstacle",
            "策略循环第二项应为 obstacle");
        expect(policy_switch.config.posture_transition_cycles == 150,
            "策略姿态过渡周期应从 policy_switch.yaml 加载");
    }

    const auto simulation =
        quadruped::config::load_simulation_config(QUADRUPED_SIMULATION_CONFIG_PATH);
    expect(simulation.ok(), "MuJoCo 仿真配置应加载成功：" + simulation.error_message);
    if (simulation.ok())
    {
        expect(simulation.config.real_time_factor == 1.0, "默认仿真应按实时倍率运行");
        expect(simulation.config.visual_sync_hz == 60.0, "默认画面状态同步应为 60 Hz");
        expect(simulation.config.vsync, "默认应启用垂直同步");
    }

    if (failures != 0)
    {
        std::cerr << failures << " 个配置测试失败\n";
        return 1;
    }
    std::cout << "配置加载测试通过\n";
    return 0;
}
