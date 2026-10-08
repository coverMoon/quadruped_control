/**
 * @file config_tests.cpp
 * @brief 验证仓库 black/blackW 机器人、控制器和运行配置可以加载并匹配。
 */

#include "quadruped/config/behavior_config_loader.hpp"
#include "quadruped/config/policy_switch_loader.hpp"
#include "quadruped/config/robot_config.hpp"
#include "quadruped/config/simulation_config.hpp"

#include <filesystem>
#include <iostream>
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
        const auto retry = quadruped::config::load_retry_config(
            QUADRUPED_RETRY_CONFIG_PATH, robot.model);
        expect(retry.ok(), "black Retry 配置应加载成功：" + retry.error_message);
        const auto event_chain = quadruped::config::load_event_chain_config(
            QUADRUPED_EVENT_CHAIN_CONFIG_PATH, robot.model);
        expect(!event_chain.ok(), "black 空 Event chain 必须在启动期拒绝");
    }
    expect(!quadruped::config::load_robot_model("/missing/quadruped.yaml").ok(),
        "缺失配置文件应拒绝");

    const auto blackw =
        quadruped::config::load_robot_model(QUADRUPED_BLACKW_ROBOT_CONFIG_PATH);
    expect(blackw.ok(), "blackW RobotModel 应加载成功：" + blackw.error_message);
    if (blackw.ok())
    {
        expect(blackw.model.name == "blackW", "机器人名称应为 blackW");
        expect(blackw.model.joint_count == 16, "blackW 应有 16 个有序关节");
        expect(blackw.model.joints[3].name == "FL_wheel_joint",
            "FL 轮关节应位于逻辑下标 3");
        expect(blackw.model.joints[8].name == "RL_hip_joint",
            "逻辑顺序必须使用 RL 在 RR 前");
        expect(blackw.model.joints[15].name == "RR_wheel_joint",
            "末关节应为 RR_wheel_joint");
        for (const std::size_t index : {3U, 7U, 11U, 15U})
        {
            expect(blackw.model.joints[index].role == quadruped::core::JointRole::Wheel,
                "blackW 轮关节必须显式标记 Wheel role");
            expect(!blackw.model.joints[index].limits.position_limited,
                "blackW 轮关节必须显式声明无位置限制");
        }

        const auto blackw_controller = quadruped::config::load_controller_config(
            QUADRUPED_BLACKW_CONTROLLER_CONFIG_PATH, blackw.model);
        expect(blackw_controller.ok(),
            "blackW ControllerConfig 应加载成功：" + blackw_controller.error_message);
        if (blackw_controller.ok())
        {
            expect(blackw_controller.config.control_period_ns == 5'000'000,
                "blackW 控制周期应为 5 ms");
            expect(blackw_controller.config.fixed_kp[3] == 0.0,
                "blackW 轮关节基础 KP 应为零");
            expect(blackw_controller.config.fixed_kd[3] == 0.5,
                "blackW 轮关节基础 KD 应为 0.5");
        }
        const auto retry = quadruped::config::load_retry_config(
            QUADRUPED_BLACKW_RETRY_CONFIG_PATH, blackw.model);
        expect(retry.ok(), "blackW Retry 配置应加载成功：" + retry.error_message);
        const auto event_chain = quadruped::config::load_event_chain_config(
            QUADRUPED_BLACKW_EVENT_CHAIN_CONFIG_PATH, blackw.model);
        expect(event_chain.ok(),
            "blackW Event chain 应加载成功：" + event_chain.error_message);
        if (event_chain.ok())
        {
            expect(event_chain.config.event_count == 10,
                "blackW 应直接加载 rl_sar 的 10 个事件");
            expect(event_chain.config.events[0].name == "settle_at_wall" &&
                    event_chain.config.events[9].name == "restore_default_stand",
                "blackW Event chain 首尾事件应与 rl_sar 一致");
            expect(event_chain.config.events[3].type ==
                        quadruped::motion::EventType::Drive &&
                    event_chain.config.events[6].type ==
                        quadruped::motion::EventType::PoseDrive,
                "blackW Event chain 应保留 drive 和 pose_drive 类型");
        }
        constexpr const char* kFixedDrivePaths[] = {
            QUADRUPED_BLACKW_BRIDGE_CONFIG_PATH,
            QUADRUPED_BLACKW_LOW_BAR_CONFIG_PATH,
            QUADRUPED_BLACKW_CAR_CONFIG_PATH,
        };
        constexpr const char* kFixedDriveNames[] = {
            "bridge_drive", "low_bar_drive", "car_drive"};
        for (std::size_t i = 0; i < 3; ++i)
        {
            const auto fixed = quadruped::config::load_fixed_drive_config(
                kFixedDrivePaths[i], blackw.model);
            expect(fixed.ok(),
                std::string("blackW 固定姿态轮驱配置应加载成功：") +
                    fixed.error_message);
            if (fixed.ok())
            {
                expect(fixed.config.behavior_name == kFixedDriveNames[i] &&
                        fixed.config.prepare_cycles == 150 &&
                        fixed.config.exit_to_rl_cycles == 150,
                    "固定姿态轮驱名称和周期应与 rl_sar 一致");
                expect(fixed.config.wheel_sides[0] ==
                            quadruped::motion::WheelSide::Left &&
                        fixed.config.wheel_sides[1] ==
                            quadruped::motion::WheelSide::Right,
                    "固定姿态轮驱必须显式加载左右轮映射");
            }
        }
    }

    const std::filesystem::path policy_switch_path(QUADRUPED_POLICY_SWITCH_CONFIG_PATH);
    const auto policy_switch = quadruped::config::load_policy_switch_config(
        policy_switch_path.string(), "black", policy_switch_path.parent_path().string());
    expect(policy_switch.ok(), "策略循环配置应加载成功：" + policy_switch.error_message);
    if (policy_switch.ok())
    {
        expect(policy_switch.config.policy_names.size() == 3,
            "black 策略循环应包含 flat、obstacle 和 test");
        expect(policy_switch.config.policy_names[0] == "flat",
            "策略循环第一项应为 flat");
        expect(policy_switch.config.policy_names[1] == "obstacle",
            "策略循环第二项应为 obstacle");
        if (policy_switch.config.policy_names.size() >= 3)
        {
            expect(policy_switch.config.policy_names[2] == "test",
                "策略循环第三项应为 test");
        }
        expect(policy_switch.config.posture_transition_cycles == 150,
            "策略姿态过渡周期应从 policy_switch.yaml 加载");
    }

    const std::filesystem::path blackw_policy_switch_path(
        QUADRUPED_BLACKW_POLICY_SWITCH_CONFIG_PATH);
    const auto blackw_policy_switch = quadruped::config::load_policy_switch_config(
        blackw_policy_switch_path.string(),
        "blackW",
        blackw_policy_switch_path.parent_path().string());
    expect(blackw_policy_switch.ok(),
        "blackW 策略循环配置应加载成功：" + blackw_policy_switch.error_message);
    if (blackw_policy_switch.ok())
    {
        expect(blackw_policy_switch.config.policy_names.size() == 3,
            "blackW 策略循环应包含 flat、obstacle 和 stair");
        expect(blackw_policy_switch.config.posture_transition_cycles == 200,
            "blackW 策略姿态过渡应为 200 周期");
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

    const auto blackw_simulation =
        quadruped::config::load_simulation_config(QUADRUPED_BLACKW_SIMULATION_CONFIG_PATH);
    expect(blackw_simulation.ok(),
        "blackW MuJoCo 仿真配置应加载成功：" + blackw_simulation.error_message);

    if (failures != 0)
    {
        std::cerr << failures << " 个配置测试失败\n";
        return 1;
    }
    std::cout << "配置加载测试通过\n";
    return 0;
}
