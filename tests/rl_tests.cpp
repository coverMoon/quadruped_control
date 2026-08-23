/**
 * @file rl_tests.cpp
 * @brief 用少量参考向量验证 black RL 配置、观测、历史和动作换算。
 */

#include "quadruped/config/rl_config_loader.hpp"
#include "quadruped/config/robot_config.hpp"
#include "quadruped/motion/rl_controller.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
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

void expect_close(const double actual, const double expected, const std::string& message)
{
    expect(std::isfinite(actual) && std::abs(actual - expected) <= 1.0e-6, message);
}

quadruped::core::StateFrame make_state(
    const quadruped::core::RobotModel& model,
    const quadruped::motion::RlConfig& config)
{
    quadruped::core::StateFrame state;
    state.header.startup_id = 1;
    state.header.session_id = 1;
    state.header.sequence = 1;
    state.joint_count = model.joint_count;
    state.safety_state = quadruped::core::SafetyState::ControlEnabled;
    state.imu.valid = true;
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        state.joints[i].position = config.default_joint_positions[i];
        state.joints[i].online = true;
        state.joints[i].valid = true;
    }
    return state;
}

}  // namespace

int main()
{
    const auto model = quadruped::config::load_robot_model(QUADRUPED_ROBOT_CONFIG_PATH);
    expect(model.ok(), "black RobotModel 应加载成功");
    if (!model.ok())
    {
        return 1;
    }
    const auto flat = quadruped::config::load_rl_config(
        QUADRUPED_POLICY_FLAT_CONFIG_PATH, QUADRUPED_PROJECT_SOURCE_DIR, model.model);
    const auto obstacle = quadruped::config::load_rl_config(
        QUADRUPED_POLICY_OBSTACLE_CONFIG_PATH, QUADRUPED_PROJECT_SOURCE_DIR, model.model);
    expect(flat.ok(), "flat RL 配置应加载成功：" + flat.error_message);
    expect(obstacle.ok(), "obstacle RL 配置应加载成功：" + obstacle.error_message);
    expect(!quadruped::config::load_rl_config(
        QUADRUPED_POLICY_FLAT_CONFIG_PATH, "relative", model.model).ok(),
        "相对资产根目录应拒绝");
    if (!flat.ok() || !obstacle.ok())
    {
        return 1;
    }

    auto created = quadruped::motion::RlController::create(model.model, flat.config);
    expect(created.ok(), "RlController 应创建成功：" + created.error_message);
    if (!created.ok())
    {
        return 1;
    }

    auto state = make_state(model.model, flat.config);
    quadruped::core::BaseCommand command;
    command.sequence = 1;
    command.expires_at_ns = 10'000'000;
    command.source = quadruped::core::CommandSource::Test;
    command.vx = 1.0;
    command.vy = -2.0;
    command.wz = 4.0;
    created.controller->update_command(&command, 0);
    const auto observation = created.controller->build_observation(state, 0);
    expect(observation.ok, "固定状态观测应构造成功：" + observation.error_message);
    if (observation.ok)
    {
        expect_close(observation.observation[0], 2.0, "vx 应缩放");
        expect_close(observation.observation[1], -2.0, "vy 应先限幅再缩放");
        expect_close(observation.observation[2], 0.75, "wz 应先限幅再缩放");
        expect_close(observation.observation[8], -1.0, "单位姿态的投影重力应向下");
        expect_close(observation.observation[9], 0.0, "默认姿态相对位置应为零");
        expect(created.controller->insert_observation(observation),
            "合法观测应插入历史");
        const auto input = created.controller->inference_input();
        expect_close(input.observation[0], 2.0, "历史首帧应为最新观测");
        expect_close(input.observation[quadruped::motion::kRlObservationDim], 0.0,
            "未填充历史应保持零");
    }

    quadruped::motion::RlInferenceOutput inference;
    inference.ok = true;
    inference.action_dimension = flat.config.action_dimension;
    inference.actions.fill(1.0F);
    std::array<double, quadruped::core::kMaxJoints> positions{};
    std::copy(flat.config.default_joint_positions.begin(),
        flat.config.default_joint_positions.end(), positions.begin());
    const auto output = created.controller->convert_actions(inference, positions);
    expect(output.ok, "动作换算应成功：" + output.error_message);
    expect_close(output.target_positions[0],
        flat.config.default_joint_positions[0] + flat.config.action_scale[0],
        "动作目标应与 rl_sar 的 default + action * scale 一致");
    expect_close(output.kp[0], flat.config.kp[0], "应使用策略 KP");

    inference.actions[0] = std::numeric_limits<float>::quiet_NaN();
    expect(!created.controller->convert_actions(inference, positions).ok,
        "非有限动作应拒绝");

    const auto blackw_model = quadruped::config::load_robot_model(
        QUADRUPED_BLACKW_ROBOT_CONFIG_PATH);
    expect(blackw_model.ok(), "blackW RobotModel 应加载成功");
    if (blackw_model.ok())
    {
        const auto blackw_flat = quadruped::config::load_rl_config(
            QUADRUPED_BLACKW_POLICY_FLAT_CONFIG_PATH,
            QUADRUPED_PROJECT_SOURCE_DIR,
            blackw_model.model);
        const auto blackw_obstacle = quadruped::config::load_rl_config(
            QUADRUPED_BLACKW_POLICY_OBSTACLE_CONFIG_PATH,
            QUADRUPED_PROJECT_SOURCE_DIR,
            blackw_model.model);
        const auto blackw_stair = quadruped::config::load_rl_config(
            QUADRUPED_BLACKW_POLICY_STAIR_CONFIG_PATH,
            QUADRUPED_PROJECT_SOURCE_DIR,
            blackw_model.model);
        expect(blackw_flat.ok(),
            "blackW flat RL 配置应加载成功：" + blackw_flat.error_message);
        expect(blackw_obstacle.ok(),
            "blackW obstacle RL 配置应加载成功：" + blackw_obstacle.error_message);
        expect(blackw_stair.ok(),
            "blackW stair RL 配置应加载成功：" + blackw_stair.error_message);
        if (blackw_flat.ok())
        {
            auto blackw_controller = quadruped::motion::RlController::create(
                blackw_model.model, blackw_flat.config);
            expect(blackw_controller.ok(),
                "blackW RlController 应创建成功：" + blackw_controller.error_message);
            if (blackw_controller.ok())
            {
                auto blackw_state = make_state(blackw_model.model, blackw_flat.config);
                for (const std::size_t index : {3U, 7U, 11U, 15U})
                {
                    blackw_state.joints[index].position = 2.0 +
                        static_cast<double>(index);
                    blackw_state.joints[index].velocity = 1.0;
                }
                const auto blackw_observation =
                    blackw_controller.controller->build_observation(blackw_state, 0);
                expect(blackw_observation.ok && blackw_observation.dimension == 57,
                    "blackW 单帧观测应为 57 维");
                for (const std::size_t action_index : {3U, 7U, 11U, 15U})
                {
                    expect_close(blackw_observation.observation[9 + action_index],
                        0.0,
                        "Wheel 位置误差观测必须固定为零");
                    expect_close(blackw_observation.observation[25 + action_index],
                        0.05,
                        "Wheel 速度应按统一关节速度比例进入观测");
                }
                expect(blackw_controller.controller->insert_observation(blackw_observation),
                    "blackW 观测应插入历史");
                expect(blackw_controller.controller->inference_input().dimension == 342,
                    "blackW 推理输入应为 342 维");

                quadruped::motion::RlInferenceOutput blackw_inference;
                blackw_inference.ok = true;
                blackw_inference.action_dimension = 16;
                blackw_inference.actions.fill(1.0F);
                std::array<double, quadruped::core::kMaxJoints> blackw_positions{};
                for (std::size_t i = 0; i < blackw_model.model.joint_count; ++i)
                {
                    blackw_positions[i] = blackw_state.joints[i].position;
                }
                const auto blackw_command = blackw_controller.controller->convert_actions(
                    blackw_inference, blackw_positions);
                expect(blackw_command.ok,
                    "blackW 腿轮混合动作应换算成功：" + blackw_command.error_message);
                expect_close(blackw_command.target_positions[0],
                    blackw_flat.config.default_joint_positions[0] + 0.25,
                    "blackW 腿动作应转换为位置残差");
                expect_close(blackw_command.target_positions[3], blackw_positions[3],
                    "blackW Wheel 应保持当前角度");
                expect_close(blackw_command.target_velocities[3], 10.0,
                    "FL Wheel 动作应转换为正目标速度");
                expect_close(blackw_command.target_velocities[7], -10.0,
                    "FR Wheel 动作应应用独立负方向缩放");
                expect(blackw_command.kp[3] == 0.0 && blackw_command.kd[3] == 1.0,
                    "blackW Wheel 应使用零 KP 和策略 KD");

                blackw_inference.actions[3] = 100.0F;
                const auto limited = blackw_controller.controller->convert_actions(
                    blackw_inference, blackw_positions);
                expect_close(limited.target_velocities[3], 50.0,
                    "Wheel 目标速度必须按 RobotModel 上限截断");
                blackw_inference.action_dimension = 12;
                expect(!blackw_controller.controller->convert_actions(
                            blackw_inference, blackw_positions).ok,
                    "策略输出动作维度不匹配时必须拒绝");
            }
        }
    }

    if (failures != 0)
    {
        std::cerr << failures << " 个 RL 测试失败\n";
        return 1;
    }
    std::cout << "RL 配置和数据路径测试通过\n";
    return 0;
}
