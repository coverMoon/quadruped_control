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
        created.controller->insert_observation(observation.observation);
        const auto input = created.controller->inference_input();
        expect_close(input.observation[0], 2.0, "历史首帧应为最新观测");
        expect_close(input.observation[quadruped::motion::kRlObservationDim], 0.0,
            "未填充历史应保持零");
    }

    std::array<float, quadruped::motion::kRlActionDim> actions{};
    actions.fill(1.0F);
    std::array<double, quadruped::core::kMaxJoints> positions{};
    std::copy(flat.config.default_joint_positions.begin(),
        flat.config.default_joint_positions.end(), positions.begin());
    const auto output = created.controller->convert_actions(actions, positions);
    expect(output.ok, "动作换算应成功：" + output.error_message);
    expect_close(output.target_positions[0],
        flat.config.default_joint_positions[0] + flat.config.action_scale,
        "动作目标应与 rl_sar 的 default + action * scale 一致");
    expect_close(output.kp[0], flat.config.kp[0], "应使用策略 KP");

    actions[0] = std::numeric_limits<float>::quiet_NaN();
    expect(!created.controller->convert_actions(actions, positions).ok,
        "非有限动作应拒绝");

    if (failures != 0)
    {
        std::cerr << failures << " 个 RL 测试失败\n";
        return 1;
    }
    std::cout << "RL 配置和数据路径测试通过\n";
    return 0;
}
