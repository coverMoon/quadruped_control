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

// Wolf 53-D 观测、腿/轮动作、signed 轮速和 HIM 历史展开的参考向量验证。
// 固定非零 StateFrame 和固定 raw action，不依赖 MuJoCo 或真实策略资产。
void check_wolf_rl()
{
    const auto wolf_model = quadruped::config::load_robot_model(
        QUADRUPED_WOLF_ROBOT_CONFIG_PATH);
    expect(wolf_model.ok(), "wolf RobotModel 应加载成功：" + wolf_model.error_message);
    if (!wolf_model.ok())
    {
        return;
    }
    const auto wolf = quadruped::config::load_rl_config(
        QUADRUPED_WOLF_POLICY_TEST_CONFIG_PATH,
        QUADRUPED_PROJECT_SOURCE_DIR,
        wolf_model.model);
    expect(wolf.ok(), "wolf test 配置应加载成功：" + wolf.error_message);
    if (!wolf.ok())
    {
        return;
    }
    expect(wolf.config.observation_dimension == 53 &&
            wolf.config.action_dimension == 16 &&
            wolf.config.history_frame_count == 6 &&
            wolf.config.inference_input_dimension == 318,
        "wolf test 应为 6 帧 53-D HIM 输入和 16-D 输出");

    // 固定非零状态：默认姿态上的已知偏差、已知 IMU 角速度和已知四轮速度。
    auto state = make_state(wolf_model.model, wolf.config);
    state.imu.angular_velocity = {0.1, 0.2, 0.3};
    state.joints[0].position = wolf.config.default_joint_positions[0] + 0.1;
    state.joints[0].velocity = 0.5;
    state.joints[1].position = wolf.config.default_joint_positions[1] - 0.2;
    state.joints[1].velocity = -0.4;
    state.joints[3].velocity = 3.0;
    state.joints[7].velocity = -4.0;
    state.joints[11].velocity = 5.0;
    state.joints[15].velocity = -6.0;

    quadruped::core::BaseCommand command;
    command.sequence = 1;
    command.expires_at_ns = 10'000'000;
    command.source = quadruped::core::CommandSource::Test;
    command.vx = 1.0;
    command.vy = 0.5;
    command.wz = 2.0;

    auto created = quadruped::motion::RlController::create(
        wolf_model.model, wolf.config);
    expect(created.ok(), "wolf RlController 应创建成功：" + created.error_message);
    if (!created.ok())
    {
        return;
    }
    created.controller->update_command(&command, 0);
    const auto observation = created.controller->build_observation(state, 0);
    expect(observation.ok && observation.dimension == 53,
        "wolf 53 维观测应构造成功：" + observation.error_message);
    if (observation.ok)
    {
        const auto& obs = observation.observation;
        expect_close(obs[0], 2.0, "wolf vx 应先限幅再缩放");
        expect_close(obs[1], 1.0, "wolf vy 应缩放");
        expect_close(obs[2], 0.5, "wolf wz 应缩放");
        expect_close(obs[3], 0.025, "wolf IMU 角速度应缩放");
        expect_close(obs[4], 0.05, "wolf IMU 角速度应缩放");
        expect_close(obs[5], 0.075, "wolf IMU 角速度应缩放");
        expect_close(obs[6], 0.0, "wolf 单位姿态投影重力 x 应为 0");
        expect_close(obs[7], 0.0, "wolf 单位姿态投影重力 y 应为 0");
        expect_close(obs[8], -1.0, "wolf 单位姿态投影重力 z 应为 -1");
        expect_close(obs[9], 0.1, "wolf 腿位置误差应进入观测");
        expect_close(obs[10], -0.2, "wolf 腿位置误差应进入观测");
        expect_close(obs[21], 0.025, "wolf 腿速度应缩放");
        expect_close(obs[22], -0.02, "wolf 腿速度应缩放");
        expect_close(obs[33], 0.15, "wolf FL 轮速应乘 +1 符号");
        expect_close(obs[34], 0.2, "wolf FR 轮速应乘 -1 符号");
        expect_close(obs[35], 0.25, "wolf RL 轮速应乘 +1 符号");
        expect_close(obs[36], 0.3, "wolf RR 轮速应乘 -1 符号");
        expect_close(obs[37], 0.0, "wolf 初始 previous raw action 应为零");
        expect_close(obs[52], 0.0, "wolf 初始 previous raw action 应为零");
        expect(created.controller->insert_observation(observation),
            "wolf 观测应插入历史");
    }

    quadruped::motion::RlInferenceOutput inference;
    inference.ok = true;
    inference.action_dimension = 16;
    inference.actions.fill(0.2F);
    std::array<double, quadruped::core::kMaxJoints> positions{};
    for (std::size_t i = 0; i < wolf_model.model.joint_count; ++i)
    {
        positions[i] = state.joints[i].position;
    }
    const auto output = created.controller->convert_actions(inference, positions);
    expect(output.ok, "wolf 动作换算应成功：" + output.error_message);
    expect_close(output.target_positions[0],
        wolf.config.default_joint_positions[0] + 0.2 * 0.20,
        "wolf 腿动作应为 default + 0.20 × raw");
    expect_close(output.target_velocities[3], 2.0, "wolf FL 轮动作应为 +10 × raw");
    expect_close(output.target_velocities[7], -2.0, "wolf FR 轮动作应为 -10 × raw");
    expect_close(output.target_velocities[11], 2.0, "wolf RL 轮动作应为 +10 × raw");
    expect_close(output.target_velocities[15], -2.0, "wolf RR 轮动作应为 -10 × raw");
    expect(output.kp[3] == 0.0 && output.kd[3] == 1.0,
        "wolf 轮应使用 Kp=0 的 JointImpedance 速度 PD");
    expect(output.kp[0] == 80.0 && output.kd[0] == 3.0,
        "wolf 腿应使用 Kp=80 / Kd=3.0 的位置 PD");

    // 安全裁剪（action_clip）只作用于控制目标；previous raw action 保留原始输出。
    auto trimmed = wolf.config;
    trimmed.action_clip = 1.0;
    auto trimmed_controller = quadruped::motion::RlController::create(
        wolf_model.model, trimmed);
    expect(trimmed_controller.ok(), "wolf 裁剪用例控制器应创建成功");
    if (trimmed_controller.ok())
    {
        std::array<double, quadruped::core::kMaxJoints> trimmed_positions{};
        for (std::size_t i = 0; i < wolf_model.model.joint_count; ++i)
        {
            trimmed_positions[i] = state.joints[i].position;
        }
        inference.actions.fill(5.0F);
        const auto trimmed_output =
            trimmed_controller.controller->convert_actions(inference, trimmed_positions);
        expect(trimmed_output.ok, "wolf 裁剪用例换算应成功");
        expect_close(trimmed_output.target_velocities[3], 10.0,
            "wolf 轮目标速度应使用裁剪后的 raw action");
        expect_close(trimmed_controller.controller->previous_actions()[3], 5.0,
            "wolf previous raw action 不得被 action_clip 污染");
        expect_close(trimmed_controller.controller->previous_actions()[0], 5.0,
            "wolf previous raw action 腿通道也应保留原始输出");
    }

    // HIM history：newest → oldest 按 frame-major 展平，每次插入的 command 不同。
    auto him_created = quadruped::motion::RlController::create(
        wolf_model.model, wolf.config);
    expect(him_created.ok(), "wolf HIM RlController 应创建成功：" + him_created.error_message);
    if (him_created.ok())
    {
        for (int frame = 0; frame < 6; ++frame)
        {
            quadruped::core::BaseCommand frame_command = command;
            frame_command.sequence = static_cast<std::uint64_t>(frame + 1);
            frame_command.vx = 0.1 * static_cast<double>(frame + 1);
            him_created.controller->update_command(&frame_command, 0);
            const auto frame_observation = him_created.controller->build_observation(state, 0);
            expect(frame_observation.ok, "wolf HIM 单帧观测应构造成功");
            expect(him_created.controller->insert_observation(frame_observation),
                "wolf HIM 观测应插入历史");
        }
        const auto input = him_created.controller->inference_input();
        expect(input.dimension == 318, "wolf HIM 推理输入应为 318 维");
        expect_close(input.observation[0], 1.2, "HIM 历史第 0 帧应为最新观测");
        expect_close(input.observation[53], 1.0, "HIM 历史第 1 帧应为次新观测");
        expect_close(input.observation[53 * 4], 0.4, "HIM 历史第 4 帧应为次旧观测");
        expect_close(input.observation[53 * 5], 0.2, "HIM 历史第 5 帧应为最旧观测");

        // reset 后 history 与 previous raw action 必须归零。
        him_created.controller->reset();
        const auto after_reset = him_created.controller->inference_input();
        for (std::size_t i = 0; i < after_reset.dimension; ++i)
        {
            if (after_reset.observation[i] != 0.0F)
            {
                expect(false, "reset 后 HIM 历史必须全部归零");
                break;
            }
        }
        const auto reset_observation = him_created.controller->build_observation(state, 0);
        expect(reset_observation.ok, "reset 后单帧观测应可构造");
        if (reset_observation.ok)
        {
            expect_close(reset_observation.observation[37], 0.0,
                "reset 后 previous raw action 应归零");
        }
    }
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

    check_wolf_rl();

    if (failures != 0)
    {
        std::cerr << failures << " 个 RL 测试失败\n";
        return 1;
    }
    std::cout << "RL 配置和数据路径测试通过\n";
    return 0;
}
