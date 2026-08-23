/**
 * @file rl_torch_tests.cpp
 * @brief 用真实模型参考输出验证精简 TorchPolicy。
 */

#include "quadruped/config/rl_config_loader.hpp"
#include "quadruped/config/robot_config.hpp"
#include "quadruped/policy/torch_policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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

void check_policy(
    const quadruped::motion::RlConfig& config,
    const std::array<float, quadruped::motion::kRlActionDim>& expected)
{
    auto created = quadruped::policy::TorchPolicy::create(config);
    expect(created.ok(), config.name + " 模型应加载：" + created.error_message);
    if (!created.ok())
    {
        return;
    }
    quadruped::motion::RlInferenceInput input;
    input.dimension = config.inference_input_dimension;
    const auto output = created.policy->forward(input);
    expect(output.ok, config.name + " 零输入推理应成功：" + output.error_message);
    for (std::size_t i = 0; output.ok && i < expected.size(); ++i)
    {
        expect(std::abs(output.actions[i] - expected[i]) <= 1.0e-5F,
            config.name + " 动作 " + std::to_string(i) + " 应匹配参考值");
    }

    constexpr int kTimingSamples = 20;
    std::int64_t total_ns = 0;
    std::int64_t maximum_ns = 0;
    for (int sample = 0; sample < kTimingSamples; ++sample)
    {
        const auto timed_output = created.policy->forward(input);
        expect(timed_output.ok, config.name + " 连续推理应成功");
        total_ns += timed_output.elapsed_ns;
        maximum_ns = std::max(maximum_ns, timed_output.elapsed_ns);
    }
    const double average_ms =
        static_cast<double>(total_ns) / static_cast<double>(kTimingSamples) / 1'000'000.0;
    const double maximum_ms = static_cast<double>(maximum_ns) / 1'000'000.0;
    std::cout << config.name << " CPU 推理：平均 "
              << average_ms << " ms，最大 " << maximum_ms << " ms\n";
}

void check_policy_shape(const quadruped::motion::RlConfig& config)
{
    auto created = quadruped::policy::TorchPolicy::create(config);
    expect(created.ok(), config.name + " blackW 模型应加载：" + created.error_message);
    if (!created.ok())
    {
        return;
    }
    quadruped::motion::RlInferenceInput input;
    input.dimension = config.inference_input_dimension;
    const auto output = created.policy->forward(input);
    expect(output.ok, config.name + " blackW 零输入推理应成功：" + output.error_message);
    expect(output.action_dimension == config.action_dimension,
        config.name + " blackW 输出应为配置的 16 维动作");
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
    expect(flat.ok(), "flat 配置应加载");
    expect(obstacle.ok(), "obstacle 配置应加载");
    if (flat.ok())
    {
        check_policy(flat.config,
            {-0.118001F, -0.135057F, 0.501130F, 0.136212F, 0.108405F, -0.493434F,
             0.138194F, 0.065731F, 0.560825F, -0.154469F, -0.037799F, -0.579427F});
    }
    if (obstacle.ok())
    {
        check_policy(obstacle.config,
            {-0.006324F, -0.045350F, 0.053213F, 0.074816F, -0.080518F, -0.190362F,
             0.034378F, 0.035897F, 0.145156F, -0.029368F, -0.083529F, -0.299602F});
    }
    const auto blackw_model = quadruped::config::load_robot_model(
        QUADRUPED_BLACKW_ROBOT_CONFIG_PATH);
    expect(blackw_model.ok(), "blackW RobotModel 应加载成功");
    if (blackw_model.ok())
    {
        for (const std::string path : {QUADRUPED_BLACKW_POLICY_FLAT_CONFIG_PATH,
                 QUADRUPED_BLACKW_POLICY_OBSTACLE_CONFIG_PATH,
                 QUADRUPED_BLACKW_POLICY_STAIR_CONFIG_PATH})
        {
            const auto config = quadruped::config::load_rl_config(
                path, QUADRUPED_PROJECT_SOURCE_DIR, blackw_model.model);
            expect(config.ok(), "blackW 策略配置应加载：" + config.error_message);
            if (config.ok())
            {
                check_policy_shape(config.config);
            }
        }
    }
    if (failures != 0)
    {
        std::cerr << failures << " 个 Torch 测试失败\n";
        return 1;
    }
    std::cout << "Torch 策略参考输出测试通过\n";
    return 0;
}
