/**
 * @file mujoco_sim_controller_tests.cpp
 * @brief 测试 mujoco_sim 的 SimController：reset 会话时间同步和暂停期间的请求时间戳。
 */

#include "test_helpers.hpp"

#include "sim_controller.hpp"
#include "sim_input.hpp"

#include "quadruped/config/robot_config.hpp"
#include "quadruped/motion/motion_runtime.hpp"

#include <string>

namespace qc = quadruped::core;
namespace qm = quadruped::motion;
namespace qsim = quadruped::apps::mujoco_sim;

using quadruped::backends::mujoco::test::expect;
using quadruped::backends::mujoco::test::failures;

// 运行若干物理步；任一步失败返回 false。
bool run_steps(qsim::SimController& sim, const int steps)
{
    for (int i = 0; i < steps; ++i)
    {
        if (!sim.step())
        {
            expect(false, "仿真步进失败");
            return false;
        }
    }
    return true;
}

// 恢复后首个控制周期：会话刚切换，请求应被延后拒绝，原因必须来自会话切换
// 而不是旧会话时间污染。
void expect_first_rejection_not_timestamp(qsim::SimController& sim)
{
    bool first_result_seen = false;
    for (int i = 0; i < 5 && !first_result_seen; ++i)
    {
        if (!run_steps(sim, 1))
        {
            return;
        }
        const auto& output = sim.last_output();
        if (output.has_result)
        {
            first_result_seen = true;
            expect(output.result.state == qc::ModeResultState::Rejected,
                "会话切换周期的 GetUp 请求应被延后拒绝");
            expect(output.result.message.rfind("session changed", 0) == 0,
                "拒绝原因应来自会话切换");
        }
    }
    expect(first_result_seen, "恢复后第一个控制周期应处理请求");
}

// 会话切换周期过后，应用自动重试的同一请求应很快被接受（无需再次按键）。
void expect_retried_request_accepted(qsim::SimController& sim)
{
    bool accepted = false;
    for (int i = 0; i < 50 && !accepted; ++i)
    {
        if (!run_steps(sim, 1))
        {
            return;
        }
        const auto& output = sim.last_output();
        accepted = output.has_result &&
            output.result.state == qc::ModeResultState::Accepted;
    }
    expect(accepted, "自动重试的 GetUp 应在 0.1 秒仿真时间内被接受");
}

// 暂停 → reset → 请求：请求时间戳必须来自新会话。旧实现会把旧会话时间写入请求，
// 恢复后因“未来时间戳”被持续拒绝。
void test_pause_reset_request(qsim::SimController& sim)
{
    sim.toggle_pause();
    expect(sim.paused(), "暂停状态应生效");
    expect(sim.reset_new_session().empty(), "暂停期间 reset 应成功");
    expect(sim.sim_time() == 0.0, "reset 后应立即同步新会话时间为 0");

    qsim::SimInput input;
    input.getup = true;
    sim.apply_input(input);
    sim.toggle_pause();

    expect_first_rejection_not_timestamp(sim);
    expect_retried_request_accepted(sim);
}

int main()
{
    const auto model = quadruped::config::load_robot_model(QUADRUPED_ROBOT_CONFIG_PATH);
    expect(model.ok(), "加载 black 机器人配置失败：" + model.error_message);
    const auto controller =
        quadruped::config::load_controller_config(QUADRUPED_CONTROLLER_CONFIG_PATH, model.model);
    expect(controller.ok(), "加载 black 控制器配置失败：" + controller.error_message);
    if (!model.ok() || !controller.ok())
    {
        return 1;
    }

    auto created = quadruped::backends::mujoco::MujocoRobotIO::create(
        QUADRUPED_BLACK_SCENE_PATH, model.model, 1);
    if (!created.ok())
    {
        expect(false, "创建 MuJoCo 后端失败：" + created.error_message);
        return 1;
    }
    auto runtime = qm::MotionRuntime::create(model.model, controller.config);
    if (!runtime.ok())
    {
        expect(false, "创建 MotionRuntime 失败：" + runtime.error_message);
        return 1;
    }

    qsim::SimController sim(*created.io, *runtime.runtime, controller.config);
    expect(sim.reset_new_session().empty(), "初始 reset 应成功");
    expect(sim.sim_time() == 0.0, "reset 后仿真时间应为 0");

    // 运行约 0.4 秒，确认仿真时间已前进。
    run_steps(sim, 200);
    expect(sim.sim_time() > 0.2, "运行后仿真时间应已前进");

    test_pause_reset_request(sim);

    // 会话切换周期的延后拒绝不参与编号去重：应用每周期自动重试同一请求，
    // 无需再次按键，安全状态就绪后应很快被接受。
    expect_retried_request_accepted(sim);

    if (failures > 0)
    {
        std::cerr << failures << " 个 SimController 测试失败\n";
        return 1;
    }
    std::cout << "全部 SimController 测试通过\n";
    return 0;
}
