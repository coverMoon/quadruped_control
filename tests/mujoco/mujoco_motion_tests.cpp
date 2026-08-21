/**
 * @file mujoco_motion_tests.cpp
 * @brief 验证 MuJoCo 中自然落地、起立、RL 前进横移和趴下闭环。
 */

#include "test_helpers.hpp"

#include "quadruped/config/robot_config.hpp"
#include "quadruped/motion/motion_runtime.hpp"

#if defined(QUADRUPED_WITH_TORCH)
#include "quadruped/config/rl_config_loader.hpp"
#include "quadruped/policy/torch_policy.hpp"
#endif

#include <cmath>
#include <cstdint>
#include <string>

namespace qc = quadruped::core;
namespace qm = quadruped::motion;

namespace
{

using quadruped::backends::mujoco::test::expect;
using quadruped::backends::mujoco::test::expect_close;
using quadruped::backends::mujoco::test::failures;

// 集成测试的关节位置容差，单位为 rad；接触和重力使稳态值略偏离目标。
constexpr double kPositionTolerance = 0.15;

// 统一调度 2 ms 物理步进和 5 ms 控制周期的最小测试环境。
class SimHarness
{
public:
    SimHarness(
        quadruped::backends::mujoco::MujocoRobotIO& io,
        qm::MotionRuntime& runtime,
        const qc::ControllerConfig& config)
        : io_(io), runtime_(runtime), config_(config)
    {
    }

    // 推进一个物理步；到达控制周期时先运行 MotionRuntime。
    // request 非空时每周期重试同一请求，便于观察结果推进。
    bool tick(const qc::ModeRequest* request = nullptr)
    {
        qc::StateFrame state;
        if (io_.read_latest(state) != qc::RobotIOCode::Ok)
        {
            return false;
        }
        if (next_control_ns_ < 0)
        {
            next_control_ns_ = state.header.timestamp_ns;
        }
        if (state.header.timestamp_ns >= next_control_ns_)
        {
            qm::MotionUpdateInput input;
            input.now_ns = state.header.timestamp_ns;
            input.request = request;
            if (has_base_command_)
            {
                base_command_.sequence += 1;
                base_command_.timestamp_ns = state.header.timestamp_ns;
                base_command_.expires_at_ns = state.header.timestamp_ns + 100'000'000;
                input.base_command = &base_command_;
            }
            last_output_ = runtime_.update(io_, input);
            if (last_output_.read_code != qc::RobotIOCode::Ok)
            {
                return false;
            }
            next_control_ns_ += config_.control_period_ns;
        }
        return io_.step() == qc::RobotIOCode::Ok;
    }

    // 推进给定秒数的仿真时间；任一步失败都会提前停止并记录断言。
    void run_seconds(double seconds, const qc::ModeRequest* request = nullptr)
    {
        const int steps = static_cast<int>(std::ceil(seconds / timestep()));
        for (int i = 0; i < steps; ++i)
        {
            if (!tick(request))
            {
                expect(false, "仿真步进或控制周期失败");
                return;
            }
        }
    }

    // 推进仿真直到请求完成或超过秒数上限；返回是否在限时内完成。
    bool run_until_completed(double max_seconds, const qc::ModeRequest& request)
    {
        const int max_steps = static_cast<int>(std::ceil(max_seconds / timestep()));
        for (int i = 0; i < max_steps; ++i)
        {
            if (!tick(&request))
            {
                expect(false, "等待请求完成期间仿真失败");
                return false;
            }
            if (last_output_.has_result &&
                last_output_.result.state == qc::ModeResultState::Completed)
            {
                return true;
            }
            if (last_output_.has_result &&
                (last_output_.result.state == qc::ModeResultState::Rejected ||
                    last_output_.result.state == qc::ModeResultState::Failed))
            {
                return false;
            }
        }
        return false;
    }

    const qm::MotionUpdateOutput& last_output() const
    {
        return last_output_;
    }

    void set_base_command(const double vx, const double vy, const double wz)
    {
        has_base_command_ = true;
        base_command_.source = qc::CommandSource::Test;
        base_command_.priority = 1;
        base_command_.vx = vx;
        base_command_.vy = vy;
        base_command_.wz = wz;
    }

private:
    double timestep() const
    {
        return io_.raw_model()->opt.timestep;
    }

    quadruped::backends::mujoco::MujocoRobotIO& io_;
    qm::MotionRuntime& runtime_;
    const qc::ControllerConfig& config_;
    qc::Nanoseconds next_control_ns_{-1};
    qm::MotionUpdateOutput last_output_{};
    qc::BaseCommand base_command_{};
    bool has_base_command_{false};
};

// 读取当前状态中的全部关节位置。
std::array<double, qc::kMaxJoints> current_positions(
    quadruped::backends::mujoco::MujocoRobotIO& io)
{
    std::array<double, qc::kMaxJoints> positions{};
    qc::StateFrame state;
    if (io.read_latest(state) == qc::RobotIOCode::Ok)
    {
        for (std::size_t i = 0; i < state.joint_count; ++i)
        {
            positions[i] = state.joints[i].position;
        }
    }
    return positions;
}

// 一次姿态比对用例的全部输入。
struct PoseCheck
{
    const std::array<double, qc::kMaxJoints>& actual;
    const std::array<double, qc::kMaxJoints>& expected;
    std::size_t joint_count;
    const char* description;
};

void expect_near_pose(const PoseCheck& check)
{
    for (std::size_t i = 0; i < check.joint_count; ++i)
    {
        expect_close(check.actual[i], check.expected[i], kPositionTolerance,
            std::string(check.description) + "（关节 " + std::to_string(i) + "）");
    }
}

// 集成测试各阶段共享的后端、调度和配置上下文。
struct MotionContext
{
    quadruped::backends::mujoco::MujocoRobotIO& io;
    SimHarness& sim;
    const qc::RobotModel& model;
    const qc::ControllerConfig& config;
};

// Reset 必须表示 MJCF 零位，不能被 RL 站姿 keyframe 或控制器配置覆盖。
void check_reset_pose(const MotionContext& ctx)
{
    const auto positions = current_positions(ctx.io);
    for (std::size_t i = 0; i < ctx.model.joint_count; ++i)
    {
        expect_close(positions[i], 0.0, 1.0e-12,
            "Reset 后关节应回到 MJCF 零位（关节 " + std::to_string(i) + "）");
    }
}

// 阶段 1 的落地结果：记录的落地姿态和落地时的躯干高度。
struct FallPhaseResult
{
    std::array<double, qc::kMaxJoints> rest{};
    double fallen_height{0.0};
};

// 阶段 1：Passive 下自然落地，返回记录的落地姿态。
FallPhaseResult run_fall_phase(const MotionContext& ctx)
{
    // XML 零位的四条腿接近伸直，被动落地比 RL 屈腿姿态慢；
    // 这里给足自然失稳和接触稳定时间，不把旧 keyframe 的塌落速度写成接口要求。
    ctx.sim.run_seconds(3.0);
    FallPhaseResult result;
    result.fallen_height = ctx.io.raw_data()->qpos[2];
    expect(result.fallen_height < 0.25,
        "Disabled 下机器人应自然落地（躯干高度 " +
            std::to_string(result.fallen_height) + "）");
    expect(ctx.sim.last_output().status.mode == qc::MotionMode::Passive,
        "落地后应保持 Passive");
    result.rest = current_positions(ctx.io);
    return result;
}

// 阶段 2/3：两段起立进入 Stand 并保持。
void run_stand_phase(const MotionContext& ctx, const double fallen_height)
{
    qc::ModeRequest getup;
    getup.request_id = 1;
    getup.type = qc::ModeRequestType::GetUp;
    expect(ctx.sim.run_until_completed(5.0, getup), "起立应在 5 秒仿真时间内完成");
    expect(ctx.sim.last_output().status.mode == qc::MotionMode::Stand,
        "起立完成后应进入 Stand");

    ctx.sim.run_seconds(1.0);
    expect(ctx.sim.last_output().status.mode == qc::MotionMode::Stand,
        "保持期间应处于 Stand");
    expect_near_pose({current_positions(ctx.io), ctx.config.stand_position,
        ctx.model.joint_count, "Stand 应保持默认站姿"});
    const double stand_height = ctx.io.raw_data()->qpos[2];
    expect(stand_height > fallen_height,
        "站立躯干高度应高于落地高度（站立 " + std::to_string(stand_height) +
            "，落地 " + std::to_string(fallen_height) + "）");
}

#if defined(QUADRUPED_WITH_TORCH)
// 阶段 4：真实 flat 策略接收前进命令，在 MuJoCo 中保持站立并产生机体位移。
void run_rl_phase(const MotionContext& ctx)
{
    const double start_x = ctx.io.raw_data()->qpos[0];
    ctx.sim.set_base_command(0.6, 0.0, 0.0);

    qc::ModeRequest start;
    start.request_id = 2;
    start.type = qc::ModeRequestType::StartBehavior;
    start.behavior_name = "rl_locomotion";
    const bool started = ctx.sim.run_until_completed(1.0, start);
    expect(started,
        "RL behavior 应在首次成功推理后启动（状态：" +
            ctx.sim.last_output().status.error_message + "，请求：" +
            ctx.sim.last_output().result.message + "）");
    ctx.sim.run_seconds(4.0);

    const double displacement = ctx.io.raw_data()->qpos[0] - start_x;
    const double height = ctx.io.raw_data()->qpos[2];
    std::cout << "RL 闭环：4 秒位移=" << displacement << " m，躯干高度=" << height
              << " m\n";
    expect(ctx.sim.last_output().status.mode == qc::MotionMode::Running,
        "RL 闭环期间应保持 Running");
    expect(ctx.sim.last_output().status.behavior_name == "rl_locomotion",
        "运行状态应报告 rl_locomotion");
    expect(ctx.sim.last_output().status.error_message.empty(), "RL 闭环不应报告错误");
    expect(displacement > 0.10,
        "前进命令应产生明显前向位移（位移 " + std::to_string(displacement) + " m）");
    expect(height > 0.25,
        "RL 行走期间躯干不应倒地（高度 " + std::to_string(height) + " m）");

    const double start_y = ctx.io.raw_data()->qpos[1];
    ctx.sim.set_base_command(0.0, 1.0, 0.0);
    ctx.sim.run_seconds(4.0);
    const double lateral_displacement = ctx.io.raw_data()->qpos[1] - start_y;
    std::cout << "RL 横移：4 秒侧向位移=" << lateral_displacement << " m\n";
    expect(lateral_displacement > 0.05,
        "正横移命令应产生正向侧移（位移 " +
            std::to_string(lateral_displacement) + " m）");
    expect(ctx.io.raw_data()->qpos[2] > 0.25, "RL 横移期间躯干不应倒地");
}
#endif

// 阶段 4/5：趴下回到记录的落地姿态并进入 Passive。
void run_getdown_phase(const MotionContext& ctx, const std::array<double, qc::kMaxJoints>& rest)
{
    qc::ModeRequest getdown;
    getdown.request_id =
#if defined(QUADRUPED_WITH_TORCH)
        3;
#else
        2;
#endif
    getdown.type = qc::ModeRequestType::GetDown;
    expect(ctx.sim.run_until_completed(8.0, getdown), "趴下应在 8 秒仿真时间内完成");
    expect(ctx.sim.last_output().status.mode == qc::MotionMode::Passive,
        "趴下完成后应进入 Passive");

    ctx.sim.run_seconds(1.0);
    expect_near_pose({current_positions(ctx.io), rest, ctx.model.joint_count,
        "趴下后应回到记录的落地姿态"});
}

}  // namespace

int main()
{
    // 配置统一来自仓库 YAML，与带界面应用使用同一份运动参数。
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
#if defined(QUADRUPED_WITH_TORCH)
    const auto rl_config = quadruped::config::load_rl_config(
        QUADRUPED_POLICY_FLAT_CONFIG_PATH, QUADRUPED_PROJECT_SOURCE_DIR, model.model);
    expect(rl_config.ok(), "加载 flat RL 配置失败：" + rl_config.error_message);
    auto policy = quadruped::policy::TorchPolicy::create(rl_config.config);
    expect(policy.ok(), "加载 flat TorchScript 失败：" + policy.error_message);
    std::string attach_error;
    if (!rl_config.ok() || !policy.ok() ||
        !runtime.runtime->attach_policy(rl_config.config, *policy.policy, attach_error))
    {
        expect(false, "接入 flat 策略失败：" + attach_error);
        return 1;
    }
#endif
    if (!created.io->reset(1).ok())
    {
        expect(false, "reset 失败");
        return 1;
    }

    SimHarness sim(*created.io, *runtime.runtime, controller.config);
    const MotionContext context{*created.io, sim, model.model, controller.config};
    check_reset_pose(context);
    const FallPhaseResult fall = run_fall_phase(context);
    run_stand_phase(context, fall.fallen_height);
#if defined(QUADRUPED_WITH_TORCH)
    run_rl_phase(context);
#endif
    run_getdown_phase(context, fall.rest);

    if (failures > 0)
    {
        std::cerr << failures << " 个 MuJoCo 运动集成测试失败\n";
        return 1;
    }
    std::cout << "全部 MuJoCo 运动集成测试通过\n";
    return 0;
}
