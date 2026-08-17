/**
 * @file mujoco_motion_tests.cpp
 * @brief MuJoCo 环境下 MotionRuntime 的无界面集成测试：自然落地、两段起立、站立和趴下。
 */

#include "test_helpers.hpp"

#include "quadruped/config/robot_config.hpp"
#include "quadruped/motion/motion_runtime.hpp"

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
        }
        return false;
    }

    const qm::MotionUpdateOutput& last_output() const
    {
        return last_output_;
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

// 阶段 1 的落地结果：记录的落地姿态和落地时的躯干高度。
struct FallPhaseResult
{
    std::array<double, qc::kMaxJoints> rest{};
    double fallen_height{0.0};
};

// 阶段 1：Passive 下自然落地，返回记录的落地姿态。
FallPhaseResult run_fall_phase(const MotionContext& ctx)
{
    ctx.sim.run_seconds(1.0);
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

// 阶段 4/5：趴下回到记录的落地姿态并进入 Passive。
void run_getdown_phase(const MotionContext& ctx, const std::array<double, qc::kMaxJoints>& rest)
{
    qc::ModeRequest getdown;
    getdown.request_id = 2;
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
    if (!created.io->reset(1).ok())
    {
        expect(false, "reset 失败");
        return 1;
    }

    SimHarness sim(*created.io, *runtime.runtime, controller.config);
    const MotionContext context{*created.io, sim, model.model, controller.config};
    const FallPhaseResult fall = run_fall_phase(context);
    run_stand_phase(context, fall.fallen_height);
    run_getdown_phase(context, fall.rest);

    if (failures > 0)
    {
        std::cerr << failures << " 个 MuJoCo 运动集成测试失败\n";
        return 1;
    }
    std::cout << "全部 MuJoCo 运动集成测试通过\n";
    return 0;
}
