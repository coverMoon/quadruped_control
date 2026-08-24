/**
 * @file mujoco_motion_tests.cpp
 * @brief 验证 MuJoCo 中自然落地、起立、RL 前进横移和趴下闭环。
 */

#include "test_helpers.hpp"

#include "quadruped/backends/mujoco/mujoco_model.hpp"
#include "quadruped/config/behavior_config_loader.hpp"
#include "quadruped/config/robot_config.hpp"
#include "quadruped/motion/motion_runtime.hpp"

#if defined(QUADRUPED_WITH_TORCH)
#include "quadruped/config/rl_config_loader.hpp"
#include "quadruped/policy/torch_policy.hpp"
#endif

#include <cmath>
#include <cstdint>
#include <limits>
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
                last_output_.result.request_id == request.request_id &&
                last_output_.result.state == qc::ModeResultState::Completed)
            {
                return true;
            }
            if (last_output_.has_result &&
                last_output_.result.request_id == request.request_id &&
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

// terrain 场景必须沿用 black 的关节、执行器和 IMU 映射，并可建立首个会话。
void check_terrain_scene_mapping(const qc::RobotModel& model)
{
    auto created = quadruped::backends::mujoco::MujocoRobotIO::create(
        QUADRUPED_BLACK_TERRAIN_SCENE_PATH, model, 1);
    expect(created.ok(), "terrain 场景应加载成功并完成 black 映射");
    if (!created.ok())
    {
        return;
    }

    const auto reset = created.io->reset(31);
    expect(reset.ok(), "terrain 场景应能建立会话并生成状态");
    if (!reset.ok())
    {
        return;
    }

    qc::StateFrame state;
    expect(created.io->read_latest(state) == qc::RobotIOCode::Ok,
        "terrain 场景 reset 后应能读取状态");
    expect(state.joint_count == model.joint_count,
        "terrain 场景状态关节数必须与 black RobotModel 一致");
    expect(state.imu.valid, "terrain 场景 IMU 映射输出必须有效");
}

// reset 必须建立新会话、清除旧命令，并拒绝使用相同会话号重新 reset。
void check_reset_session_semantics(
    quadruped::backends::mujoco::MujocoRobotIO& io,
    const qc::RobotModel& model)
{
    const auto first_reset = io.reset(11);
    expect(first_reset.ok(), "会话 11 的 reset 应成功");
    if (!first_reset.ok())
    {
        return;
    }

    qc::CommandFrame old_command =
        quadruped::backends::mujoco::test::make_command(1, qc::ControlMode::Disabled);
    old_command.header.startup_id = 1;
    old_command.header.session_id = 11;
    old_command.joint_count = model.joint_count;
    expect(io.submit(old_command) == qc::RobotIOCode::Ok,
        "旧会话中的合法命令应被接受");
    expect(io.status().latest_command_sequence == 1,
        "接受旧会话命令后应记录其序号");

    const auto same_session_reset = io.reset(11);
    expect(same_session_reset.code == qc::RobotIOCode::Rejected,
        "相同 session_id 的 reset 必须被拒绝");

    const auto second_reset = io.reset(12);
    expect(second_reset.ok(), "新会话 12 的 reset 应成功");
    if (!second_reset.ok())
    {
        return;
    }

    qc::StateFrame state;
    expect(io.read_latest(state) == qc::RobotIOCode::Ok,
        "新会话 reset 后必须立即发布状态");
    expect(state.header.session_id == 12 && state.header.sequence == 1,
        "新会话的首个状态帧必须使用 session 12 和序号 1");
    expect(io.status().latest_command_sequence == 0,
        "新会话 reset 后必须清除旧命令序号");
    expect(io.submit(old_command) == qc::RobotIOCode::Rejected,
        "旧会话命令不得跨 session 提交");
    expect(io.step() == qc::RobotIOCode::Ok,
        "新会话无有效命令时仍应能安全步进");
    expect(io.read_latest(state) == qc::RobotIOCode::Ok,
        "安全步进后应发布新状态");
    expect(state.effective_command_sequence == 0,
        "reset 后旧命令不得在新会话中生效");
}

// MuJoCo 发生内部时间故障后必须锁存 Fault，且不再接受正常命令。
void check_fault_latches_command_rejection(
    quadruped::backends::mujoco::MujocoRobotIO& io,
    const qc::RobotModel& model)
{
    const auto reset = io.reset(21);
    expect(reset.ok(), "故障用例的 reset 应成功");
    if (!reset.ok())
    {
        return;
    }

    io.raw_data()->time = std::numeric_limits<double>::quiet_NaN();
    expect(io.step() == qc::RobotIOCode::Fault,
        "非有限仿真时间必须使 MuJoCo 后端进入 Fault");
    expect(io.status().state == qc::RobotIOState::Fault,
        "后端故障后状态必须锁存为 Fault");

    qc::CommandFrame normal_command =
        quadruped::backends::mujoco::test::make_command(1, qc::ControlMode::Disabled);
    normal_command.header.startup_id = 1;
    normal_command.header.session_id = 21;
    normal_command.joint_count = model.joint_count;
    expect(io.submit(normal_command) == qc::RobotIOCode::Fault,
        "Fault 后不得继续接受正常命令");
}

// 执行侧必须让过期命令自然失效，并允许更新序号的新命令显式恢复。
void check_command_expiry_and_recovery(
    quadruped::backends::mujoco::MujocoRobotIO& io,
    const qc::RobotModel& model)
{
    expect(io.reset(32).ok(), "命令过期测试 reset 应成功");
    qc::StateFrame state;
    expect(io.read_latest(state) == qc::RobotIOCode::Ok,
        "命令过期测试应取得初始状态");
    qc::CommandFrame command;
    command.header = state.header;
    command.header.sequence = 1;
    command.expires_at_ns = state.header.timestamp_ns + 1;
    command.joint_count = model.joint_count;
    command.source = qc::CommandSource::Test;
    expect(io.submit(command) == qc::RobotIOCode::Ok,
        "尚未过期的边界时刻命令应接受");
    expect(io.step() == qc::RobotIOCode::Ok && io.step() == qc::RobotIOCode::Ok,
        "命令过期前后物理步进应保持正常");
    expect(io.read_latest(state) == qc::RobotIOCode::Ok &&
            state.effective_command_sequence == 0,
        "过期命令必须停止成为实际执行命令");
    expect(io.status().state == qc::RobotIOState::Ready,
        "命令过期是安全退路，不应锁存 backend Fault");

    command.header.timestamp_ns = state.header.timestamp_ns;
    command.header.sequence = 2;
    command.expires_at_ns = state.header.timestamp_ns + 10'000'000;
    expect(io.submit(command) == qc::RobotIOCode::Ok &&
            io.step() == qc::RobotIOCode::Ok,
        "更新序号和有效期后应允许恢复命令执行");
    expect(io.read_latest(state) == qc::RobotIOCode::Ok &&
            state.effective_command_sequence == 2,
        "恢复后的新命令应成为实际执行命令");
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

    qc::StateFrame before_reset;
    expect(ctx.io.read_latest(before_reset) == qc::RobotIOCode::Ok,
        "RL 仿真姿态复位前应能读取状态");
    const int default_pose_id =
        mj_name2id(ctx.io.raw_model(), mjOBJ_KEY, "default_pose");
    const auto pose_reset = ctx.io.reset_simulation_state(default_pose_id);
    expect(pose_reset.ok(), "RL 运行中应能加载 default_pose keyframe");
    qc::StateFrame after_reset;
    expect(ctx.io.read_latest(after_reset) == qc::RobotIOCode::Ok,
        "RL 仿真姿态复位后应立即发布状态");
    expect(after_reset.header.session_id == before_reset.header.session_id &&
            after_reset.header.timestamp_ns > before_reset.header.timestamp_ns,
        "仿真姿态复位必须保持会话和单调时间轴");
    ctx.sim.run_seconds(0.1);
    expect(ctx.sim.last_output().status.mode == qc::MotionMode::Running &&
            ctx.sim.last_output().status.behavior_name == "rl_locomotion",
        "加载 keyframe 后 RL 行为不应退出");

    qc::ModeRequest obstacle;
    obstacle.request_id = 3;
    obstacle.type = qc::ModeRequestType::SwitchPolicy;
    obstacle.policy_name = "obstacle";
    expect(ctx.sim.run_until_completed(1.0, obstacle),
        "Running 中应能从 flat 切换到 obstacle");
    expect(ctx.sim.last_output().status.policy_name == "obstacle",
        "切换完成后应报告 obstacle 策略");
    ctx.sim.run_seconds(1.0);
    expect(ctx.sim.last_output().status.mode == qc::MotionMode::Running,
        "obstacle 策略运行后应保持 Running");
    expect(ctx.io.raw_data()->qpos[2] > 0.25, "obstacle 策略运行期间躯干不应倒地");

    qc::ModeRequest flat;
    flat.request_id = 4;
    flat.type = qc::ModeRequestType::SwitchPolicy;
    flat.policy_name = "flat";
    expect(ctx.sim.run_until_completed(1.0, flat),
        "Running 中应能从 obstacle 切换回 flat");
    expect(ctx.sim.last_output().status.policy_name == "flat",
        "切换完成后应重新报告 flat 策略");

    const double start_y = ctx.io.raw_data()->qpos[1];
    ctx.sim.set_base_command(0.0, 1.0, 0.0);
    ctx.sim.run_seconds(4.0);
    const double lateral_displacement = ctx.io.raw_data()->qpos[1] - start_y;
    std::cout << "RL 横移：4 秒侧向位移=" << lateral_displacement << " m\n";
    expect(lateral_displacement > 0.05,
        "正横移命令应产生正向侧移（位移 " +
            std::to_string(lateral_displacement) + " m）");
    expect(ctx.io.raw_data()->qpos[2] > 0.25, "RL 横移期间躯干不应倒地");

    qc::ModeRequest passive;
    passive.request_id = 5;
    passive.type = qc::ModeRequestType::EnterPassive;
    expect(ctx.sim.run_until_completed(1.0, passive),
        "Running 应能通过 EnterPassive 回到 Passive");
    expect(ctx.sim.last_output().status.mode == qc::MotionMode::Passive,
        "EnterPassive 完成后应处于 Passive");
    expect(!ctx.sim.last_output().submitted ||
            ctx.sim.last_output().submit_code == qc::RobotIOCode::Ok,
        "Running → Passive 的安全命令提交不应失败");

    qc::ModeRequest getup_again;
    getup_again.request_id = 6;
    getup_again.type = qc::ModeRequestType::GetUp;
    expect(ctx.sim.run_until_completed(5.0, getup_again),
        "Running → Passive 后应能重新起立");
    expect(ctx.sim.last_output().status.mode == qc::MotionMode::Stand,
        std::string("重新起立完成后应回到 Stand（当前 ") +
            qm::motion_mode_name(ctx.sim.last_output().status.mode) +
            "，错误：" + ctx.sim.last_output().status.error_message +
            "，结果：" + ctx.sim.last_output().result.message + "）");
}
#endif

// 阶段 4/5：趴下回到记录的落地姿态并进入 Passive。
void run_getdown_phase(const MotionContext& ctx, const std::array<double, qc::kMaxJoints>& rest)
{
    qc::ModeRequest getdown;
    getdown.request_id =
#if defined(QUADRUPED_WITH_TORCH)
        7;
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

// blackW 轮方向在逻辑关节空间单独验证；正目标速度必须产生正角度变化。
void check_blackw_wheel_direction(
    quadruped::backends::mujoco::MujocoRobotIO& io,
    const qc::RobotModel& model)
{
    expect(io.reset(41).ok(), "blackW 轮方向测试 reset 应成功");
    const auto start = current_positions(io);
    std::uint64_t sequence = 0;
    for (int step = 0; step < 200; ++step)
    {
        qc::StateFrame state;
        expect(io.read_latest(state) == qc::RobotIOCode::Ok,
            "blackW 轮方向测试应读取状态");
        qc::CommandFrame command;
        command.header.startup_id = state.header.startup_id;
        command.header.session_id = state.header.session_id;
        command.header.sequence = ++sequence;
        command.header.timestamp_ns = state.header.timestamp_ns;
        command.expires_at_ns = state.header.timestamp_ns + 10'000'000;
        command.joint_count = model.joint_count;
        command.motion_mode = qc::MotionMode::Running;
        for (std::size_t i = 0; i < model.joint_count; ++i)
        {
            command.joints[i].mode = qc::ControlMode::Disabled;
            if (model.joints[i].role == qc::JointRole::Wheel)
            {
                command.joints[i].mode = qc::ControlMode::JointImpedance;
                command.joints[i].target_position = state.joints[i].position;
                command.joints[i].target_velocity = 2.0;
                command.joints[i].kp = 0.0;
                command.joints[i].kd = 2.0;
            }
        }
        expect(io.submit(command) == qc::RobotIOCode::Ok,
            "blackW 正轮速命令应被接受");
        expect(io.step() == qc::RobotIOCode::Ok,
            "blackW 正轮速测试应能步进");
    }
    const auto finish = current_positions(io);
    for (const std::size_t index : {3U, 7U, 11U, 15U})
    {
        expect(finish[index] - start[index] > 0.1,
            "blackW 正逻辑轮速应产生正角度变化（关节 " +
                std::to_string(index) + "）");
    }
}

// 验证 blackW 的映射、轮方向、基础动作和策略闭环。
void check_blackw_model_and_basic_motion()
{
    const auto model =
        quadruped::config::load_robot_model(QUADRUPED_BLACKW_ROBOT_CONFIG_PATH);
    expect(model.ok(), "加载 blackW RobotModel 失败：" + model.error_message);
    if (!model.ok())
    {
        return;
    }

    const auto loaded = quadruped::backends::mujoco::MujocoModel::load(
        QUADRUPED_BLACKW_SCENE_PATH, model.model);
    expect(loaded.ok(), "加载 blackW MuJoCo 模型失败：" + loaded.error_message);
    if (!loaded.ok())
    {
        return;
    }

    expect(loaded.model->joint_count() == 16, "blackW MuJoCo 映射应包含 16 个关节");
    expect(loaded.model->info().nu == 16, "blackW MuJoCo 模型应包含 16 个执行器");
    for (std::size_t i = 0; i < model.model.joint_count; ++i)
    {
        expect(loaded.model->joint_mappings()[i].actuator_id >= 0,
            "blackW 每个逻辑关节都应映射到唯一执行器");
    }
    expect(loaded.model->imu_mapping().quat.sensor_id >= 0,
        "blackW 应映射 imu_quat");
    expect(loaded.model->imu_mapping().gyro.sensor_id >= 0,
        "blackW 应映射 imu_gyro");
    expect(loaded.model->imu_mapping().acc.sensor_id >= 0,
        "blackW 应映射 imu_acc");

    const auto terrain = quadruped::backends::mujoco::MujocoModel::load(
        QUADRUPED_BLACKW_TERRAIN_SCENE_PATH, model.model);
    expect(terrain.ok(), "加载 blackW 地形场景失败：" + terrain.error_message);

    const auto controller = quadruped::config::load_controller_config(
        QUADRUPED_BLACKW_CONTROLLER_CONFIG_PATH, model.model);
    const auto retry = quadruped::config::load_retry_config(
        QUADRUPED_BLACKW_RETRY_CONFIG_PATH, model.model);
    const auto event_chain = quadruped::config::load_event_chain_config(
        QUADRUPED_BLACKW_EVENT_CHAIN_CONFIG_PATH, model.model);
    const auto bridge = quadruped::config::load_fixed_drive_config(
        QUADRUPED_BLACKW_BRIDGE_CONFIG_PATH, model.model);
    const auto low_bar = quadruped::config::load_fixed_drive_config(
        QUADRUPED_BLACKW_LOW_BAR_CONFIG_PATH, model.model);
    const auto car = quadruped::config::load_fixed_drive_config(
        QUADRUPED_BLACKW_CAR_CONFIG_PATH, model.model);
    expect(controller.ok(), "加载 blackW 控制器失败：" + controller.error_message);
    expect(retry.ok(), "加载 blackW Retry 失败：" + retry.error_message);
    expect(event_chain.ok(),
        "加载 blackW Event chain 失败：" + event_chain.error_message);
    expect(bridge.ok(), "加载 blackW Bridge drive 失败：" + bridge.error_message);
    expect(low_bar.ok(), "加载 blackW Low-bar drive 失败：" + low_bar.error_message);
    expect(car.ok(), "加载 blackW Car drive 失败：" + car.error_message);
    if (!controller.ok() || !retry.ok() || !event_chain.ok() ||
        !bridge.ok() || !low_bar.ok() || !car.ok())
    {
        return;
    }

    auto created = quadruped::backends::mujoco::MujocoRobotIO::create(
        QUADRUPED_BLACKW_SCENE_PATH, model.model, 1);
    expect(created.ok(), "创建 blackW MuJoCo RobotIO 失败：" + created.error_message);
    if (!created.ok())
    {
        return;
    }
    check_blackw_wheel_direction(*created.io, model.model);
    expect(created.io->reset(42).ok(), "blackW 基础动作 reset 应成功");
    auto runtime = qm::MotionRuntime::create(model.model, controller.config);
    expect(runtime.ok(), "创建 blackW MotionRuntime 失败：" + runtime.error_message);
    if (!runtime.ok())
    {
        return;
    }
    std::string behavior_error;
    expect(runtime.runtime->configure_retry(retry.config, behavior_error),
        "配置 blackW Retry 失败：" + behavior_error);
    expect(runtime.runtime->configure_event_chain(event_chain.config, behavior_error),
        "配置 rl_sar blackW Event chain 失败：" + behavior_error);
    expect(runtime.runtime->configure_fixed_drive(bridge.config, behavior_error),
        "配置 blackW Bridge drive 失败：" + behavior_error);
    expect(runtime.runtime->configure_fixed_drive(low_bar.config, behavior_error),
        "配置 blackW Low-bar drive 失败：" + behavior_error);
    expect(runtime.runtime->configure_fixed_drive(car.config, behavior_error),
        "配置 blackW Car drive 失败：" + behavior_error);

#if defined(QUADRUPED_WITH_TORCH)
    const auto flat_config = quadruped::config::load_rl_config(
        QUADRUPED_BLACKW_POLICY_FLAT_CONFIG_PATH,
        QUADRUPED_PROJECT_SOURCE_DIR,
        model.model);
    const auto obstacle_config = quadruped::config::load_rl_config(
        QUADRUPED_BLACKW_POLICY_OBSTACLE_CONFIG_PATH,
        QUADRUPED_PROJECT_SOURCE_DIR,
        model.model);
    const auto stair_config = quadruped::config::load_rl_config(
        QUADRUPED_BLACKW_POLICY_STAIR_CONFIG_PATH,
        QUADRUPED_PROJECT_SOURCE_DIR,
        model.model);
    expect(flat_config.ok(), "加载 blackW flat 配置失败：" + flat_config.error_message);
    expect(obstacle_config.ok(),
        "加载 blackW obstacle 配置失败：" + obstacle_config.error_message);
    expect(stair_config.ok(), "加载 blackW stair 配置失败：" + stair_config.error_message);
    auto flat_policy = quadruped::policy::TorchPolicy::create(flat_config.config);
    auto obstacle_policy = quadruped::policy::TorchPolicy::create(obstacle_config.config);
    auto stair_policy = quadruped::policy::TorchPolicy::create(stair_config.config);
    expect(flat_policy.ok(), "加载 blackW flat 模型失败：" + flat_policy.error_message);
    expect(obstacle_policy.ok(),
        "加载 blackW obstacle 模型失败：" + obstacle_policy.error_message);
    expect(stair_policy.ok(), "加载 blackW stair 模型失败：" + stair_policy.error_message);
    std::string policy_error;
    if (!flat_config.ok() || !obstacle_config.ok() || !stair_config.ok() ||
        !flat_policy.ok() || !obstacle_policy.ok() || !stair_policy.ok() ||
        !runtime.runtime->attach_policy(
            flat_config.config, *flat_policy.policy, policy_error) ||
        !runtime.runtime->register_policy(
            obstacle_config.config, *obstacle_policy.policy, policy_error) ||
        !runtime.runtime->register_policy(
            stair_config.config, *stair_policy.policy, policy_error) ||
        !runtime.runtime->set_policy_cycle(
            {"flat", "obstacle", "stair"}, 200, policy_error))
    {
        expect(false, "接入 blackW 策略循环失败：" + policy_error);
        return;
    }
#endif

    SimHarness sim(*created.io, *runtime.runtime, controller.config);
    // 首个无请求周期用于让 MotionRuntime 接收新会话的执行侧安全状态。
    sim.run_seconds(0.01);
    qc::ModeRequest getup;
    getup.request_id = 101;
    getup.type = qc::ModeRequestType::GetUp;
    const bool getup_completed = sim.run_until_completed(4.0, getup);
    expect(getup_completed,
        "blackW 起立应在无 Torch 条件下完成（结果：" +
            sim.last_output().result.message + "，错误：" +
            sim.last_output().status.error_message + "）");
    expect(sim.last_output().status.mode == qc::MotionMode::Stand,
        "blackW 起立后应进入 Stand");
    sim.run_seconds(1.0);
    const double blackw_stand_height = created.io->raw_data()->qpos[2];
    std::cout << "blackW Stand：躯干高度=" << blackw_stand_height << " m\n";

#if defined(QUADRUPED_WITH_TORCH)
    sim.set_base_command(0.4, 0.0, 0.0);
    qc::ModeRequest start_rl;
    start_rl.request_id = 102;
    start_rl.type = qc::ModeRequestType::StartBehavior;
    start_rl.behavior_name = "rl_locomotion";
    expect(sim.run_until_completed(1.0, start_rl),
        "blackW RL 应在首次 342→16 推理后启动");
    sim.run_seconds(20.0);
    expect(sim.last_output().status.mode == qc::MotionMode::Running &&
            sim.last_output().status.policy_name == "flat" &&
            sim.last_output().status.error_message.empty(),
        "blackW flat 长时间闭环应保持 Running");
    const double blackw_rl_height = created.io->raw_data()->qpos[2];
    std::cout << "blackW flat 闭环：20 秒躯干高度=" << blackw_rl_height << " m\n";
    expect(blackw_rl_height > 0.20,
        "blackW flat 闭环期间躯干不应倒地（高度 " +
            std::to_string(blackw_rl_height) + " m）");

    qc::ModeRequest obstacle;
    obstacle.request_id = 103;
    obstacle.type = qc::ModeRequestType::SwitchPolicy;
    obstacle.policy_name = "obstacle";
    expect(sim.run_until_completed(2.0, obstacle),
        "blackW 应按公共流程切换到 obstacle");
    sim.run_seconds(1.0);
    expect(sim.last_output().status.policy_name == "obstacle",
        "blackW 切换后应报告 obstacle");

    qc::ModeRequest stair;
    stair.request_id = 104;
    stair.type = qc::ModeRequestType::SwitchPolicy;
    stair.policy_name = "stair";
    expect(sim.run_until_completed(2.0, stair),
        "blackW 应按公共流程切换到 stair");
    sim.run_seconds(1.0);
    expect(sim.last_output().status.policy_name == "stair" &&
            sim.last_output().status.mode == qc::MotionMode::Running,
        "blackW stair 切换后应继续 Running");
#endif

    qc::ModeRequest retry_request;
    retry_request.request_id =
#if defined(QUADRUPED_WITH_TORCH)
        105;
#else
        102;
#endif
    retry_request.type = qc::ModeRequestType::StartBehavior;
    retry_request.behavior_name = "retry";
    expect(sim.run_until_completed(2.0, retry_request), "blackW Retry 应完成锁定");
    sim.run_seconds(0.5);
    expect(sim.last_output().status.mode == qc::MotionMode::Running &&
            sim.last_output().status.behavior_name == "retry" &&
            sim.last_output().status.behavior_phase == "locked",
        "blackW Retry 应持续保持 locked，不自动退出");

    qc::ModeRequest getup_again;
    getup_again.request_id = retry_request.request_id + 1;
    getup_again.type = qc::ModeRequestType::GetUp;
    expect(sim.run_until_completed(4.0, getup_again),
        "blackW Retry 应允许 GetUp 打断并回到 Stand");

    sim.set_base_command(0.2, 0.0, 0.0);
    const auto drive_start = current_positions(*created.io);
    qc::ModeRequest car_request;
    car_request.request_id = retry_request.request_id + 2;
    car_request.type = qc::ModeRequestType::StartBehavior;
    car_request.behavior_name = "car_drive";
    expect(sim.run_until_completed(1.2, car_request),
        "blackW Car drive 应完成固定姿态准备");
    sim.run_seconds(0.3);
    const auto drive_end = current_positions(*created.io);
    expect(drive_end[3] > drive_start[3] && drive_end[7] < drive_start[7] &&
            drive_end[11] > drive_start[11] && drive_end[15] < drive_start[15],
        "Car drive 正 vx 应按显式每轮符号产生统一前进方向");

    sim.set_base_command(0.0, 0.0, 0.2);
    qc::ModeRequest bridge_request;
    bridge_request.request_id = retry_request.request_id + 3;
    bridge_request.type = qc::ModeRequestType::StartBehavior;
    bridge_request.behavior_name = "bridge_drive";
    expect(sim.run_until_completed(1.2, bridge_request),
        "blackW Bridge drive 应允许从 Car drive 直接切换");

    qc::ModeRequest low_bar_request;
    low_bar_request.request_id = retry_request.request_id + 4;
    low_bar_request.type = qc::ModeRequestType::StartBehavior;
    low_bar_request.behavior_name = "low_bar_drive";
    expect(sim.run_until_completed(1.2, low_bar_request),
        "blackW Low-bar drive 应允许从 Bridge drive 直接切换");

    qc::ModeRequest fixed_getup;
    fixed_getup.request_id = retry_request.request_id + 5;
    fixed_getup.type = qc::ModeRequestType::GetUp;
    expect(sim.run_until_completed(4.0, fixed_getup),
        "固定姿态轮驱应允许 GetUp 打断并回到 Stand");

    auto encoder_event = event_chain.config;
    encoder_event.event_count = 1;
    encoder_event.events[0] = {};
    encoder_event.events[0].name = "front_encoder_drive";
    encoder_event.events[0].type = qm::EventType::Drive;
    encoder_event.events[0].wheel_group = qm::WheelGroup::Front;
    encoder_event.events[0].distance_m = 0.02;
    encoder_event.events[0].speed_mps = 0.2;
    encoder_event.events[0].timeout_cycles = 200;
    expect(runtime.runtime->configure_event_chain(encoder_event, behavior_error),
        "配置 MuJoCo 编码器位移事件失败：" + behavior_error);
    const auto wheel_start = current_positions(*created.io);
    qc::ModeRequest event_request;
    event_request.request_id = retry_request.request_id + 6;
    event_request.type = qc::ModeRequestType::StartBehavior;
    event_request.behavior_name = "event_chain";
    expect(sim.run_until_completed(1.5, event_request),
        "blackW MuJoCo 前轮编码器位移事件应完成");
    const auto wheel_end = current_positions(*created.io);
    const double encoder_distance = 0.5 * event_chain.config.wheel_radius *
        ((wheel_end[3] - wheel_start[3]) - (wheel_end[7] - wheel_start[7]));
    expect(encoder_distance >= 0.02,
        "Event chain 应按前轮平均编码器位移完成，而不是仅按周期结束");

    qc::ModeRequest event_getup;
    event_getup.request_id = retry_request.request_id + 7;
    event_getup.type = qc::ModeRequestType::GetUp;
    expect(sim.run_until_completed(4.0, event_getup),
        "完成 Event chain 后应允许 GetUp 返回 Stand");

    qc::ModeRequest getdown;
    getdown.request_id = retry_request.request_id + 8;
    getdown.type = qc::ModeRequestType::GetDown;
    expect(sim.run_until_completed(5.0, getdown),
        "blackW GetDown 应在无 Torch 条件下完成");
    expect(sim.last_output().status.mode == qc::MotionMode::Passive,
        "blackW GetDown 完成后应进入 Passive");
}

}  // namespace

int main()
{
    check_blackw_model_and_basic_motion();

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

    check_terrain_scene_mapping(model.model);

    auto reset_semantics = quadruped::backends::mujoco::MujocoRobotIO::create(
        QUADRUPED_BLACK_SCENE_PATH, model.model, 1);
    if (!reset_semantics.ok())
    {
        expect(false, "创建 reset 语义 MuJoCo 后端失败：" + reset_semantics.error_message);
        return 1;
    }
    check_reset_session_semantics(*reset_semantics.io, model.model);

    auto expiry_semantics = quadruped::backends::mujoco::MujocoRobotIO::create(
        QUADRUPED_BLACK_SCENE_PATH, model.model, 1);
    if (!expiry_semantics.ok())
    {
        expect(false, "创建命令过期语义 MuJoCo 后端失败：" +
                expiry_semantics.error_message);
        return 1;
    }
    check_command_expiry_and_recovery(*expiry_semantics.io, model.model);

    auto fault_semantics = quadruped::backends::mujoco::MujocoRobotIO::create(
        QUADRUPED_BLACK_SCENE_PATH, model.model, 1);
    if (!fault_semantics.ok())
    {
        expect(false, "创建 fault 语义 MuJoCo 后端失败：" + fault_semantics.error_message);
        return 1;
    }
    check_fault_latches_command_rejection(*fault_semantics.io, model.model);

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
    const auto flat_config = quadruped::config::load_rl_config(
        QUADRUPED_POLICY_FLAT_CONFIG_PATH, QUADRUPED_PROJECT_SOURCE_DIR, model.model);
    const auto obstacle_config = quadruped::config::load_rl_config(
        QUADRUPED_POLICY_OBSTACLE_CONFIG_PATH, QUADRUPED_PROJECT_SOURCE_DIR, model.model);
    expect(flat_config.ok(), "加载 flat RL 配置失败：" + flat_config.error_message);
    expect(obstacle_config.ok(),
        "加载 obstacle RL 配置失败：" + obstacle_config.error_message);
    auto flat_policy = quadruped::policy::TorchPolicy::create(flat_config.config);
    auto obstacle_policy = quadruped::policy::TorchPolicy::create(obstacle_config.config);
    expect(flat_policy.ok(), "加载 flat TorchScript 失败：" + flat_policy.error_message);
    expect(obstacle_policy.ok(),
        "加载 obstacle TorchScript 失败：" + obstacle_policy.error_message);
    std::string attach_error;
    if (!flat_config.ok() || !obstacle_config.ok() || !flat_policy.ok() ||
        !obstacle_policy.ok() ||
        !runtime.runtime->attach_policy(
            flat_config.config, *flat_policy.policy, attach_error) ||
        !runtime.runtime->register_policy(
            obstacle_config.config, *obstacle_policy.policy, attach_error))
    {
        expect(false, "接入 flat/obstacle 策略失败：" + attach_error);
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
