/**
 * @file mujoco_robot_io.cpp
 * @brief 实现 MujocoRobotIO：加载模型、reset 到 keyframe 并生成 StateFrame。
 */

#include "quadruped/backends/mujoco/mujoco_robot_io.hpp"

#include "state_frame_fill.hpp"

#include <cmath>
#include <string>
#include <utility>

namespace quadruped::backends::mujoco
{
namespace
{

// 秒到纳秒的换算比例，用于把 MuJoCo 的 double 仿真时间转成 int64 单调纳秒。
constexpr double kNanosecondsPerSecond = 1.0e9;

// int64 纳秒能表示的仿真秒数上限，即 (2^63 - 1) / 1e9，约 292 年。
// 超过该值时秒到纳秒的转换会溢出，必须在取整前拒绝。
constexpr double kMaxSimulationSeconds = 9.223372036854775807e9;

// 把 MuJoCo 仿真时间（秒）转换为单调纳秒。
// 取整规则：四舍五入到最近的整数纳秒；时间必须有限且非负，转换不得溢出 int64。
bool seconds_to_nanoseconds(const double seconds, core::Nanoseconds& out_ns)
{
    if (!std::isfinite(seconds) || seconds < 0.0)
    {
        return false;
    }
    if (seconds > kMaxSimulationSeconds)
    {
        return false;
    }
    out_ns = static_cast<core::Nanoseconds>(std::llround(seconds * kNanosecondsPerSecond));
    return true;
}

}  // 匿名命名空间

MujocoRobotIO::CreateResult MujocoRobotIO::create(
    const std::string& scene_path,
    const core::RobotModel& robot_model,
    const std::uint64_t startup_id)
{
    CreateResult result;
    if (startup_id == 0)
    {
        result.error_message = "startup_id must be non-zero";
        return result;
    }

    // 复用 MujocoModel::load，不重复实现第二套 XML 加载和名称映射。
    auto load = MujocoModel::load(scene_path, robot_model);
    if (!load.ok())
    {
        result.error_message = load.error_message;
        return result;
    }

    result.io.reset(new MujocoRobotIO(robot_model, std::move(*load.model), startup_id));
    return result;
}

MujocoRobotIO::ResetResult MujocoRobotIO::reset(const std::uint64_t session_id)
{
    ResetResult result;

    if (session_id == 0)
    {
        result.code = core::RobotIOCode::Rejected;
        result.error_message = "session_id must be non-zero";
        return result;
    }
    // 已建立会话后重复使用同一 session_id，会使新旧帧无法区分，因此拒绝。
    if (has_state_ && session_id == session_id_)
    {
        result.code = core::RobotIOCode::Rejected;
        result.error_message = "session already established with the same session_id";
        return result;
    }

    const mjModel* model = model_.raw_model();
    mjData* data = model_.raw_data();
    if (model == nullptr || data == nullptr)
    {
        status_.state = core::RobotIOState::Fault;
        result.code = core::RobotIOCode::Fault;
        result.error_message = "MuJoCo model or data is not available";
        return result;
    }

    // 按名称查找 default_pose keyframe，不假设它在 keyframe 数组中是第 0 个。
    const int key_id = mj_name2id(model, mjOBJ_KEY, "default_pose");
    if (key_id < 0)
    {
        status_.state = core::RobotIOState::Fault;
        result.code = core::RobotIOCode::Fault;
        result.error_message = "keyframe \"default_pose\" does not exist in the MuJoCo model";
        return result;
    }

    // 用 keyframe 恢复初始位置、速度、执行器状态和仿真时间，再刷新派生状态与传感器。
    mj_resetDataKeyframe(model, data, key_id);
    mj_forward(model, data);

    // 每个新会话的状态序号从 1 开始。
    sequence_ = 1;
    if (const std::string error = refresh_latest_state(session_id); !error.empty())
    {
        result.code = core::RobotIOCode::Fault;
        result.error_message = error;
        return result;
    }

    session_id_ = session_id;
    status_.state = core::RobotIOState::Paused;
    result.code = core::RobotIOCode::Ok;
    return result;
}

core::RobotIOCode MujocoRobotIO::read_latest(core::StateFrame& frame)
{
    if (!has_state_)
    {
        return core::RobotIOCode::NoData;
    }
    frame = latest_state_;
    latest_read_ = true;
    return core::RobotIOCode::Ok;
}

core::RobotIOCode MujocoRobotIO::submit(const core::CommandFrame&)
{
    // M1-3 尚未实现命令执行：任何命令都直接拒绝，不保存为已接受命令，
    // 不修改 latest_command_sequence，也不推进物理仿真。
    // 该阶段性行为将在 M1-4 中被真正的命令校验与力矩计算替换。
    ++status_.rejected_command_frames;
    return core::RobotIOCode::Rejected;
}

core::RobotIOStatus MujocoRobotIO::status() const noexcept
{
    return status_;
}

std::string MujocoRobotIO::refresh_latest_state(const std::uint64_t session_id)
{
    const mjData* data = model_.raw_data();
    if (model_.raw_model() == nullptr || data == nullptr)
    {
        status_.state = core::RobotIOState::Fault;
        return "MuJoCo model or data is not available";
    }

    core::Nanoseconds timestamp_ns{0};
    if (!seconds_to_nanoseconds(data->time, timestamp_ns))
    {
        status_.state = core::RobotIOState::Fault;
        return "MuJoCo simulation time is not finite and non-negative";
    }

    core::StateFrame frame;
    frame.header.schema_version = core::kFrameSchemaVersion;
    frame.header.startup_id = startup_id_;
    frame.header.session_id = session_id;
    frame.header.sequence = sequence_;
    frame.header.timestamp_ns = timestamp_ns;
    frame.header.model_id = robot_model_.model_id;
    frame.header.calibration_id = robot_model_.calibration_id;
    frame.joint_count = robot_model_.joint_count;
    // M1-3 没有主动命令和物理步进，安全状态固定为阻尼。
    frame.safety_state = core::SafetyState::Damping;
    frame.last_accepted_command_sequence = 0;
    frame.effective_command_sequence = 0;

    if (const std::string error = fill_joint_states(frame, model_, robot_model_);
        !error.empty())
    {
        status_.state = core::RobotIOState::Fault;
        return error;
    }
    if (const std::string error = fill_imu_state(frame, model_); !error.empty())
    {
        status_.state = core::RobotIOState::Fault;
        return error;
    }

    // 新状态覆盖了尚未被读取的旧状态时计一次丢帧；首次生成状态不记为丢帧。
    if (has_state_ && !latest_read_)
    {
        ++status_.dropped_state_frames;
    }
    latest_state_ = std::move(frame);
    has_state_ = true;
    latest_read_ = false;
    status_.latest_state_sequence = latest_state_.header.sequence;
    ++sequence_;
    return {};
}

}  // 命名空间 quadruped::backends::mujoco
