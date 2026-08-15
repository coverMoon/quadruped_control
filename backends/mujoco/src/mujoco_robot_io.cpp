/**
 * @file mujoco_robot_io.cpp
 * @brief 实现 MujocoRobotIO：加载模型、reset 到 keyframe 并生成 StateFrame。
 */

#include "quadruped/backends/mujoco/mujoco_robot_io.hpp"

#include "sim_time.hpp"
#include "state_frame_fill.hpp"

#include <string>
#include <utility>

namespace quadruped::backends::mujoco
{
namespace
{

// 把时间转换失败分类映射为可读错误文本。
std::string sim_time_error_message(const SimTimeError error)
{
    switch (error)
    {
    case SimTimeError::NonFinite:
        return "MuJoCo simulation time is not finite";
    case SimTimeError::Negative:
        return "MuJoCo simulation time is negative";
    case SimTimeError::Overflow:
        return "MuJoCo simulation time exceeds the int64 nanosecond range";
    case SimTimeError::None:
        break;
    }
    return {};
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
    // 刷新失败时恢复原序号，保证当前已建立会话内已发布帧的序号仍然单调。
    const std::uint64_t previous_sequence = sequence_;
    sequence_ = 1;
    if (const std::string error = refresh_latest_state(session_id); !error.empty())
    {
        sequence_ = previous_sequence;
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

std::string MujocoRobotIO::refresh_for_test()
{
    // 会话未建立时拒绝，保证 reset 之前不存在任何可发布状态的路径。
    if (session_id_ == 0)
    {
        return "no established session; reset must succeed before refreshing state";
    }
    return refresh_latest_state(session_id_);
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
    if (const SimTimeError error = seconds_to_nanoseconds(data->time, timestamp_ns);
        error != SimTimeError::None)
    {
        status_.state = core::RobotIOState::Fault;
        return sim_time_error_message(error);
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
