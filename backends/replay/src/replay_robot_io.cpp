/**
 * @file replay_robot_io.cpp
 * @brief 实现不会连接执行设备的确定性状态回放 RobotIO。
 */

#include "quadruped/backends/replay/replay_robot_io.hpp"

#include "quadruped/core/validation.hpp"

#include <utility>

namespace quadruped::backends::replay
{

ReplayRobotIO::ReplayRobotIO(
    core::RobotModel model, std::vector<core::StateFrame> frames)
    : model_(std::move(model)), frames_(std::move(frames))
{
    status_.state = core::RobotIOState::Ready;
    status_.latest_state_sequence = frames_.front().header.sequence;
}

ReplayRobotIO::CreateResult ReplayRobotIO::create(
    core::RobotModel model, std::vector<core::StateFrame> frames)
{
    CreateResult result;
    if (const auto validation = core::validate(model); !validation)
    {
        result.error_message = "invalid RobotModel: " + validation.message;
        return result;
    }
    if (frames.empty())
    {
        result.error_message = "replay requires at least one StateFrame";
        return result;
    }
    for (std::size_t i = 0; i < frames.size(); ++i)
    {
        if (const auto validation =
                core::validate(frames[i], model, frames[i].header.timestamp_ns);
            !validation)
        {
            result.error_message = "invalid replay StateFrame at index " +
                std::to_string(i) + ": " + validation.message;
            return result;
        }
        if (i == 0)
        {
            continue;
        }
        const auto& previous = frames[i - 1].header;
        const auto& current = frames[i].header;
        const bool same_session = previous.startup_id == current.startup_id &&
            previous.session_id == current.session_id;
        if (current.timestamp_ns < previous.timestamp_ns ||
            (same_session && current.sequence <= previous.sequence))
        {
            result.error_message = "replay timestamps or sequences are not monotonic";
            return result;
        }
    }
    result.io.reset(new ReplayRobotIO(std::move(model), std::move(frames)));
    return result;
}

core::RobotIOCode ReplayRobotIO::read_latest(core::StateFrame& frame)
{
    if (frames_.empty() || frame_index_ >= frames_.size())
    {
        status_.state = core::RobotIOState::Paused;
        return core::RobotIOCode::NoData;
    }
    frame = frames_[frame_index_];
    current_read_ = true;
    status_.state = core::RobotIOState::Ready;
    status_.latest_state_sequence = frame.header.sequence;
    return core::RobotIOCode::Ok;
}

core::RobotIOCode ReplayRobotIO::submit(const core::CommandFrame& frame)
{
    if (frames_.empty() || frame_index_ >= frames_.size())
    {
        ++status_.rejected_command_frames;
        return core::RobotIOCode::NoData;
    }
    const auto& state = frames_[frame_index_];
    if (frame.header.startup_id != state.header.startup_id ||
        frame.header.session_id != state.header.session_id ||
        frame.header.sequence <= status_.latest_command_sequence ||
        !core::validate(frame, model_, state.header.timestamp_ns))
    {
        ++status_.rejected_command_frames;
        return core::RobotIOCode::InvalidFrame;
    }
    generated_commands_.push_back(frame);
    status_.latest_command_sequence = frame.header.sequence;
    return core::RobotIOCode::Ok;
}

core::RobotIOStatus ReplayRobotIO::status() const noexcept
{
    return status_;
}

bool ReplayRobotIO::advance() noexcept
{
    if (frame_index_ + 1 >= frames_.size())
    {
        status_.state = core::RobotIOState::Paused;
        return false;
    }
    if (!current_read_)
    {
        ++status_.dropped_state_frames;
    }
    const auto& previous = frames_[frame_index_].header;
    ++frame_index_;
    const auto& current = frames_[frame_index_].header;
    if (previous.startup_id != current.startup_id ||
        previous.session_id != current.session_id)
    {
        status_.latest_command_sequence = 0;
    }
    current_read_ = false;
    status_.latest_state_sequence = frames_[frame_index_].header.sequence;
    return true;
}

bool ReplayRobotIO::finished() const noexcept
{
    return frames_.empty() || frame_index_ + 1 >= frames_.size();
}

const std::vector<core::CommandFrame>& ReplayRobotIO::generated_commands() const noexcept
{
    return generated_commands_;
}

}  // namespace quadruped::backends::replay
