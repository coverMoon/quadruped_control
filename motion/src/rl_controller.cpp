/**
 * @file rl_controller.cpp
 * @brief 实现与 rl_sar black 策略一致的固定 RL 数据路径。
 */

#include "quadruped/motion/rl_controller.hpp"

#include "quadruped/core/validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace quadruped::motion
{
namespace
{

double clamp_value(const double value, const double limit)
{
    return std::clamp(value, -limit, limit);
}

bool finite_positive(const double value)
{
    return std::isfinite(value) && value > 0.0;
}

std::array<double, 3> projected_gravity(const std::array<double, 4>& q)
{
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];
    return {2.0 * (w * y - x * z),
            -2.0 * (w * x + y * z),
            -1.0 + 2.0 * (x * x + y * y)};
}

bool valid_orientation(const std::array<double, 4>& q)
{
    const double norm = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    return std::isfinite(norm) && std::abs(norm - 1.0) <= 1.0e-4;
}

}  // namespace

bool validate_rl_config(
    const RlConfig& config,
    const core::RobotModel& model,
    std::string& error_message)
{
    if (const auto validation = core::validate(model); !validation)
    {
        error_message = "invalid RobotModel: " + validation.message;
        return false;
    }
    if (model.name != "black" || model.joint_count != kRlJointCount ||
        config.robot_name != model.name)
    {
        error_message = "RL controller currently supports only the 12-joint black model";
        return false;
    }
    if (config.name.empty() || config.model_path.empty() ||
        !finite_positive(config.angular_velocity_scale) ||
        !finite_positive(config.joint_position_scale) ||
        !finite_positive(config.joint_velocity_scale) ||
        !finite_positive(config.observation_clip) || !finite_positive(config.action_scale) ||
        !finite_positive(config.action_clip) || !finite_positive(config.max_position_jump))
    {
        error_message = "RL config contains an empty identity or invalid positive scalar";
        return false;
    }
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        if (!finite_positive(config.command_scale[axis]) ||
            !finite_positive(config.command_limits[axis]))
        {
            error_message = "RL command scales and limits must be finite and positive";
            return false;
        }
    }
    for (std::size_t i = 0; i < kRlJointCount; ++i)
    {
        const auto& limits = model.joints[i].limits;
        if (!std::isfinite(config.default_joint_positions[i]) ||
            !std::isfinite(config.kp[i]) || !std::isfinite(config.kd[i]) ||
            config.kp[i] < 0.0 || config.kp[i] > limits.max_kp ||
            config.kd[i] < 0.0 || config.kd[i] > limits.max_kd ||
            (limits.position_limited &&
                (config.default_joint_positions[i] < limits.min_position ||
                    config.default_joint_positions[i] > limits.max_position)))
        {
            error_message = "RL joint parameters are invalid at index " + std::to_string(i);
            return false;
        }
    }
    error_message.clear();
    return true;
}

RlController::RlController(core::RobotModel model, RlConfig config)
    : model_(std::move(model)), config_(std::move(config))
{
    reset();
}

RlController::CreateResult RlController::create(core::RobotModel model, RlConfig config)
{
    CreateResult result;
    if (!validate_rl_config(config, model, result.error_message))
    {
        return result;
    }
    result.controller.reset(new RlController(std::move(model), std::move(config)));
    return result;
}

void RlController::reset()
{
    command_.reset();
    active_source_ = core::CommandSource::None;
    history_.fill(0.0F);
    previous_actions_.fill(0.0F);
    previous_targets_.fill(0.0);
    has_previous_targets_ = false;
}

void RlController::update_command(
    const core::BaseCommand* const command,
    const core::Nanoseconds now_ns)
{
    if (command == nullptr || !core::validate(*command, now_ns))
    {
        command_.reset();
        return;
    }
    command_ = *command;
}

RlController::ObservationResult RlController::build_observation(
    const core::StateFrame& state,
    const core::Nanoseconds now_ns)
{
    ObservationResult result;
    if (const auto validation = core::validate(state, model_, now_ns); !validation)
    {
        result.error_message = "invalid StateFrame: " + validation.message;
        return result;
    }
    if (!state.imu.valid || !valid_orientation(state.imu.orientation))
    {
        result.error_message = "RL observation requires a valid unit IMU quaternion";
        return result;
    }

    std::array<double, 3> command{};
    active_source_ = core::CommandSource::None;
    if (command_.has_value() && now_ns <= command_->expires_at_ns)
    {
        command = {clamp_value(command_->vx, config_.command_limits[0]) * config_.command_scale[0],
                   clamp_value(command_->vy, config_.command_limits[1]) * config_.command_scale[1],
                   clamp_value(command_->wz, config_.command_limits[2]) * config_.command_scale[2]};
        active_source_ = command_->source;
    }
    else
    {
        command_.reset();
    }

    const auto gravity = projected_gravity(state.imu.orientation);
    std::size_t index = 0;
    for (const double value : command)
    {
        result.observation[index++] = static_cast<float>(value);
    }
    for (const double value : state.imu.angular_velocity)
    {
        result.observation[index++] =
            static_cast<float>(value * config_.angular_velocity_scale);
    }
    for (const double value : gravity)
    {
        result.observation[index++] = static_cast<float>(value);
    }
    for (std::size_t i = 0; i < kRlJointCount; ++i)
    {
        result.observation[index++] = static_cast<float>(
            (state.joints[i].position - config_.default_joint_positions[i]) *
            config_.joint_position_scale);
    }
    for (std::size_t i = 0; i < kRlJointCount; ++i)
    {
        result.observation[index++] =
            static_cast<float>(state.joints[i].velocity * config_.joint_velocity_scale);
    }
    for (const float action : previous_actions_)
    {
        result.observation[index++] = action;
    }
    const float clip = static_cast<float>(config_.observation_clip);
    for (float& value : result.observation)
    {
        value = std::clamp(value, -clip, clip);
    }
    result.ok = true;
    return result;
}

void RlController::insert_observation(
    const std::array<float, kRlObservationDim>& observation)
{
    constexpr std::size_t kShiftBytes =
        (kRlHistoryFrames - 1) * kRlObservationDim * sizeof(float);
    std::memmove(history_.data() + kRlObservationDim, history_.data(), kShiftBytes);
    std::memcpy(history_.data(), observation.data(), kRlObservationDim * sizeof(float));
}

RlController::CommandResult RlController::convert_actions(
    const std::array<float, kRlActionDim>& actions,
    const std::array<double, core::kMaxJoints>& current_positions)
{
    CommandResult result;
    for (std::size_t i = 0; i < kRlJointCount; ++i)
    {
        if (!std::isfinite(current_positions[i]))
        {
            result.error_message = "current joint position is non-finite";
            return result;
        }
        if (!std::isfinite(actions[i]))
        {
            result.error_message = "policy action is non-finite";
            return result;
        }
    }

    if (!has_previous_targets_)
    {
        std::copy_n(current_positions.begin(), kRlJointCount, previous_targets_.begin());
        has_previous_targets_ = true;
    }
    for (std::size_t i = 0; i < kRlJointCount; ++i)
    {
        const double action = clamp_value(actions[i], config_.action_clip);
        double target = config_.default_joint_positions[i] + action * config_.action_scale;
        const auto& limits = model_.joints[i].limits;
        if (limits.position_limited)
        {
            const double limited = std::clamp(target, limits.min_position, limits.max_position);
            result.position_limited = result.position_limited || limited != target;
            target = limited;
        }
        const double jump = target - previous_targets_[i];
        if (std::abs(jump) > config_.max_position_jump)
        {
            target = previous_targets_[i] + std::copysign(config_.max_position_jump, jump);
            result.jump_limited = true;
        }
        result.target_positions[i] = target;
        result.kp[i] = config_.kp[i];
        result.kd[i] = config_.kd[i];
        previous_actions_[i] = static_cast<float>(action);
        previous_targets_[i] = target;
    }
    result.ok = true;
    return result;
}

RlInferenceInput RlController::inference_input() const
{
    RlInferenceInput input;
    input.observation = history_;
    return input;
}

const std::array<float, kRlActionDim>& RlController::previous_actions() const noexcept
{
    return previous_actions_;
}

core::CommandSource RlController::active_command_source() const noexcept
{
    return active_source_;
}

}  // namespace quadruped::motion
