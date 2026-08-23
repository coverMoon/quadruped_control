/**
 * @file rl_controller.cpp
 * @brief 实现与 rl_sar 一致的 black/blackW 配置化固定容量 RL 数据路径。
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
    if (config.robot_name != model.name || config.joint_count != model.joint_count)
    {
        error_message = "RL config identity or joint count does not match RobotModel";
        return false;
    }
    if (config.action_dimension == 0 || config.action_dimension != model.joint_count ||
        config.action_dimension > kMaxRlActionDim ||
        config.observation_dimension != 9 + 3 * config.action_dimension ||
        config.observation_dimension > kMaxRlObservationDim ||
        config.history_frame_count == 0 ||
        config.history_frame_count > kMaxRlHistoryFrames ||
        config.inference_input_dimension !=
            config.observation_dimension * config.history_frame_count ||
        config.inference_input_dimension > kMaxRlInputDim)
    {
        error_message = "RL observation, history, input, or action dimensions are inconsistent";
        return false;
    }
    std::array<bool, core::kMaxJoints> mapped{};
    for (std::size_t action_index = 0;
         action_index < config.action_dimension;
         ++action_index)
    {
        const std::size_t joint_index = config.policy_dof_indices[action_index];
        if (joint_index >= model.joint_count || mapped[joint_index])
        {
            error_message = "RL policy_dof_indices must be a joint permutation";
            return false;
        }
        mapped[joint_index] = true;
    }
    std::array<bool, kMaxRlHistoryFrames> selected_history{};
    for (std::size_t i = 0; i < config.history_frame_count; ++i)
    {
        const std::size_t frame = config.history_frames[i];
        if (frame >= kMaxRlHistoryFrames || selected_history[frame])
        {
            error_message = "RL history_frames contain an invalid or duplicate index";
            return false;
        }
        selected_history[frame] = true;
    }
    std::size_t wheel_count = 0;
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        if (config.joint_names[i] != model.joints[i].name)
        {
            error_message = "RL joint order mismatch at index " + std::to_string(i);
            return false;
        }
        if (model.joints[i].role == core::JointRole::Wheel)
        {
            if (wheel_count >= config.wheel_count ||
                config.wheel_indices[wheel_count] != i)
            {
                error_message = "RL wheel_indices do not match RobotModel JointRole";
                return false;
            }
            ++wheel_count;
        }
    }
    if (wheel_count != config.wheel_count ||
        config.leg_action_mode != RlJointActionMode::PositionResidual ||
        (wheel_count > 0 &&
            config.wheel_action_mode != RlJointActionMode::TargetVelocity))
    {
        error_message = "RL joint action modes or Wheel count are unsupported";
        return false;
    }
    if (config.name.empty() || config.model_path.empty() ||
        !finite_positive(config.angular_velocity_scale) ||
        !finite_positive(config.joint_position_scale) ||
        !finite_positive(config.joint_velocity_scale) ||
        !finite_positive(config.observation_clip) || !finite_positive(config.action_clip) ||
        !finite_positive(config.max_position_jump))
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
    for (std::size_t action_index = 0;
         action_index < config.action_dimension;
         ++action_index)
    {
        const std::size_t joint_index = config.policy_dof_indices[action_index];
        const auto& joint = model.joints[joint_index];
        const auto& limits = joint.limits;
        if (!std::isfinite(config.default_joint_positions[action_index]) ||
            !std::isfinite(config.kp[action_index]) ||
            !std::isfinite(config.kd[action_index]) ||
            !std::isfinite(config.action_scale[action_index]) ||
            config.action_scale[action_index] == 0.0 ||
            config.kp[action_index] < 0.0 || config.kp[action_index] > limits.max_kp ||
            config.kd[action_index] < 0.0 || config.kd[action_index] > limits.max_kd ||
            (joint.role == core::JointRole::Wheel && config.kp[action_index] != 0.0) ||
            (limits.position_limited &&
                (config.default_joint_positions[action_index] < limits.min_position ||
                    config.default_joint_positions[action_index] > limits.max_position)))
        {
            error_message = "RL joint parameters are invalid at action index " +
                std::to_string(action_index);
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
    for (std::size_t action_index = 0;
         action_index < config_.action_dimension;
         ++action_index)
    {
        const std::size_t joint_index = config_.policy_dof_indices[action_index];
        const double error = model_.joints[joint_index].role == core::JointRole::Wheel
            ? 0.0
            : state.joints[joint_index].position -
                config_.default_joint_positions[action_index];
        result.observation[index++] =
            static_cast<float>(error * config_.joint_position_scale);
    }
    for (std::size_t action_index = 0;
         action_index < config_.action_dimension;
         ++action_index)
    {
        const std::size_t joint_index = config_.policy_dof_indices[action_index];
        result.observation[index++] =
            static_cast<float>(
                state.joints[joint_index].velocity * config_.joint_velocity_scale);
    }
    for (std::size_t i = 0; i < config_.action_dimension; ++i)
    {
        result.observation[index++] = previous_actions_[i];
    }
    const float clip = static_cast<float>(config_.observation_clip);
    for (std::size_t i = 0; i < config_.observation_dimension; ++i)
    {
        result.observation[i] = std::clamp(result.observation[i], -clip, clip);
    }
    result.dimension = index;
    if (result.dimension != config_.observation_dimension)
    {
        result.error_message = "RL observation layout does not match configured dimension";
        return result;
    }
    result.ok = true;
    return result;
}

bool RlController::insert_observation(const ObservationResult& observation)
{
    if (!observation.ok || observation.dimension != config_.observation_dimension)
    {
        return false;
    }
    const std::size_t shift_values =
        (kMaxRlHistoryFrames - 1) * config_.observation_dimension;
    std::memmove(history_.data() + config_.observation_dimension,
        history_.data(),
        shift_values * sizeof(float));
    std::memcpy(history_.data(),
        observation.observation.data(),
        config_.observation_dimension * sizeof(float));
    return true;
}

RlController::CommandResult RlController::convert_actions(
    const RlInferenceOutput& inference,
    const std::array<double, core::kMaxJoints>& current_positions)
{
    CommandResult result;
    if (!inference.ok || inference.action_dimension != config_.action_dimension)
    {
        result.error_message = "policy action dimension does not match RL config";
        return result;
    }
    for (std::size_t action_index = 0;
         action_index < config_.action_dimension;
         ++action_index)
    {
        const std::size_t joint_index = config_.policy_dof_indices[action_index];
        if (!std::isfinite(current_positions[joint_index]))
        {
            result.error_message = "current joint position is non-finite";
            return result;
        }
        if (!std::isfinite(inference.actions[action_index]))
        {
            result.error_message = "policy action is non-finite";
            return result;
        }
    }

    if (!has_previous_targets_)
    {
        std::copy_n(current_positions.begin(), model_.joint_count, previous_targets_.begin());
        has_previous_targets_ = true;
    }
    for (std::size_t action_index = 0;
         action_index < config_.action_dimension;
         ++action_index)
    {
        const std::size_t joint_index = config_.policy_dof_indices[action_index];
        const double action =
            clamp_value(inference.actions[action_index], config_.action_clip);
        const auto& joint = model_.joints[joint_index];
        const auto& limits = joint.limits;
        result.target_positions[joint_index] = current_positions[joint_index];
        result.kp[joint_index] = config_.kp[action_index];
        result.kd[joint_index] = config_.kd[action_index];
        previous_actions_[action_index] = static_cast<float>(action);
        if (joint.role == core::JointRole::Wheel)
        {
            const double velocity = action * config_.action_scale[action_index];
            result.target_velocities[joint_index] =
                std::clamp(velocity, -limits.max_velocity, limits.max_velocity);
            continue;
        }

        double target = config_.default_joint_positions[action_index] +
            action * config_.action_scale[action_index];
        if (limits.position_limited)
        {
            const double limited = std::clamp(target, limits.min_position, limits.max_position);
            result.position_limited = result.position_limited || limited != target;
            target = limited;
        }
        const double jump = target - previous_targets_[joint_index];
        if (std::abs(jump) > config_.max_position_jump)
        {
            target = previous_targets_[joint_index] +
                std::copysign(config_.max_position_jump, jump);
            result.jump_limited = true;
        }
        result.target_positions[joint_index] = target;
        previous_targets_[joint_index] = target;
    }
    result.ok = true;
    return result;
}

RlInferenceInput RlController::inference_input() const
{
    RlInferenceInput input;
    input.dimension = config_.inference_input_dimension;
    for (std::size_t output_frame = 0;
         output_frame < config_.history_frame_count;
         ++output_frame)
    {
        const std::size_t source_frame = config_.history_frames[output_frame];
        std::copy_n(history_.begin() + source_frame * config_.observation_dimension,
            config_.observation_dimension,
            input.observation.begin() + output_frame * config_.observation_dimension);
    }
    return input;
}

const std::array<float, kMaxRlActionDim>& RlController::previous_actions() const noexcept
{
    return previous_actions_;
}

core::CommandSource RlController::active_command_source() const noexcept
{
    return active_source_;
}

}  // namespace quadruped::motion
