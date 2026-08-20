/**
 * @file rl_controller.hpp
 * @brief 定义 black 策略的固定观测、历史和动作换算控制器。
 */

#pragma once

#include "quadruped/core/robot_model.hpp"
#include "quadruped/core/types.hpp"

#include <array>
#include <memory>
#include <optional>
#include <string>

namespace quadruped::motion
{

inline constexpr std::size_t kRlJointCount = 12;
inline constexpr std::size_t kRlObservationDim = 45;
inline constexpr std::size_t kRlHistoryFrames = 6;
inline constexpr std::size_t kRlInputDim = kRlObservationDim * kRlHistoryFrames;
inline constexpr std::size_t kRlActionDim = 12;
inline constexpr std::uint32_t kRlDecimation = 4;
// 50 Hz 策略的单次推理必须在一个 20 ms 策略周期内完成。
inline constexpr core::Nanoseconds kRlInferenceDeadlineNs = 20'000'000;

// black 策略真正随模型变化的参数。
// 观测布局和历史长度与 rl_sar 保持固定。
struct RlConfig
{
    std::string name{};
    std::string robot_name{};
    std::string model_path{};
    std::array<double, 3> command_scale{};
    std::array<double, 3> command_limits{};
    double angular_velocity_scale{0.0};
    double joint_position_scale{0.0};
    double joint_velocity_scale{0.0};
    double observation_clip{0.0};
    std::array<double, kRlJointCount> default_joint_positions{};
    std::array<double, kRlJointCount> kp{};
    std::array<double, kRlJointCount> kd{};
    double action_scale{0.0};
    double action_clip{0.0};
    double max_position_jump{0.0};
};

struct RlInferenceInput
{
    std::array<float, kRlInputDim> observation{};
};

struct RlInferenceOutput
{
    bool ok{false};
    std::string error_message{};
    std::array<float, kRlActionDim> actions{};
    core::Nanoseconds elapsed_ns{0};
};

// Torch 等推理后端只需实现固定形状的同步 forward。
class Policy
{
public:
    virtual ~Policy() = default;
    virtual RlInferenceOutput forward(const RlInferenceInput& input) = 0;
};

[[nodiscard]] bool validate_rl_config(
    const RlConfig& config, const core::RobotModel& model, std::string& error_message);

class RlController
{
public:
    struct ObservationResult
    {
        bool ok{false};
        std::string error_message{};
        std::array<float, kRlObservationDim> observation{};
    };

    struct CommandResult
    {
        bool ok{false};
        std::string error_message{};
        std::array<double, core::kMaxJoints> target_positions{};
        std::array<double, core::kMaxJoints> target_velocities{};
        std::array<double, core::kMaxJoints> kp{};
        std::array<double, core::kMaxJoints> kd{};
        bool position_limited{false};
        bool jump_limited{false};
    };

    struct CreateResult
    {
        std::unique_ptr<RlController> controller{};
        std::string error_message{};

        [[nodiscard]] bool ok() const noexcept
        {
            return controller != nullptr;
        }
    };

    static CreateResult create(core::RobotModel model, RlConfig config);

    void reset();
    void update_command(const core::BaseCommand* command, core::Nanoseconds now_ns);
    ObservationResult build_observation(
        const core::StateFrame& state, core::Nanoseconds now_ns);
    void insert_observation(const std::array<float, kRlObservationDim>& observation);
    CommandResult convert_actions(
        const std::array<float, kRlActionDim>& actions,
        const std::array<double, core::kMaxJoints>& current_positions);

    [[nodiscard]] RlInferenceInput inference_input() const;
    [[nodiscard]] const std::array<float, kRlActionDim>& previous_actions() const noexcept;
    [[nodiscard]] core::CommandSource active_command_source() const noexcept;

private:
    RlController(core::RobotModel model, RlConfig config);

    core::RobotModel model_;
    RlConfig config_;
    std::optional<core::BaseCommand> command_{};
    core::CommandSource active_source_{core::CommandSource::None};
    std::array<float, kRlInputDim> history_{};
    std::array<float, kRlActionDim> previous_actions_{};
    std::array<double, core::kMaxJoints> previous_targets_{};
    bool has_previous_targets_{false};
};

}  // namespace quadruped::motion
