/**
 * @file rl_controller.hpp
 * @brief 定义 black/blackW 共用的固定容量 RL 数据路径。
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

// 旧 black 契约常量保留给参考向量测试；运行时使用下方最大容量和配置尺寸。
inline constexpr std::size_t kRlObservationDim = 45;
inline constexpr std::size_t kRlJointCount = 12;
inline constexpr std::size_t kRlHistoryFrames = 6;
inline constexpr std::size_t kRlInputDim = kRlObservationDim * kRlHistoryFrames;
inline constexpr std::size_t kRlActionDim = 12;
inline constexpr std::size_t kMaxRlObservationDim = 57;
inline constexpr std::size_t kMaxRlHistoryFrames = 6;
inline constexpr std::size_t kMaxRlInputDim =
    kMaxRlObservationDim * kMaxRlHistoryFrames;
inline constexpr std::size_t kMaxRlActionDim = core::kMaxJoints;
inline constexpr std::uint32_t kRlDecimation = 4;
// 50 Hz 策略的单次推理必须在一个 20 ms 策略周期内完成。
inline constexpr core::Nanoseconds kRlInferenceDeadlineNs = 20'000'000;

enum class RlJointActionMode : std::uint8_t
{
    PositionResidual = 0,
    TargetVelocity = 1,
};

// 启动期 RL 配置。关节参数数组按策略动作槽位排列，policy_dof_indices 将槽位映射到
// RobotModel 统一关节下标；当前要求映射覆盖全部关节且不重复。
struct RlConfig
{
    std::string name{};
    std::string robot_name{};
    std::string model_path{};
    std::size_t observation_dimension{0};
    std::size_t history_frame_count{0};
    std::array<std::size_t, kMaxRlHistoryFrames> history_frames{};
    std::size_t inference_input_dimension{0};
    std::size_t action_dimension{0};
    std::size_t joint_count{0};
    std::array<std::string, core::kMaxJoints> joint_names{};
    std::array<std::size_t, kMaxRlActionDim> policy_dof_indices{};
    std::size_t wheel_count{0};
    std::array<std::size_t, core::kMaxJoints> wheel_indices{};
    std::array<double, 3> command_scale{};
    std::array<double, 3> command_limits{};
    double angular_velocity_scale{0.0};
    double joint_position_scale{0.0};
    double joint_velocity_scale{0.0};
    double observation_clip{0.0};
    std::array<double, kMaxRlActionDim> default_joint_positions{};
    std::array<double, kMaxRlActionDim> kp{};
    std::array<double, kMaxRlActionDim> kd{};
    std::array<double, kMaxRlActionDim> action_scale{};
    double action_clip{0.0};
    double max_position_jump{0.0};
    RlJointActionMode leg_action_mode{RlJointActionMode::PositionResidual};
    RlJointActionMode wheel_action_mode{RlJointActionMode::TargetVelocity};
};

struct RlInferenceInput
{
    std::size_t dimension{0};
    std::array<float, kMaxRlInputDim> observation{};
};

struct RlInferenceOutput
{
    bool ok{false};
    std::string error_message{};
    std::size_t action_dimension{0};
    std::array<float, kMaxRlActionDim> actions{};
    core::Nanoseconds elapsed_ns{0};
};

// Torch 等推理后端只需实现固定容量、运行时有效尺寸的同步 forward。
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
        std::size_t dimension{0};
        std::array<float, kMaxRlObservationDim> observation{};
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
    bool insert_observation(const ObservationResult& observation);
    CommandResult convert_actions(
        const RlInferenceOutput& inference,
        const std::array<double, core::kMaxJoints>& current_positions);

    [[nodiscard]] RlInferenceInput inference_input() const;
    [[nodiscard]] const std::array<float, kMaxRlActionDim>& previous_actions() const noexcept;
    [[nodiscard]] core::CommandSource active_command_source() const noexcept;

private:
    RlController(core::RobotModel model, RlConfig config);

    core::RobotModel model_;
    RlConfig config_;
    std::optional<core::BaseCommand> command_{};
    core::CommandSource active_source_{core::CommandSource::None};
    std::array<float, kMaxRlInputDim> history_{};
    std::array<float, kMaxRlActionDim> previous_actions_{};
    std::array<double, core::kMaxJoints> previous_targets_{};
    bool has_previous_targets_{false};
};

}  // namespace quadruped::motion
