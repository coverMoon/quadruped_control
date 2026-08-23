/**
 * @file diagnostic_log.cpp
 * @brief 实现状态、目标命令和运行统计的低频诊断 CSV 输出。
 */

#include "quadruped/backends/replay/diagnostic_log.hpp"

#include "quadruped/core/validation.hpp"

#include <fstream>
#include <iomanip>
#include <limits>
#include <utility>

namespace quadruped::backends::replay
{
namespace
{

void write_csv_text(std::ostream& output, const std::string& text)
{
    output << '"';
    for (const char character : text)
    {
        if (character == '"')
        {
            output << "\"\"";
        }
        else if (character != '\n' && character != '\r')
        {
            output << character;
        }
    }
    output << '"';
}

const core::ModeResult* latest_result(const motion::MotionUpdateOutput& update)
{
    if (update.result_event_count > 0)
    {
        return &update.result_events[update.result_event_count - 1];
    }
    return update.has_result ? &update.result : nullptr;
}

}  // namespace

class DiagnosticLogWriter::Impl
{
public:
    core::RobotModel model{};
    std::ofstream output{};
};

DiagnosticLogWriter::DiagnosticLogWriter(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl))
{
}

DiagnosticLogWriter::~DiagnosticLogWriter() = default;

DiagnosticLogWriter::CreateResult DiagnosticLogWriter::create(
    const std::string& path, core::RobotModel model)
{
    CreateResult result;
    if (const auto validation = core::validate(model); !validation)
    {
        result.error_message = "invalid RobotModel: " + validation.message;
        return result;
    }
    auto impl = std::make_unique<Impl>();
    impl->model = std::move(model);
    impl->output.open(path, std::ios::trunc);
    if (!impl->output)
    {
        result.error_message = "cannot open diagnostic log: " + path;
        return result;
    }
    auto& output = impl->output;
    output << "timestamp_ns,startup_id,session_id,state_sequence,command_sequence,"
              "effective_command_sequence,generated_command_applied,state_age_ns,"
              "dropped_state_frames,rejected_command_frames,inference_elapsed_ns,"
              "mode,behavior,behavior_phase,policy,has_result,result_request_id,"
              "result_state,read_code,submit_code";
    for (std::size_t i = 0; i < impl->model.joint_count; ++i)
    {
        const std::string& name = impl->model.joints[i].name;
        output << ',' << name << "_actual_position," << name << "_actual_velocity,"
               << name << "_actual_effort," << name << "_target_position,"
               << name << "_target_velocity," << name << "_kp," << name << "_kd,"
               << name << "_control_mode";
    }
    output << '\n' << std::setprecision(std::numeric_limits<double>::max_digits10);
    result.writer.reset(new DiagnosticLogWriter(std::move(impl)));
    return result;
}

bool DiagnosticLogWriter::write(
    const core::StateFrame& state,
    const motion::MotionUpdateOutput& update,
    const core::CommandFrame* const generated_command,
    std::string& error_message)
{
    if (const auto validation =
            core::validate(state, impl_->model, state.header.timestamp_ns);
        !validation)
    {
        error_message = "invalid diagnostic StateFrame: " + validation.message;
        return false;
    }
    if (generated_command != nullptr)
    {
        const auto validation = core::validate(
            *generated_command, impl_->model, generated_command->header.timestamp_ns);
        if (!validation || generated_command->header.startup_id != state.header.startup_id ||
            generated_command->header.session_id != state.header.session_id)
        {
            error_message = "invalid diagnostic CommandFrame";
            return false;
        }
    }
    const bool command_applied = generated_command != nullptr &&
        generated_command->header.sequence == state.effective_command_sequence;
    const core::ModeResult* const result = latest_result(update);
    auto& output = impl_->output;
    output << state.header.timestamp_ns << ',' << update.diagnostics.startup_id << ','
           << update.diagnostics.session_id << ',' << update.diagnostics.state_sequence << ','
           << update.diagnostics.command_sequence << ','
           << update.diagnostics.effective_command_sequence << ','
           << (command_applied ? 1 : 0) << ',' << update.diagnostics.state_age_ns << ','
           << update.diagnostics.dropped_state_frames << ','
           << update.diagnostics.rejected_command_frames << ','
           << update.diagnostics.inference_elapsed_ns << ','
           << static_cast<unsigned int>(update.status.mode) << ',';
    write_csv_text(output, update.status.behavior_name);
    output << ',';
    write_csv_text(output, update.status.behavior_phase);
    output << ',';
    write_csv_text(output, update.status.policy_name);
    output << ',' << (result != nullptr ? 1 : 0) << ','
           << (result != nullptr ? result->request_id : 0) << ','
           << (result != nullptr ? static_cast<unsigned int>(result->state) : 0) << ','
           << static_cast<unsigned int>(update.read_code) << ','
           << static_cast<unsigned int>(update.submit_code);
    for (std::size_t i = 0; i < impl_->model.joint_count; ++i)
    {
        const auto& joint = state.joints[i];
        output << ',' << joint.position << ',' << joint.velocity << ',' << joint.effort;
        if (generated_command == nullptr)
        {
            output << ",0,0,0,0,0";
            continue;
        }
        const auto& command = generated_command->joints[i];
        output << ',' << command.target_position << ',' << command.target_velocity << ','
               << command.kp << ',' << command.kd << ','
               << static_cast<unsigned int>(command.mode);
    }
    output << '\n';
    output.flush();
    if (!output)
    {
        error_message = "failed while writing diagnostic log";
        return false;
    }
    error_message.clear();
    return true;
}

}  // namespace quadruped::backends::replay
