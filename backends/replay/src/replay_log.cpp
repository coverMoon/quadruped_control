/**
 * @file replay_log.cpp
 * @brief 实现带机器人身份和完整关节状态的 CSV 回放日志。
 */

#include "quadruped/backends/replay/replay_log.hpp"

#include "quadruped/backends/replay/replay_robot_io.hpp"

#include "quadruped/core/validation.hpp"

#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <type_traits>
#include <utility>

namespace quadruped::backends::replay
{
namespace
{

constexpr const char* kFormatName = "quadruped_state_log";
constexpr std::uint32_t kFormatVersion = 1;

std::vector<std::string> split_csv(const std::string& line)
{
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ','))
    {
        fields.push_back(field);
    }
    if (!line.empty() && line.back() == ',')
    {
        fields.emplace_back();
    }
    return fields;
}

template <typename Integer>
bool parse_integer(const std::string& text, Integer& output)
{
    try
    {
        std::size_t used = 0;
        if constexpr (std::is_signed<Integer>::value)
        {
            const long long value = std::stoll(text, &used);
            if (used != text.size() || value < std::numeric_limits<Integer>::min() ||
                value > std::numeric_limits<Integer>::max())
            {
                return false;
            }
            output = static_cast<Integer>(value);
        }
        else
        {
            if (text.empty() || text.front() == '-')
            {
                return false;
            }
            const unsigned long long value = std::stoull(text, &used);
            if (used != text.size() || value > std::numeric_limits<Integer>::max())
            {
                return false;
            }
            output = static_cast<Integer>(value);
        }
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool parse_double(const std::string& text, double& output)
{
    try
    {
        std::size_t used = 0;
        output = std::stod(text, &used);
        return used == text.size();
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool valid_csv_identity(const std::string& value)
{
    return !value.empty() && value.find(',') == std::string::npos &&
        value.find('\n') == std::string::npos && value.find('\r') == std::string::npos;
}

void write_frame(std::ostream& output, const core::StateFrame& frame)
{
    output << "state," << frame.header.schema_version << ',' << frame.header.startup_id << ','
           << frame.header.session_id << ',' << frame.header.sequence << ','
           << frame.header.timestamp_ns << ',' << frame.joint_count << ','
           << static_cast<unsigned int>(frame.safety_state) << ','
           << frame.last_accepted_command_sequence << ','
           << frame.effective_command_sequence << ',' << (frame.imu.valid ? 1 : 0) << ','
           << frame.imu.age_ns;
    for (const double value : frame.imu.orientation)
    {
        output << ',' << value;
    }
    for (const double value : frame.imu.angular_velocity)
    {
        output << ',' << value;
    }
    for (const double value : frame.imu.linear_acceleration)
    {
        output << ',' << value;
    }
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        const auto& joint = frame.joints[i];
        output << ',' << joint.position << ',' << joint.velocity << ',' << joint.effort << ','
               << joint.temperature_c << ',' << joint.error_code << ',' << joint.age_ns << ','
               << (joint.online ? 1 : 0) << ',' << (joint.valid ? 1 : 0);
    }
    output << '\n';
}

bool parse_frame(
    const std::vector<std::string>& fields,
    const core::RobotModel& model,
    core::StateFrame& frame)
{
    constexpr std::size_t kFixedFieldCount = 22;
    if (fields.size() != kFixedFieldCount + 8 * model.joint_count ||
        fields[0] != "state")
    {
        return false;
    }
    std::uint32_t safety = 0;
    std::uint64_t joint_count = 0;
    std::uint32_t imu_valid = 0;
    if (!parse_integer(fields[1], frame.header.schema_version) ||
        !parse_integer(fields[2], frame.header.startup_id) ||
        !parse_integer(fields[3], frame.header.session_id) ||
        !parse_integer(fields[4], frame.header.sequence) ||
        !parse_integer(fields[5], frame.header.timestamp_ns) ||
        !parse_integer(fields[6], joint_count) || !parse_integer(fields[7], safety) ||
        !parse_integer(fields[8], frame.last_accepted_command_sequence) ||
        !parse_integer(fields[9], frame.effective_command_sequence) ||
        !parse_integer(fields[10], imu_valid) || !parse_integer(fields[11], frame.imu.age_ns))
    {
        return false;
    }
    if (joint_count != model.joint_count || safety > 255 || imu_valid > 1)
    {
        return false;
    }
    frame.joint_count = static_cast<std::size_t>(joint_count);
    frame.safety_state = static_cast<core::SafetyState>(safety);
    frame.imu.valid = imu_valid != 0;
    std::size_t index = 12;
    for (double& value : frame.imu.orientation)
    {
        if (!parse_double(fields[index++], value))
        {
            return false;
        }
    }
    for (double& value : frame.imu.angular_velocity)
    {
        if (!parse_double(fields[index++], value))
        {
            return false;
        }
    }
    for (double& value : frame.imu.linear_acceleration)
    {
        if (!parse_double(fields[index++], value))
        {
            return false;
        }
    }
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        auto& joint = frame.joints[i];
        std::uint32_t online = 0;
        std::uint32_t valid = 0;
        if (!parse_double(fields[index++], joint.position) ||
            !parse_double(fields[index++], joint.velocity) ||
            !parse_double(fields[index++], joint.effort) ||
            !parse_double(fields[index++], joint.temperature_c) ||
            !parse_integer(fields[index++], joint.error_code) ||
            !parse_integer(fields[index++], joint.age_ns) ||
            !parse_integer(fields[index++], online) ||
            !parse_integer(fields[index++], valid) || online > 1 || valid > 1)
        {
            return false;
        }
        joint.online = online != 0;
        joint.valid = valid != 0;
    }
    return true;
}

}  // namespace

bool write_replay_log(
    const std::string& path,
    const core::RobotModel& model,
    const std::vector<core::StateFrame>& frames,
    std::string& error_message)
{
    if (!core::validate(model) || !valid_csv_identity(model.name) || frames.empty())
    {
        error_message = "invalid RobotModel identity or empty replay log";
        return false;
    }
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        if (!valid_csv_identity(model.joints[i].name))
        {
            error_message = "joint name cannot be represented in replay CSV";
            return false;
        }
    }
    const auto replay = ReplayRobotIO::create(model, frames);
    if (!replay.ok())
    {
        error_message = replay.error_message;
        return false;
    }
    std::ofstream output(path, std::ios::trunc);
    if (!output)
    {
        error_message = "cannot open replay log for writing: " + path;
        return false;
    }
    output << kFormatName << ',' << kFormatVersion << ',' << model.name << ','
           << model.joint_count << '\n';
    output << "joint_names";
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        output << ',' << model.joints[i].name;
    }
    output << '\n' << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& frame : frames)
    {
        write_frame(output, frame);
    }
    if (!output)
    {
        error_message = "failed while writing replay log: " + path;
        return false;
    }
    error_message.clear();
    return true;
}

ReplayLogLoadResult load_replay_log(
    const std::string& path, const core::RobotModel& model)
{
    ReplayLogLoadResult result;
    std::ifstream input(path);
    if (!input)
    {
        result.error_message = "cannot open replay log: " + path;
        return result;
    }
    std::string line;
    if (!std::getline(input, line))
    {
        result.error_message = "replay log is empty";
        return result;
    }
    const auto identity = split_csv(line);
    std::uint32_t version = 0;
    std::uint64_t joint_count = 0;
    if (identity.size() != 4 || identity[0] != kFormatName ||
        !parse_integer(identity[1], version) || version != kFormatVersion ||
        identity[2] != model.name || !parse_integer(identity[3], joint_count) ||
        joint_count != model.joint_count)
    {
        result.error_message = "replay log identity does not match RobotModel";
        return result;
    }
    if (!std::getline(input, line))
    {
        result.error_message = "replay log is missing joint_names";
        return result;
    }
    const auto names = split_csv(line);
    if (names.size() != model.joint_count + 1 || names[0] != "joint_names")
    {
        result.error_message = "replay joint_names have an invalid dimension";
        return result;
    }
    for (std::size_t i = 0; i < model.joint_count; ++i)
    {
        if (names[i + 1] != model.joints[i].name)
        {
            result.error_message = "replay joint order mismatch at index " +
                std::to_string(i);
            return result;
        }
    }
    std::size_t line_number = 2;
    while (std::getline(input, line))
    {
        ++line_number;
        if (line.empty())
        {
            continue;
        }
        core::StateFrame frame;
        if (!parse_frame(split_csv(line), model, frame))
        {
            result.error_message = "invalid replay row at line " +
                std::to_string(line_number);
            return result;
        }
        result.frames.push_back(frame);
    }
    const auto replay = ReplayRobotIO::create(model, result.frames);
    if (!replay.ok())
    {
        result.error_message = replay.error_message;
    }
    return result;
}

}  // namespace quadruped::backends::replay
