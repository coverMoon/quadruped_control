/**
 * @file replay_tests.cpp
 * @brief 验证状态日志身份、CSV 往返和只读 MotionRuntime 回放确定性。
 */

#include "motion_test_helpers.hpp"

#include "quadruped/backends/replay/diagnostic_log.hpp"
#include "quadruped/backends/replay/replay_log.hpp"
#include "quadruped/backends/replay/replay_robot_io.hpp"

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace qreplay = quadruped::backends::replay;

namespace
{

using motion_test::expect;
using motion_test::expect_close;

constexpr const char* kReplayPath = "/tmp/quadruped_control_replay_test.csv";
constexpr const char* kDiagnosticPath = "/tmp/quadruped_control_diagnostic_test.csv";
constexpr const char* kWheelDiagnosticPath =
    "/tmp/quadruped_control_wheel_diagnostic_test.csv";

std::vector<qc::StateFrame> make_frames(const qc::RobotModel& model)
{
    std::vector<qc::StateFrame> frames;
    for (std::uint64_t index = 0; index < 4; ++index)
    {
        auto positions = motion_test::make_rest_positions();
        positions[0] += static_cast<double>(index) * 0.01;
        auto frame = motion_test::make_state(model, positions);
        frame.header.sequence = index + 1;
        frame.header.timestamp_ns = static_cast<qc::Nanoseconds>(index) * 5'000'000;
        frame.effective_command_sequence = index;
        frames.push_back(frame);
    }
    return frames;
}

std::vector<qc::CommandFrame> run_replay(
    const qc::RobotModel& model, const std::vector<qc::StateFrame>& frames)
{
    auto runtime = qm::MotionRuntime::create(model, motion_test::make_test_config());
    expect(runtime.ok(), "回放 MotionRuntime 应创建成功：" + runtime.error_message);
    auto replay = qreplay::ReplayRobotIO::create(model, frames);
    expect(replay.ok(), "ReplayRobotIO 应创建成功：" + replay.error_message);
    if (!runtime.ok() || !replay.ok())
    {
        return {};
    }

    for (const auto& frame : frames)
    {
        qm::MotionUpdateInput input;
        input.now_ns = frame.header.timestamp_ns;
        const auto output = runtime.runtime->update(*replay.io, input);
        expect(output.read_code == qc::RobotIOCode::Ok,
            "回放状态应只读驱动 MotionRuntime");
        expect(output.submitted && output.submit_code == qc::RobotIOCode::Ok,
            "回放生成命令应仅被 ReplayRobotIO 捕获");
        expect(output.diagnostics.state_sequence == frame.header.sequence,
            "诊断应记录当前回放状态序号");
        expect(output.diagnostics.state_age_ns == 0,
            "按记录时间回放时状态年龄应为零");
        replay.io->advance();
    }
    return replay.io->generated_commands();
}

void test_log_round_trip_and_identity()
{
    const auto model = motion_test::make_test_model();
    const auto frames = make_frames(model);
    std::string error;
    expect(qreplay::write_replay_log(kReplayPath, model, frames, error),
        "回放 CSV 应写入成功：" + error);
    const auto loaded = qreplay::load_replay_log(kReplayPath, model);
    expect(loaded.ok(), "回放 CSV 应加载成功：" + loaded.error_message);
    expect(loaded.frames.size() == frames.size(), "回放 CSV 帧数应保持一致");
    if (loaded.frames.size() == frames.size())
    {
        for (std::size_t i = 0; i < frames.size(); ++i)
        {
            expect(loaded.frames[i].header.sequence == frames[i].header.sequence,
                "回放 CSV 应保持状态序号");
            expect_close({loaded.frames[i].joints[0].position,
                frames[i].joints[0].position, 0.0,
                "回放 CSV 应无损保持关节位置"});
        }
    }

    auto wrong_model = model;
    wrong_model.name = "wrong_robot";
    const auto wrong = qreplay::load_replay_log(kReplayPath, wrong_model);
    expect(!wrong.ok(), "回放必须拒绝不匹配的机器人名称");
    std::remove(kReplayPath);
}

void test_replay_is_read_only_and_deterministic()
{
    const auto model = motion_test::make_test_model();
    const auto frames = make_frames(model);
    const auto first = run_replay(model, frames);
    const auto second = run_replay(model, frames);
    expect(first.size() == frames.size() && second.size() == frames.size(),
        "每个回放状态应产生一条内存命令");
    if (first.size() != second.size())
    {
        return;
    }
    for (std::size_t i = 0; i < first.size(); ++i)
    {
        expect(first[i].header.sequence == second[i].header.sequence,
            "重复回放应生成相同命令序号");
        for (std::size_t joint = 0; joint < model.joint_count; ++joint)
        {
            expect(first[i].joints[joint].mode == second[i].joints[joint].mode,
                "重复回放应生成相同控制模式");
            expect_close({first[i].joints[joint].target_position,
                second[i].joints[joint].target_position, 0.0,
                "重复回放应生成相同目标位置"});
        }
    }

    auto invalid_frames = frames;
    invalid_frames[2].header.sequence = invalid_frames[1].header.sequence;
    const auto invalid = qreplay::ReplayRobotIO::create(model, invalid_frames);
    expect(!invalid.ok(), "回放必须拒绝同会话内倒退或重复的状态序号");
}

void test_low_rate_diagnostic_log()
{
    const auto model = motion_test::make_test_model();
    auto frames = make_frames(model);
    frames[0].effective_command_sequence = 1;
    auto runtime = qm::MotionRuntime::create(model, motion_test::make_test_config());
    auto replay = qreplay::ReplayRobotIO::create(model, frames);
    expect(runtime.ok() && replay.ok(), "诊断日志测试运行时和回放后端应创建成功");
    if (!runtime.ok() || !replay.ok())
    {
        return;
    }
    qm::MotionUpdateInput input;
    input.now_ns = frames[0].header.timestamp_ns;
    const auto update = runtime.runtime->update(*replay.io, input);
    expect(!replay.io->generated_commands().empty(), "诊断日志应有目标命令可对照");

    auto writer = qreplay::DiagnosticLogWriter::create(kDiagnosticPath, model);
    expect(writer.ok(), "低频诊断 CSV 应创建成功：" + writer.error_message);
    if (!writer.ok() || replay.io->generated_commands().empty())
    {
        return;
    }
    std::string error;
    expect(writer.writer->write(
               frames[0], update, &replay.io->generated_commands().back(), error),
        "低频诊断 CSV 应写入成功：" + error);
    writer.writer.reset();

    std::ifstream input_file(kDiagnosticPath);
    std::string header;
    std::string row;
    std::getline(input_file, header);
    std::getline(input_file, row);
    expect(header.find("inference_elapsed_ns") != std::string::npos &&
            header.find("FL_hip_joint_actual_position") != std::string::npos &&
            header.find("FL_hip_joint_target_position") != std::string::npos,
        "诊断表头应包含推理耗时、实际状态和目标命令");
    expect(!row.empty(), "诊断 CSV 应包含一条抽样记录");
    std::remove(kDiagnosticPath);

    const auto wheel_model = motion_test::make_wheel_test_model();
    const auto wheel_frames = make_frames(wheel_model);
    auto wheel_runtime =
        qm::MotionRuntime::create(wheel_model, motion_test::make_wheel_test_config());
    auto wheel_replay = qreplay::ReplayRobotIO::create(wheel_model, wheel_frames);
    auto wheel_writer =
        qreplay::DiagnosticLogWriter::create(kWheelDiagnosticPath, wheel_model);
    expect(wheel_runtime.ok() && wheel_replay.ok() && wheel_writer.ok(),
        "blackW 规格的 16 关节诊断链路应创建成功");
    if (!wheel_runtime.ok() || !wheel_replay.ok() || !wheel_writer.ok())
    {
        return;
    }
    input.now_ns = wheel_frames[0].header.timestamp_ns;
    const auto wheel_update = wheel_runtime.runtime->update(*wheel_replay.io, input);
    if (wheel_replay.io->generated_commands().empty())
    {
        expect(false, "blackW 规格回放应生成目标命令");
        return;
    }
    expect(wheel_writer.writer->write(wheel_frames[0], wheel_update,
               &wheel_replay.io->generated_commands().back(), error),
        "blackW 规格诊断行应写入成功：" + error);
    wheel_writer.writer.reset();
    std::ifstream wheel_input(kWheelDiagnosticPath);
    std::getline(wheel_input, header);
    expect(header.find("RR_wheel_joint_target_velocity") != std::string::npos,
        "blackW 诊断 CSV 应包含最后一个轮关节目标速度");
    std::remove(kWheelDiagnosticPath);
}

}  // namespace

int main()
{
    test_log_round_trip_and_identity();
    test_replay_is_read_only_and_deterministic();
    test_low_rate_diagnostic_log();
    if (motion_test::failures > 0)
    {
        std::cerr << motion_test::failures << " 个回放测试失败\n";
        return 1;
    }
    std::cout << "全部回放测试通过\n";
    return 0;
}
