/**
 * @file ipc_tests.cpp
 * @brief 测试三进程共享内存 wire 转换、最新值、可靠队列和会话安全语义。
 */

#include "quadruped/config/robot_config.hpp"
#include "quadruped/core/constants.hpp"
#include "quadruped/ipc/conversions.hpp"
#include "quadruped/ipc/remote_robot_io.hpp"
#include "quadruped/ipc/shared_memory.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <unistd.h>

namespace qc = quadruped::core;
namespace qi = quadruped::ipc;

namespace
{

int failures = 0;

void expect(const bool condition, const std::string& description)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << description << '\n';
        ++failures;
    }
}

qc::StateFrame make_state(
    const qc::RobotModel& model,
    const std::uint64_t startup_id,
    const std::uint64_t session_id,
    const qc::Nanoseconds timestamp_ns)
{
    qc::StateFrame frame;
    frame.header.schema_version = qc::kFrameSchemaVersion;
    frame.header.startup_id = startup_id;
    frame.header.session_id = session_id;
    frame.header.sequence = 9;
    frame.header.timestamp_ns = timestamp_ns;
    frame.joint_count = model.joint_count;
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].position = 0.01 * static_cast<double>(i);
        frame.joints[i].velocity = -0.02 * static_cast<double>(i);
        frame.joints[i].online = true;
        frame.joints[i].valid = true;
    }
    frame.imu.valid = true;
    frame.safety_state = qc::SafetyState::ControlEnabled;
    return frame;
}

qc::CommandFrame make_command(
    const qc::RobotModel& model,
    const std::uint64_t startup_id,
    const std::uint64_t session_id,
    const qc::Nanoseconds timestamp_ns)
{
    qc::CommandFrame frame;
    frame.header.schema_version = qc::kFrameSchemaVersion;
    frame.header.startup_id = startup_id;
    frame.header.session_id = session_id;
    frame.header.sequence = 10;
    frame.header.timestamp_ns = timestamp_ns;
    frame.expires_at_ns = timestamp_ns + 10'000'000;
    frame.joint_count = model.joint_count;
    frame.motion_mode = qc::MotionMode::Stand;
    frame.source = qc::CommandSource::Test;
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        frame.joints[i].mode = qc::ControlMode::JointImpedance;
        frame.joints[i].target_position = 0.01 * static_cast<double>(i);
        frame.joints[i].kp = 10.0;
        frame.joints[i].kd = 1.0;
    }
    return frame;
}

void test_identity_and_conversions(const qc::RobotModel& model)
{
    const qi::WireIdentity identity = qi::make_identity(model);
    std::string error_message;
    expect(qi::identity_matches(identity, model, error_message), "robot identity should match");

    qi::WireIdentity wrong = identity;
    wrong.joint_names[1][0] = 'X';
    expect(!qi::identity_matches(wrong, model, error_message), "joint order mismatch is rejected");

    const auto state = make_state(model, 11, 12, 1'000'000);
    qc::StateFrame decoded_state;
    expect(qi::from_wire(qi::to_wire(state), decoded_state), "StateFrame roundtrip succeeds");
    expect(decoded_state.header.session_id == 12, "StateFrame preserves session");
    expect(decoded_state.joints[3].velocity == state.joints[3].velocity,
        "StateFrame preserves joint values");

    auto bad_state = qi::to_wire(state);
    bad_state.safety_state = 255;
    expect(!qi::from_wire(bad_state, decoded_state), "unknown SafetyState is rejected");

    const auto command = make_command(model, 11, 12, 1'000'000);
    qc::CommandFrame decoded_command;
    expect(qi::from_wire(qi::to_wire(command), decoded_command),
        "CommandFrame roundtrip succeeds");
    expect(decoded_command.joints[2].mode == qc::ControlMode::JointImpedance,
        "CommandFrame preserves explicit control mode");

    auto bad_command = qi::to_wire(command);
    bad_command.joints[0].mode = 255;
    expect(!qi::from_wire(bad_command, decoded_command), "unknown ControlMode is rejected");

    qc::BaseCommand base;
    base.sequence = 3;
    base.timestamp_ns = 100;
    base.expires_at_ns = 200;
    base.source = qc::CommandSource::Navigation;
    base.priority = 4;
    base.vx = 0.5;
    qi::WireBaseCommand wire_base = qi::to_wire(base);
    qc::BaseCommand decoded_base;
    expect(qi::from_wire(wire_base, decoded_base), "BaseCommand roundtrip succeeds");
    wire_base.schema_version += 1;
    expect(!qi::from_wire(wire_base, decoded_base), "BaseCommand schema mismatch is rejected");
}

void test_latest_and_queue()
{
    qi::LatestSlot<qi::WireHeartbeat> slot;
    qi::WireHeartbeat heartbeat;
    expect(!qi::read_latest(slot, heartbeat), "empty latest slot has no value");

    heartbeat.startup_id = 1;
    qi::publish_latest(slot, heartbeat);
    heartbeat.startup_id = 2;
    qi::publish_latest(slot, heartbeat);
    std::uint64_t version = 0;
    expect(qi::read_latest(slot, heartbeat, &version), "latest slot can be read");
    expect(heartbeat.startup_id == 2 && version == 2, "latest slot overwrites old value");

    qi::SpscQueue<qi::WireModeRequest, 2> queue;
    qi::WireModeRequest first;
    first.request_id = 10;
    qi::WireModeRequest second;
    second.request_id = 11;
    expect(qi::queue_push(queue, first), "queue accepts first request");
    expect(qi::queue_push(queue, second), "queue accepts request up to capacity");
    expect(!qi::queue_push(queue, first), "full queue rejects producer");

    qi::WireModeRequest output;
    expect(qi::queue_pop(queue, output) && output.request_id == 10, "queue preserves FIFO first");
    expect(qi::queue_pop(queue, output) && output.request_id == 11, "queue preserves FIFO second");
    expect(!qi::queue_pop(queue, output), "empty queue reports no value");
}

void test_shared_memory_and_remote_io(const qc::RobotModel& model)
{
    const std::string name = "/quadruped_ipc_test_" + std::to_string(getpid());
    auto owner = qi::SharedMemory::create_owner(name, qi::make_identity(model));
    expect(owner.ok(), "shared memory owner can be created");
    if (!owner.ok())
    {
        return;
    }
    auto client = qi::SharedMemory::open_existing(name);
    expect(client.ok(), "shared memory client validates schema and opens");
    if (!client.ok())
    {
        return;
    }

    constexpr std::uint64_t startup_id = 31;
    constexpr std::uint64_t session_id = 41;
    const qc::Nanoseconds state_time = 2'000'000;
    qi::WireHeartbeat heartbeat;
    heartbeat.startup_id = startup_id;
    heartbeat.session_id = session_id;
    heartbeat.monotonic_ns = qi::monotonic_now_ns();
    heartbeat.online = 1;
    qi::publish_latest(owner.memory->layout().backend_heartbeat, heartbeat);
    qi::publish_latest(
        owner.memory->layout().state,
        qi::to_wire(make_state(model, startup_id, session_id, state_time)));

    qi::RemoteRobotIO io(*client.memory, model);
    qc::StateFrame state;
    expect(io.read_latest(state) == qc::RobotIOCode::Ok, "RemoteRobotIO reads matching state");

    const auto command = make_command(model, startup_id, session_id, state_time);
    expect(io.submit(command) == qc::RobotIOCode::Ok, "RemoteRobotIO publishes matching command");

    // latest slot 的读锁竞争只是瞬态，不得把已建立的连接误判为断开或无状态。
    owner.memory->layout().backend_heartbeat.lock.store(1, std::memory_order_release);
    expect(io.backend_online(), "busy heartbeat slot preserves cached online state");
    expect(io.submit(command) == qc::RobotIOCode::Ok,
        "busy heartbeat slot does not reject a valid command");
    owner.memory->layout().backend_heartbeat.lock.store(0, std::memory_order_release);

    owner.memory->layout().state.lock.store(1, std::memory_order_release);
    qc::StateFrame cached_state;
    expect(io.read_latest(cached_state) == qc::RobotIOCode::Ok,
        "busy state slot reuses the matching cached state");
    expect(cached_state.header.sequence == state.header.sequence,
        "cached state preserves the last valid sequence");
    owner.memory->layout().state.lock.store(0, std::memory_order_release);

    auto old_session_command = command;
    old_session_command.header.sequence += 1;
    old_session_command.header.session_id = session_id - 1;
    expect(io.submit(old_session_command) == qc::RobotIOCode::InvalidFrame,
        "old-session command is rejected");

    heartbeat.monotonic_ns = qi::monotonic_now_ns() - 600'000'000;
    qi::publish_latest(owner.memory->layout().backend_heartbeat, heartbeat);
    expect(io.read_latest(state) == qc::RobotIOCode::Disconnected,
        "expired backend heartbeat disconnects RemoteRobotIO");
}

}  // 匿名命名空间

int main()
{
    const auto loaded = quadruped::config::load_robot_model(QUADRUPED_ROBOT_CONFIG_PATH);
    if (!loaded.ok())
    {
        std::cerr << "FAIL: load black model: " << loaded.error_message << '\n';
        return 1;
    }

    test_identity_and_conversions(loaded.model);
    test_latest_and_queue();
    test_shared_memory_and_remote_io(loaded.model);

    if (failures == 0)
    {
        std::cout << "IPC tests passed\n";
    }
    return failures == 0 ? 0 : 1;
}
