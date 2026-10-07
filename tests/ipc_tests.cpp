/**
 * @file ipc_tests.cpp
 * @brief 测试共享内存 wire、数据通道、futex 通知和会话安全语义。
 */

#include "quadruped/config/robot_config.hpp"
#include "quadruped/core/constants.hpp"
#include "quadruped/ipc/conversions.hpp"
#include "quadruped/ipc/remote_robot_io.hpp"
#include "quadruped/ipc/shared_memory.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <csignal>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include <pthread.h>
#include <unistd.h>

namespace qc = quadruped::core;
namespace qi = quadruped::ipc;

namespace
{

int failures = 0;

void handle_test_signal(int)
{
}

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
    frame.target_generated_at_ns = timestamp_ns;
    frame.target_expires_at_ns = frame.expires_at_ns;
    frame.joint_count = model.joint_count;
    frame.motion_mode = qc::MotionMode::Stand;
    frame.source = qc::CommandSource::Test;
    for (std::size_t i = 0; i < frame.joint_count; ++i)
    {
        const auto& limits = model.joints[i].limits;
        frame.joints[i].mode = qc::ControlMode::JointImpedance;
        frame.joints[i].target_position = limits.position_limited
            ? 0.5 * (limits.min_position + limits.max_position)
            : 0.01 * static_cast<double>(i);
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
    expect(decoded_command.target_generated_at_ns == command.target_generated_at_ns &&
            decoded_command.target_expires_at_ns == command.target_expires_at_ns,
        "CommandFrame preserves semantic target timing bit-exact");

    auto old_frame = qi::to_wire(command);
    old_frame.schema_version = 1;
    expect(!qi::from_wire(old_frame, decoded_command),
        "frame schema 1 is rejected by schema 2 conversion");

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

    qi::WireEvent event;
    expect(qi::queue_push(queue, first) && qi::queue_push(queue, second),
        "queue can be filled again for notification test");
    const std::uint32_t full_sequence = qi::event_sequence(event);
    expect(!qi::queue_push_and_notify(queue, first, event),
        "full queue does not publish an event");
    expect(qi::event_sequence(event) == full_sequence,
        "full queue leaves event sequence unchanged");
}

void test_event_waiting()
{
    qi::WireEvent event;
    const std::uint32_t initial = qi::event_sequence(event);
    expect(qi::wait_event(event, initial, 1'000'000) == qi::EventWaitResult::TimedOut,
        "event wait reports timeout");

    qi::notify_event(event);
    expect(qi::wait_event(event, initial, 1'000'000) == qi::EventWaitResult::Changed,
        "notification before wait is not lost");

    const std::uint32_t waiting_sequence = qi::event_sequence(event);
    std::atomic<qi::EventWaitResult> wake_result{qi::EventWaitResult::Failed};
    std::thread waiter([&]()
    {
        wake_result.store(qi::wait_event(event, waiting_sequence, 1'000'000'000));
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    qi::notify_event(event);
    waiter.join();
    expect(wake_result.load() == qi::EventWaitResult::Changed,
        "waiting consumer is woken by notification");

    qi::LatestSlot<qi::WireHeartbeat> slot;
    qi::WireHeartbeat published;
    published.startup_id = 77;
    const std::uint32_t data_sequence = qi::event_sequence(event);
    qi::publish_latest_and_notify(slot, published, event);
    qi::WireHeartbeat received;
    expect(qi::wait_event(event, data_sequence, 1'000'000) == qi::EventWaitResult::Changed &&
            qi::read_latest(slot, received) && received.startup_id == 77,
        "latest value is visible after event notification");

    const std::uint32_t merged_sequence = qi::event_sequence(event);
    published.startup_id = 78;
    qi::publish_latest_and_notify(slot, published, event);
    published.startup_id = 79;
    qi::publish_latest_and_notify(slot, published, event);
    expect(qi::wait_event(event, merged_sequence, 1'000'000) == qi::EventWaitResult::Changed &&
            qi::read_latest(slot, received) && received.startup_id == 79,
        "multiple notifications can merge while latest value is preserved");

    struct sigaction action{};
    action.sa_handler = handle_test_signal;
    sigemptyset(&action.sa_mask);
    sigaction(SIGUSR1, &action, nullptr);
    const std::uint32_t interrupted_sequence = qi::event_sequence(event);
    std::atomic<bool> waiting{false};
    wake_result.store(qi::EventWaitResult::Failed);
    std::thread interrupted_waiter([&]()
    {
        waiting.store(true, std::memory_order_release);
        wake_result.store(qi::wait_event(event, interrupted_sequence, 1'000'000'000));
    });
    while (!waiting.load(std::memory_order_acquire))
    {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    pthread_kill(interrupted_waiter.native_handle(), SIGUSR1);
    interrupted_waiter.join();
    expect(wake_result.load() == qi::EventWaitResult::Interrupted,
        "signal interruption is reported separately");
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

    const std::uint32_t cross_mapping_sequence =
        qi::event_sequence(client.memory->layout().motion_event);
    std::atomic<qi::EventWaitResult> cross_mapping_result{qi::EventWaitResult::Failed};
    std::thread cross_mapping_waiter([&]()
    {
        cross_mapping_result.store(qi::wait_event(
            client.memory->layout().motion_event,
            cross_mapping_sequence,
            1'000'000'000));
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    qi::notify_event(owner.memory->layout().motion_event);
    cross_mapping_waiter.join();
    expect(cross_mapping_result.load() == qi::EventWaitResult::Changed,
        "futex wakes a waiter through another shared mapping");

    owner.memory->layout().identity.wire_schema_version = 2;
    const auto incompatible = qi::SharedMemory::open_existing(name);
    expect(!incompatible.ok(), "wire schema 2 mapping is rejected by schema 3 client");
    owner.memory->layout().identity.wire_schema_version = qi::kWireSchemaVersion;

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
    expect(io.clock_now_ns() == -1, "remote clock requires a sampled backend session");
    qc::StateFrame state;
    std::uint64_t state_version = 0;
    expect(io.read_latest(state, state_version) == qc::RobotIOCode::Ok,
        "RemoteRobotIO reads matching state");
    expect(state_version == 1, "RemoteRobotIO returns the state slot version");
    expect(io.clock_now_ns() == state_time,
        "remote command clock uses backend simulation time, not host monotonic time");

    const auto next_state = make_state(model, startup_id, session_id, state_time + 2'000'000);
    qi::publish_latest(owner.memory->layout().state, qi::to_wire(next_state));
    expect(io.clock_now_ns() == next_state.header.timestamp_ns,
        "remote clock observes backend advancement during policy inference");
    qi::publish_latest(owner.memory->layout().state, qi::to_wire(state));

    const auto command = make_command(model, startup_id, session_id, state_time);
    expect(io.submit(command) == qc::RobotIOCode::Ok, "RemoteRobotIO publishes matching command");

    // latest slot 的读锁竞争只是瞬态，不得把已建立的连接误判为断开或无状态。
    owner.memory->layout().backend_heartbeat.lock.store(1, std::memory_order_release);
    expect(io.backend_online(), "busy heartbeat slot preserves cached online state");
    expect(io.submit(command) == qc::RobotIOCode::Ok,
        "busy heartbeat slot does not reject a valid command");
    owner.memory->layout().backend_heartbeat.lock.store(0, std::memory_order_release);

    owner.memory->layout().state.lock.store(1, std::memory_order_release);
    expect(io.clock_now_ns() == state_time,
        "busy state slot preserves backend clock domain without wall-clock extrapolation");
    qc::StateFrame cached_state;
    expect(io.read_latest(cached_state) == qc::RobotIOCode::Ok,
        "busy state slot reuses the matching cached state");
    expect(cached_state.header.sequence == state.header.sequence,
        "cached state preserves the last valid sequence");
    std::uint64_t cached_version = 0;
    expect(io.read_latest(cached_state, cached_version) == qc::RobotIOCode::Ok &&
            cached_version == state_version,
        "cached state preserves its matching slot version");
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

    const std::uint32_t close_sequence =
        qi::event_sequence(client.memory->layout().gateway_event);
    std::atomic<qi::EventWaitResult> close_result{qi::EventWaitResult::Failed};
    std::thread close_waiter([&]()
    {
        close_result.store(qi::wait_event(
            client.memory->layout().gateway_event,
            close_sequence,
            1'000'000'000));
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    owner.memory.reset();
    close_waiter.join();
    expect(close_result.load() == qi::EventWaitResult::Changed &&
            client.memory->layout().ready.load(std::memory_order_acquire) == 0,
        "owner shutdown clears ready and wakes event waiters");
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
    test_event_waiting();
    test_shared_memory_and_remote_io(loaded.model);

    if (failures == 0)
    {
        std::cout << "IPC tests passed\n";
    }
    return failures == 0 ? 0 : 1;
}
