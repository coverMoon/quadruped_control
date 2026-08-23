/**
 * @file wire_protocol.hpp
 * @brief 定义三进程本机共享内存中的固定容量显式 wire schema。
 */

#pragma once

#include "quadruped/core/constants.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace quadruped::ipc
{

constexpr std::uint32_t kWireMagic = 0x51435043U;
constexpr std::uint32_t kWireSchemaVersion = 1;
constexpr std::size_t kWireNameCapacity = 64;
constexpr std::size_t kWireMessageCapacity = 256;
constexpr std::size_t kRequestQueueCapacity = 32;
constexpr std::size_t kResultQueueCapacity = 64;
constexpr std::size_t kControlQueueCapacity = 8;

struct WireJointState
{
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double temperature_c{0.0};
    std::uint32_t error_code{0};
    std::int64_t age_ns{0};
    std::uint8_t online{0};
    std::uint8_t valid{0};
};

struct WireImuState
{
    std::array<double, 4> orientation{1.0, 0.0, 0.0, 0.0};
    std::array<double, 3> angular_velocity{};
    std::array<double, 3> linear_acceleration{};
    std::int64_t age_ns{0};
    std::uint8_t valid{0};
};

struct WireStateFrame
{
    std::uint32_t schema_version{0};
    std::uint64_t startup_id{0};
    std::uint64_t session_id{0};
    std::uint64_t sequence{0};
    std::int64_t timestamp_ns{0};
    std::uint32_t joint_count{0};
    std::array<WireJointState, core::kMaxJoints> joints{};
    WireImuState imu{};
    std::uint8_t safety_state{0};
    std::uint64_t last_accepted_command_sequence{0};
    std::uint64_t effective_command_sequence{0};
};

struct WireJointCommand
{
    std::uint8_t mode{0};
    double target_position{0.0};
    double target_velocity{0.0};
    double kp{0.0};
    double kd{0.0};
    double feedforward_effort{0.0};
};

struct WireCommandFrame
{
    std::uint32_t schema_version{0};
    std::uint64_t startup_id{0};
    std::uint64_t session_id{0};
    std::uint64_t sequence{0};
    std::int64_t timestamp_ns{0};
    std::int64_t expires_at_ns{0};
    std::uint32_t joint_count{0};
    std::array<WireJointCommand, core::kMaxJoints> joints{};
    std::uint8_t motion_mode{0};
    std::uint8_t source{0};
};

struct WireBaseCommand
{
    std::uint32_t schema_version{0};
    std::uint64_t startup_id{0};
    std::uint64_t session_id{0};
    std::uint64_t sequence{0};
    std::int64_t timestamp_ns{0};
    std::int64_t expires_at_ns{0};
    std::uint8_t source{0};
    std::uint8_t priority{0};
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
};

struct WireModeRequest
{
    std::uint32_t schema_version{0};
    std::uint64_t startup_id{0};
    std::uint64_t session_id{0};
    std::uint64_t request_id{0};
    std::int64_t timestamp_ns{0};
    std::uint8_t type{0};
    std::array<char, kWireNameCapacity> behavior_name{};
    std::array<char, kWireNameCapacity> policy_name{};
};

struct WireModeResult
{
    std::uint32_t schema_version{0};
    std::uint64_t startup_id{0};
    std::uint64_t session_id{0};
    std::uint64_t request_id{0};
    std::uint8_t state{0};
    std::array<char, kWireMessageCapacity> message{};
};

struct WireMotionStatus
{
    std::uint32_t schema_version{0};
    std::uint64_t startup_id{0};
    std::uint64_t session_id{0};
    std::uint8_t mode{0};
    std::uint8_t active_source{0};
    std::array<char, kWireNameCapacity> behavior_name{};
    std::array<char, kWireNameCapacity> behavior_phase{};
    std::array<char, kWireNameCapacity> policy_name{};
    std::array<char, kWireMessageCapacity> error_message{};
    std::uint8_t policy_ready{0};
    std::array<double, 3> command_limits{};
};

struct WireRobotIOStatus
{
    std::uint32_t schema_version{0};
    std::uint64_t startup_id{0};
    std::uint64_t session_id{0};
    std::uint8_t state{0};
    std::uint64_t latest_state_sequence{0};
    std::uint64_t latest_command_sequence{0};
    std::uint64_t dropped_state_frames{0};
    std::uint64_t rejected_command_frames{0};
};

enum class WireControlType : std::uint8_t
{
    Reset = 0,
    PauseToggle = 1,
    SimulationStateReset = 2,
};

struct WireControlRequest
{
    std::uint32_t schema_version{0};
    std::uint64_t startup_id{0};
    std::uint64_t session_id{0};
    std::uint64_t request_id{0};
    std::uint8_t type{0};
};

struct WireControlResult
{
    std::uint32_t schema_version{0};
    std::uint64_t startup_id{0};
    std::uint64_t session_id{0};
    std::uint64_t request_id{0};
    std::uint8_t success{0};
    std::array<char, kWireMessageCapacity> message{};
};

struct WireIdentity
{
    std::uint32_t magic{0};
    std::uint32_t wire_schema_version{0};
    std::uint32_t frame_schema_version{0};
    std::uint32_t joint_count{0};
    std::array<char, kWireNameCapacity> robot_name{};
    std::array<std::array<char, kWireNameCapacity>, core::kMaxJoints> joint_names{};
};

struct WireHeartbeat
{
    std::uint64_t startup_id{0};
    std::uint64_t session_id{0};
    std::int64_t monotonic_ns{0};
    std::uint8_t online{0};
};

template<typename T>
struct alignas(64) LatestSlot
{
    mutable std::atomic<std::uint32_t> lock{0};
    std::atomic<std::uint64_t> version{0};
    T value{};
};

template<typename T, std::size_t Capacity>
struct alignas(64) SpscQueue
{
    std::atomic<std::uint64_t> write_index{0};
    std::atomic<std::uint64_t> read_index{0};
    std::array<T, Capacity> entries{};
};

struct SharedLayout
{
    std::atomic<std::uint32_t> ready{0};
    WireIdentity identity{};
    LatestSlot<WireHeartbeat> backend_heartbeat{};
    LatestSlot<WireHeartbeat> motion_heartbeat{};
    LatestSlot<WireHeartbeat> gateway_heartbeat{};
    LatestSlot<WireStateFrame> state{};
    LatestSlot<WireCommandFrame> command{};
    LatestSlot<WireBaseCommand> base_command{};
    LatestSlot<WireMotionStatus> motion_status{};
    LatestSlot<WireRobotIOStatus> robot_io_status{};
    SpscQueue<WireModeRequest, kRequestQueueCapacity> requests{};
    SpscQueue<WireModeResult, kResultQueueCapacity> results{};
    SpscQueue<WireControlRequest, kControlQueueCapacity> control_requests{};
    SpscQueue<WireControlResult, kControlQueueCapacity> control_results{};
};

static_assert(std::is_trivially_copyable_v<WireStateFrame>);
static_assert(std::is_trivially_copyable_v<WireCommandFrame>);
static_assert(std::is_trivially_copyable_v<WireModeRequest>);
static_assert(std::is_trivially_copyable_v<WireModeResult>);

}  // 命名空间 quadruped::ipc
