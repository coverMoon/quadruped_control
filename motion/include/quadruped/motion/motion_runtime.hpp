/**
 * @file motion_runtime.hpp
 * @brief 定义不依赖外部框架的基础运动运行时 MotionRuntime。
 */

#pragma once

#include "quadruped/core/controller_config.hpp"
#include "quadruped/core/robot_io.hpp"
#include "quadruped/core/robot_model.hpp"
#include "quadruped/core/types.hpp"
#include "quadruped/motion/behavior_config.hpp"
#include "quadruped/motion/rl_controller.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace quadruped::motion
{

// MotionRuntime 一次周期更新的输入；base_command 和 request 都可以为空。
// now_ns 必须与 StateFrame 使用同一单调时钟，第一版取最新状态的 timestamp。
struct MotionUpdateInput
{
    core::Nanoseconds now_ns{0};

    // 最新有效的机体速度命令；启动 rl_locomotion 时必须存在且未过期。
    // 运行中命令失效时按零速观测。
    const core::BaseCommand* base_command{nullptr};

    // 本周期待处理的一次性请求；同一请求可以在后续周期重试。
    const core::ModeRequest* request{nullptr};
};

// 单周期诊断快照只包含标量和枚举，可由低频日志线程按需抽样；MotionRuntime 本身不写文件。
struct MotionDiagnostics
{
    std::uint64_t startup_id{0};
    std::uint64_t session_id{0};
    std::uint64_t state_sequence{0};
    std::uint64_t command_sequence{0};
    std::uint64_t effective_command_sequence{0};
    core::Nanoseconds state_age_ns{0};
    std::uint64_t dropped_state_frames{0};
    std::uint64_t rejected_command_frames{0};
    core::Nanoseconds inference_elapsed_ns{0};
};

// MotionRuntime 一次周期更新的输出，全部按值返回，不持有内部指针。
struct MotionUpdateOutput
{
    // 本周期 read_latest() 的结果；Ok 表示拿到了完整状态。
    core::RobotIOCode read_code{core::RobotIOCode::NoData};

    // 本周期是否向 RobotIO 提交了命令，以及 submit() 的结果。
    bool submitted{false};
    core::RobotIOCode submit_code{core::RobotIOCode::Ok};

    // 当前低频状态快照。
    core::MotionStatus status{};

    // 供日志、回放对比和状态界面抽样的诊断字段。
    MotionDiagnostics diagnostics{};

    // 输入中存在请求时对应的处理结果；has_result 为 false 时 result 无意义。
    bool has_result{false};
    core::ModeResult result{};

    // 本周期在输入请求处理之后产生的全部终态结果，固定容量 2 交付（例如被中断的
    // 旧活动请求，或输入请求重试与动作完成发生在同一周期时的 Completed）。
    // result 是请求处理时刻的快照；同一请求随后产生的终态只经本数组交付。
    // 最坏情况一周期至多两项：旧活动请求终止 + 新动作随后失败，不需要动态分配。
    // 终态在产生周期必须通过本数组或 result 字段交付给调用方；
    // 内部固定容量历史只用于重试查询，淘汰后查询返回 Rejected(unknown)。
    static constexpr std::size_t kMaxResultEvents = 2;
    std::size_t result_event_count{0};
    std::array<core::ModeResult, kMaxResultEvents> result_events{};
};

// 基础运动状态机：Passive、两段 GetUp、Stand、GetDown 和 EnterPassive。
// 不创建线程、不推进物理、不读取输入设备；由外部按控制周期调用 update()。
class MotionRuntime
{
public:
    // create() 的返回结果：成功时 runtime 非空，失败时 error_message 含有可读原因。
    struct CreateResult
    {
        std::unique_ptr<MotionRuntime> runtime{};
        std::string error_message{};

        [[nodiscard]] bool ok() const noexcept
        {
            return runtime != nullptr;
        }
    };

    MotionRuntime(const MotionRuntime&) = delete;
    MotionRuntime& operator=(const MotionRuntime&) = delete;
    ~MotionRuntime() = default;

    // 构造前校验 RobotModel 和 ControllerConfig；不合法的配置拒绝创建，
    // 不允许运行时使用部分默认值继续主动控制。
    static CreateResult create(core::RobotModel model, core::ControllerConfig config);

    // 配置共用 Retry；数组必须与 RobotModel 的名称、顺序和限制一致。
    bool configure_retry(RetryConfig config, std::string& error_message);

    // 注册 Car、Bridge 或 Low-bar 配置；同名配置只能注册一次。
    bool configure_fixed_drive(FixedDriveConfig config, std::string& error_message);

    // 配置 Event chain 并检查姿态、轮组、行程和关节限制。
    bool configure_event_chain(EventChainConfig config, std::string& error_message);

    // 注册一个由调用方持有的同步策略。策略对象必须比 MotionRuntime 生命周期长。
    // 注册不会改变当前策略；策略名称在同一运行时内必须唯一。
    bool register_policy(RlConfig config, Policy& policy, std::string& error_message);

    // 绑定一个由调用方持有的同步策略并立即选为当前策略。
    // 该接口保留给单策略调用方，重复名称会被拒绝；运行中切换使用 SwitchPolicy。
    bool attach_policy(RlConfig config, Policy& policy, std::string& error_message);

    // 设置按键 toggle 使用的策略循环；策略必须已经注册且名称唯一。
    bool set_policy_cycle(
        const std::vector<std::string>& policy_names,
        std::uint32_t posture_transition_cycles,
        std::string& error_message);

    // 返回当前策略在循环中的下一个策略；没有配置循环时返回空字符串。
    [[nodiscard]] std::string next_policy_name() const;

    // 执行一个控制周期：读取最新状态、处理请求、生成并提交命令。
    // 不抛出异常；所有 RobotIO 错误都通过输出结果表达。
    MotionUpdateOutput update(core::RobotIO& io, const MotionUpdateInput& input);

    // 查询指定编号请求的结果：活动请求返回实时状态，已终结请求从固定容量历史
    // 返回终态；已淘汰或从未出现的编号返回 Rejected(unknown)。
    // 终态在产生周期已经通过更新输出交付，此查询只是重试和界面展示的旁路。
    [[nodiscard]] core::ModeResult query_result(std::uint64_t request_id) const;

private:
    // 单周期状态读取与校验的结果，供 update() 分解出的步骤复用。
    struct StateRead
    {
        core::StateFrame frame{};
        core::RobotIOCode code{core::RobotIOCode::NoData};
        core::Nanoseconds now_ns{0};
        bool usable{false};

        // 本周期是否从已建立会话切换到新会话；切换周期只交付旧请求终态，
        // 不处理输入请求，调用方在后续周期重新提交。
        bool session_changed{false};
        std::string failure_reason{};
    };

    MotionRuntime(core::RobotModel model, core::ControllerConfig config);

    // 会话建立或会话号变化时回到 Passive，清空未完成请求、rest_pose 和命令序号。
    // 返回 true 表示从已建立会话切换到了新会话。
    bool track_session(const core::StateFrame& state);

    // 读取并校验最新状态，同时更新会话跟踪、关节位置和安全状态。
    StateRead read_state(core::RobotIO& io, core::Nanoseconds now_ns);

    // 处理本周期的请求：新请求分派，相同编号返回当前结果，旧编号查询终态。
    // 去重查询优先于字段校验，保证过期重试仍能取得原请求结果。
    core::ModeResult handle_request(
        const core::ModeRequest& request,
        bool state_usable,
        bool base_command_usable);

    // 处理输入请求并写入输出：非法请求直接拒绝，合法请求进入编号去重；
    // 会话切换周期不处理任何输入请求，避免旧请求被自动重启到新会话。
    void handle_input_request(
        const MotionUpdateInput& input,
        const StateRead& state,
        bool base_command_usable,
        MotionUpdateOutput& output);

    // 分派一个编号严格大于最近编号的新请求；按请求类型拆到具体处理函数。
    core::ModeResult dispatch_request(
        const core::ModeRequest& request,
        bool state_usable,
        bool base_command_usable);

    core::ModeResult dispatch_enter_passive(const core::ModeRequest& request);
    core::ModeResult dispatch_getup(const core::ModeRequest& request, bool state_usable);
    core::ModeResult dispatch_stand(const core::ModeRequest& request);
    core::ModeResult dispatch_start_behavior(
        const core::ModeRequest& request,
        bool state_usable,
        bool base_command_usable);
    core::ModeResult dispatch_getdown(const core::ModeRequest& request, bool state_usable);
    core::ModeResult dispatch_switch_policy(
        const core::ModeRequest& request, bool state_usable);
    core::ModeResult dispatch_reset_fault(const core::ModeRequest& request);

    // 接受 GetUp：必要时记录 rest_pose，并从当前关节位置开始第一段插值。
    core::ModeResult accept_getup(std::uint64_t request_id);

    // 接受 GetDown：从当前关节位置向 rest_pose 插值。
    core::ModeResult accept_getdown(std::uint64_t request_id);

    // 主动模式（GetUp/Stand/GetDown）的每周期前置条件；失败时写出可读原因。
    bool check_active_preconditions(std::string& reason) const;

    // 主动模式分支的一个周期：条件失败时回到 Passive 并提交 Disabled；
    // 否则推进插值或保持姿态并提交主动命令。返回 true 表示本周期发生了失败回退。
    bool run_active_mode(
        core::RobotIO& io,
        const StateRead& state,
        MotionUpdateOutput& output);

    // Running 模式按固定 decimation 执行策略，其余控制周期保持最近关节目标。
    bool run_rl_mode(
        core::RobotIO& io,
        const StateRead& state,
        const core::BaseCommand* base_command,
        MotionUpdateOutput& output);

    // Retry 插值完成后继续锁定恢复姿态，不自动退出 Running。
    bool run_retry_mode(
        core::RobotIO& io,
        const StateRead& state,
        MotionUpdateOutput& output);

    // 以固定腿姿态和显式左右轮映射执行 Car、Bridge、Low-bar 差速轮控制。
    bool run_fixed_drive_mode(
        core::RobotIO& io,
        const StateRead& state,
        const core::BaseCommand* base_command,
        MotionUpdateOutput& output);

    // 按 rl_sar 的 pose、drive、pose_drive 语义推进 Event chain。
    bool run_event_chain_mode(
        core::RobotIO& io,
        const StateRead& state,
        MotionUpdateOutput& output);

    // 策略切换期间只输出固定位置阻抗，不调用旧策略推理。
    bool run_policy_transition(
        core::RobotIO& io,
        const StateRead& state,
        MotionUpdateOutput& output);

    // 将已注册策略设为当前策略，并清空其观测历史和动作目标。
    void activate_policy(std::size_t policy_index);

    [[nodiscard]] std::size_t find_policy(const std::string& name) const;

    [[nodiscard]] std::size_t find_fixed_drive(const std::string& name) const;

    [[nodiscard]] bool pose_close_to_policy(
        const RlConfig& config, const std::array<double, core::kMaxJoints>& positions) const;

    // 主动模式条件失效：活动请求转为 Failed 终态，回到 Passive 并记录错误。
    void fail_active_motion(const std::string& reason);

    // 推进当前主动模式一个成功控制周期，填写本周期的关节目标位置。
    // 返回 false 表示本周期应提交 Disabled（GetDown 完成的这一周期）。
    bool advance_active_motion(std::array<double, core::kMaxJoints>& positions);

    // 完成当前动作对应的活动请求（Accepted/Running 转为 Completed 终态）。
    void complete_active_request(const char* message);

    // 中止当前活动请求（转为 Failed 终态），不改变运动模式；由中断方负责切模式。
    void abort_active_request(const std::string& reason);

    // 把终态结果写入固定容量环形存储；容量满时覆盖最旧条目，不无界保存历史。
    // 覆盖前结果已在产生周期通过更新输出交付，此处仅为重试查询服务。
    void store_terminal_result(const core::ModeResult& result);

    // 在终态存储中查找指定编号的结果。
    [[nodiscard]] bool find_terminal_result(
        std::uint64_t request_id,
        core::ModeResult& result) const;

    // 追加本周期待交付的终态事件；update() 结束时复制到输出。
    void append_result_event(const core::ModeResult& result);

    // 一次命令提交的全部输入，集中传递避免位置参数过多。
    struct CommandSubmission
    {
        const std::array<double, core::kMaxJoints>& positions;
        bool use_impedance{true};
        core::Nanoseconds now_ns{0};
        const std::array<double, core::kMaxJoints>* velocities{nullptr};
        const std::array<double, core::kMaxJoints>* kp{nullptr};
        const std::array<double, core::kMaxJoints>* kd{nullptr};
        core::CommandSource source{core::CommandSource::None};
    };

    // 按当前模式填充命令的关节数组并提交；use_impedance 为 false 时提交 Disabled。
    core::RobotIOCode submit_command(core::RobotIO& io, const CommandSubmission& submission);

    core::RobotModel model_;
    core::ControllerConfig config_;

    struct RegisteredPolicy
    {
        bool registered{false};
        RlConfig config{};
        Policy* policy{nullptr};
        std::unique_ptr<RlController> controller{};
    };

    // 策略数量是启动期低频配置，容量覆盖 rl_sar 当前 blackW 的五项循环。
    static constexpr std::size_t kMaxPolicies = 8;
    static constexpr std::size_t kInvalidPolicyIndex = kMaxPolicies;
    static constexpr std::uint32_t kDefaultPolicyTransitionCycles = 20;
    static constexpr double kPolicyPoseTolerance = 0.15;

    std::array<RegisteredPolicy, kMaxPolicies> policies_{};
    std::array<std::string, kMaxPolicies> policy_cycle_{};
    std::size_t policy_cycle_size_{0};
    std::uint32_t policy_transition_cycles_{kDefaultPolicyTransitionCycles};
    std::size_t current_policy_index_{kInvalidPolicyIndex};
    std::size_t pending_policy_index_{kInvalidPolicyIndex};
    bool policy_transition_active_{false};
    bool event_to_rl_transition_{false};
    bool fixed_drive_to_rl_transition_{false};
    RlController* rl_controller_{nullptr};
    Policy* policy_{nullptr};
    std::string policy_name_{};
    std::uint32_t rl_control_cycle_{0};
    RlController::CommandResult rl_command_{};
    core::Nanoseconds latest_inference_elapsed_ns_{0};

    RetryConfig retry_config_{};
    static constexpr std::size_t kMaxFixedDriveConfigs = 3;
    static constexpr std::size_t kInvalidFixedDriveIndex = kMaxFixedDriveConfigs;
    std::array<FixedDriveConfig, kMaxFixedDriveConfigs> fixed_drive_configs_{};
    std::array<bool, kMaxFixedDriveConfigs> fixed_drive_configured_{};
    std::size_t active_fixed_drive_index_{kInvalidFixedDriveIndex};
    EventChainConfig event_chain_config_{};
    bool retry_configured_{false};
    bool event_chain_configured_{false};
    bool retry_locked_{false};
    std::size_t event_index_{0};
    std::uint32_t event_cycle_{0};
    std::uint32_t event_hold_cycle_{0};
    bool event_initialized_{false};
    bool event_motion_complete_{false};
    bool event_chain_complete_{false};
    std::array<double, core::kMaxJoints> event_active_pose_{};
    std::array<double, core::kMaxJoints> event_segment_start_{};
    std::array<double, core::kMaxJoints> event_drive_start_{};

    core::MotionMode mode_{core::MotionMode::Passive};

    // 当前执行侧会话标识；来自最新有效 StateFrame，0 表示尚未见到状态。
    std::uint64_t startup_id_{0};
    std::uint64_t session_id_{0};

    // 会话内单调递增的命令序号；新会话从 1 重新开始。
    std::uint64_t command_sequence_{0};

    // 本会话最近处理过的请求编号，用于新旧请求判定。
    std::uint64_t latest_request_id_{0};

    // 当前主动动作对应的请求及其实时结果；同一时刻至多一个主动动作。
    // 新请求（包括被拒绝的）只记录自身结果，不覆盖活动请求。
    std::uint64_t active_request_id_{0};
    core::ModeRequestType active_request_type_{core::ModeRequestType::EnterPassive};
    core::ModeResult active_result_{};
    bool has_active_request_{false};

    // 固定容量的终态结果环形存储，保存最近的 Completed/Failed/Rejected 结果，
    // 供旧编号查询终态；所有终态都已先在其产生周期通过更新输出交付，
    // 淘汰不会丢失未交付的结果。
    static constexpr std::size_t kMaxTerminalResults = 8;
    std::array<core::ModeResult, kMaxTerminalResults> terminal_results_{};
    std::size_t terminal_count_{0};
    std::size_t terminal_next_{0};

    // 本周期待交付的终态事件缓冲；update() 开始时清空，结束时复制到输出。
    std::array<core::ModeResult, MotionUpdateOutput::kMaxResultEvents> pending_result_events_{};
    std::size_t pending_result_event_count_{0};

    // 本会话首次 GetUp 前记录的落地关节姿态，单位为 rad；GetDown 返回该姿态。
    std::array<double, core::kMaxJoints> rest_pose_{};
    bool has_rest_pose_{false};

    // 当前插值段的起点、终点和进度；主动模式按成功控制周期推进。
    std::array<double, core::kMaxJoints> interp_start_{};
    std::array<double, core::kMaxJoints> interp_target_{};
    std::uint32_t interp_total_cycles_{1};
    std::uint32_t interp_elapsed_cycles_{0};

    // GetUp 当前是否处于第二段（预起立姿态到默认站姿）。
    bool getup_second_phase_{false};

    // 最新一份通过校验的关节位置，用于插值起点和 rest_pose 记录。
    std::array<double, core::kMaxJoints> current_positions_{};
    core::SafetyState latest_safety_{core::SafetyState::Unknown};
    bool joints_ready_{false};

    core::MotionStatus status_{};
};

// 面向界面和日志的 MotionMode 名称；不用于控制分支。
[[nodiscard]] const char* motion_mode_name(core::MotionMode mode) noexcept;

}  // 命名空间 quadruped::motion
