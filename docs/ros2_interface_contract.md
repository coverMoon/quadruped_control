# ROS 2 顶层接口契约

本文件冻结阶段 5 的 ROS 2 消息、动作、服务、命名和请求语义。阶段 6 已由
`adapters/ros2/quadruped_gateway/` 实现顶层 gateway，并通过本机 IPC 连接独立的 motiond
和 mujoco_backendd；接口字段和请求语义仍以本文件为准。

## 1. 包和主题命名

接口包名称为 `quadruped_interfaces`。阶段 6 的 `ros2_gateway` 使用以下名称：

| 类型 | 名称 | ROS 类型 | 语义 |
| --- | --- | --- | --- |
| 连续命令 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 最新机体速度目标 |
| 运动状态 | `/motion/status` | `quadruped_interfaces/msg/MotionStatus` | MotionRuntime 低频状态 |
| RobotIO 状态 | `/robot_io/status` | `quadruped_interfaces/msg/RobotIOStatus` | 后端连接和帧统计 |
| 状态诊断 | `/state/diagnostic` | `quadruped_interfaces/msg/StateDiagnostic` | StateFrame 低频诊断子集 |
| 终态事件 | `/motion/result` | `quadruped_interfaces/msg/ModeResult` | 一次性请求终态事件 |

一次性控制入口使用以下接口：

| 入口 | ROS 类型 | 目标 |
| --- | --- | --- |
| `/motion/get_up` | `quadruped_interfaces/action/GetUp` | 执行起立流程 |
| `/motion/get_down` | `quadruped_interfaces/action/GetDown` | 执行趴下流程 |
| `/motion/start_behavior` | `quadruped_interfaces/action/StartBehavior` | 启动指定行为 |
| `/motion/switch_policy` | `quadruped_interfaces/action/SwitchPolicy` | 切换指定策略 |
| `/motion/enter_passive` | `quadruped_interfaces/srv/EnterPassive` | 立即进入 Passive |
| `/motion/reset_fault` | `quadruped_interfaces/srv/ResetFault` | 请求复位可软件复位的故障 |

主题和服务名属于顶层约定；接口包本身不创建节点，也不注册 executor 或 callback。

## 2. ROS 字段到 core 字段的映射

### 2.1 `/cmd_vel`

`geometry_msgs/msg/Twist` 只使用以下三个字段：

| ROS 字段 | `core::BaseCommand` 字段 | 单位 |
| --- | --- | --- |
| `twist.linear.x` | `vx` | m/s |
| `twist.linear.y` | `vy` | m/s |
| `twist.angular.z` | `wz` | rad/s |

`linear.z`、`angular.x` 和 `angular.y` 当前不进入控制语义。后续 adapter 应在收到非零
未支持字段时记录诊断并按项目策略拒绝或忽略，不能把它们映射到未知 core 字段。

adapter 必须：

- 将 `source` 固定为 `core::CommandSource::Navigation`；
- 为每个来源维护严格递增且非零的 `sequence`；
- 使用本机 monotonic clock 生成 `timestamp_ns`；
- gateway 使用独立的 `cmd_vel_timeout_ns` ROS 参数生成 `expires_at_ns`，默认
  200,000,000 ns（200 ms）；
- 该超时只约束 `BaseCommand`，不得复用 10 ms 级的 `CommandFrame` 有效期；
- 不把 ROS wall time 或 `builtin_interfaces/msg/Time` 直接写入 core 时间戳；
- 只保留同一来源的最新命令，旧消息允许丢弃；
- 只通过后续 motion 层提交 `BaseCommand`，ROS callback 不直接调用
  `MotionRuntime::update()` 或 `RobotIO::submit()`。

当前 black 配置的控制周期为 5 ms、命令有效期为 10 ms。这个数值来自
`configs/controllers/black.yaml`，不是 ROS 接口的固定默认值；不同机器人必须使用对应
`ControllerConfig` 的有效期。

命令在 `now_ns > expires_at_ns` 时过期。过期后 adapter 必须提供零速语义：清除当前有效
速度或生成同一 `Navigation` 来源的零速最新值，但不能继续沿用过期速度。ROS 进程断开不应
阻塞 motion 控制周期，motion 层仍按过期规则运行。

### 2.2 `MotionStatus`

`quadruped_interfaces/msg/MotionStatus` 的字段按下表直接对应：

| ROS 字段 | core 字段 |
| --- | --- |
| `mode` | `core::MotionStatus::mode`，数值对应 `MotionMode` |
| `active_source` | `core::MotionStatus::active_source`，数值对应 `CommandSource` |
| `behavior_name` | `behavior_name` |
| `behavior_phase` | `behavior_phase` |
| `policy_name` | `policy_name` |
| `error_message` | `error_message` |
| `policy_ready` | `policy_ready` |

消息中的常量冻结当前 core 枚举值。`behavior_phase` 只用于诊断，不得被 ROS 节点用作
控制分支；行为仍由 `behavior_name` 和 core 的请求语义选择。

### 2.3 `RobotIOStatus`

| ROS 字段 | core 字段 |
| --- | --- |
| `state` | `core::RobotIOStatus::state`，数值对应 `RobotIOState` |
| `latest_state_sequence` | 同名字段 |
| `latest_command_sequence` | 同名字段 |
| `dropped_state_frames` | 同名字段 |
| `rejected_command_frames` | 同名字段 |

### 2.4 `StateDiagnostic`

该消息只发布 `StateFrame` 的低频诊断字段，不承载高频关节数组，也不作为控制输入：

| ROS 字段 | `core::StateFrame` 字段 |
| --- | --- |
| `schema_version` | `header.schema_version` |
| `startup_id` | `header.startup_id` |
| `session_id` | `header.session_id` |
| `sequence` | `header.sequence` |
| `timestamp_ns` | `header.timestamp_ns`，单调 ns |
| `joint_count` | `joint_count` |
| `safety_state` | `safety_state`，数值对应 `SafetyState` |
| `last_accepted_command_sequence` | 同名字段 |
| `effective_command_sequence` | 同名字段 |
| `imu_valid` | `imu.valid` |
| `imu_orientation_{w,x,y,z}` | `imu.orientation[0..3]`，顺序 w、x、y、z |
| `imu_angular_velocity_{x,y,z}` | `imu.angular_velocity[0..2]`，rad/s |
| `imu_linear_acceleration_{x,y,z}` | `imu.linear_acceleration[0..2]`，m/s² |

## 3. 一次性请求和 ModeResult

### 3.1 request_id

每个 action goal 或 service request 必须携带非零 `request_id`。请求方在一个控制会话内
保证编号唯一并递增；adapter 不得因为 ROS goal 重试而生成另一个 core 请求编号。

`request_id` 只用于去重和结果关联，不代表时间戳，也不代表 action goal handle。请求的
core `timestamp_ns` 由 adapter 使用本机 monotonic clock 生成。控制会话由
`StateDiagnostic.session_id` 识别；session 变化后，旧请求不得在新会话自动重放，未完成的
ROS goal 应被标记为失败或取消并重新建立请求。

MotionRuntime 的既有去重语义通过 ROS 统一保留：

- 相同 `request_id` 重试：返回当前活动状态或历史终态，不重复执行；
- 活动请求重试：返回 `Accepted` 或 `Running` 的当前快照；
- 已终态请求重试：返回原 `Completed`、`Rejected` 或 `Failed`；
- 乱序的旧编号不能覆盖更新请求；
- session 变化后的旧 request 不能继续执行。

### 3.2 状态值

`ModeResult.msg` 的 `state` 常量与 `core::ModeResultState` 一一对应：

| ROS 状态 | core 状态 | 语义 |
| --- | --- | --- |
| `ACCEPTED` | `Accepted` | 前置检查通过，等待控制周期开始执行 |
| `RUNNING` | `Running` | 请求对应的动作正在执行 |
| `COMPLETED` | `Completed` | 请求成功完成 |
| `REJECTED` | `Rejected` | 参数、当前模式、session 或能力检查失败，未执行 |
| `FAILED` | `Failed` | 已接受但运行期间失败或被安全打断 |

`message` 只用于日志、界面和诊断，不得作为程序分支条件。每个 action 的 feedback
使用 `ModeResult status`，result 使用 `ModeResult result`；`request_id` 必须与 goal
一致。终态产生时只交付一次 `/motion/result` 事件，但 action server 可以把相同终态返回
给对应 goal，二者必须使用同一个 `ModeResult` 内容。

### 3.3 action 语义

- `GetUp`：请求 `ModeRequestType::GetUp`，成功后进入 `Stand`；非法状态、无状态、fault
  或 session 不匹配返回 `Rejected`，运行中安全错误返回 `Failed`。
- `GetDown`：请求 `ModeRequestType::GetDown`，完成后进入 `Passive`；允许
  `EnterPassive` 打断。
- `StartBehavior`：`behavior_name` 原样映射到
  `ModeRequest::behavior_name`。未注册行为直接 `Rejected`。当前阶段不新增行为实现。
- `SwitchPolicy`：`policy_name` 原样映射到 `ModeRequest::policy_name`。可以在 Running
  状态接收；切换期间旧策略暂停，feedback 保持 `Running`，完成后才报告 `Completed`。

`SwitchPolicy` 的成功路径由 MotionRuntime 决定：当前姿态接近目标默认姿态时直接 reload；
不接近时先输出固定位置阻抗过渡，再初始化目标策略、清空旧历史和动作目标，随后恢复
`rl_locomotion`。未知策略、策略未加载或配置不合法返回 `Rejected`；已经接受后发生
推理、过渡或后端错误返回 `Failed`，不得继续使用旧策略的旧动作。

### 3.4 service 语义

`EnterPassive` 使用 `ModeRequestType::EnterPassive`，拥有最高打断优先级，可以立即打断
所有主动 action。service response 的 `result` 是 EnterPassive 请求自身的终态；若确实打断
了活动请求，`has_interrupted_request` 为 true，`interrupted_result` 携带被打断请求的
`Failed` 终态，并且该终态也必须通过 `/motion/result` 发布。没有活动请求时该字段为 false。

`ResetFault` 使用 `ModeRequestType::ResetFault`。当前 `core::RobotIO` 没有真实的故障复位
边界，MotionRuntime 对该请求返回 `Rejected`，消息应保留
`fault reset is not supported by RobotIO` 语义。ROS service 不得伪造复位成功；后续新增
明确的 RobotIO 复位能力时，才可以扩展 adapter 行为。

## 4. QoS 和线程边界

| 数据 | QoS | 约束 |
| --- | --- | --- |
| `/cmd_vel` | reliable、depth 1、volatile | 只保留最新值，允许覆盖旧消息 |
| action/service | reliable 请求应答 | 不丢弃请求，不在 callback 内执行控制周期 |
| `/motion/status`、`/robot_io/status` | reliable、depth 1、transient local | 低频最新状态，新订阅者可取得最近快照 |
| `/state/diagnostic` | best effort、depth 1、volatile | 允许丢帧，不阻塞控制线程 |
| `/motion/result` | reliable、depth 16、volatile | 终态事件可靠发布，订阅端仍以 request_id 去重 |

QoS 是顶层 adapter 的约定，不改变 core 的固定容量数据结构。ROS callback 只负责解析、
校验、写入最新值通道或固定容量请求通道；独立 motiond 在自己的周期读取这些输入，再创建
`BaseCommand` / `ModeRequest` 并调用 `MotionRuntime::update()`。ROS 接口不能绕过
`RobotIO` 提交 `CommandFrame`，也不能把 ROS 消息类型放入 core。

## 5. 当前实现边界

阶段 7 已实现 ROS 2 node、executor、IPC、session、三进程故障退路和统一 launch，仍明确不包含：

- GUI 长时间压力测试和多显示环境适配；
- ROS 到 RobotIO 的直接提交；
- blackW、真实硬件或跨机器协议；
- 新的通用行为框架。

三进程运行、构建和故障规则见
[`ros2_three_process_runtime.md`](ros2_three_process_runtime.md)。
