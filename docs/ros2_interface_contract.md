# ROS 2 接口

ROS 2 gateway 位于 `adapters/ros2/quadruped_gateway/`，通过本机 IPC 与 `motiond` 和 `mujoco_backendd` 通信。

## 1. Topic

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 机体速度目标 |
| `/motion/status` | `quadruped_interfaces/msg/MotionStatus` | MotionRuntime 状态 |
| `/robot_io/status` | `quadruped_interfaces/msg/RobotIOStatus` | 后端状态和帧统计 |
| `/state/diagnostic` | `quadruped_interfaces/msg/StateDiagnostic` | StateFrame 低频诊断 |
| `/motion/result` | `quadruped_interfaces/msg/ModeResult` | 一次性请求结果 |

### `/cmd_vel`

使用三个字段：

| ROS 字段 | BaseCommand | 单位 |
| --- | --- | --- |
| `linear.x` | `vx` | m/s |
| `linear.y` | `vy` | m/s |
| `angular.z` | `wz` | rad/s |

gateway 将其转换为 `CommandSource::Navigation`，时间戳使用本机 monotonic clock。默认有效期为 200 ms，可通过 `cmd_vel_timeout_ns` 调整。

BaseCommand 与关节级 CommandFrame 使用独立的有效期。`/cmd_vel` 停止更新后，导航速度按 BaseCommand 超时规则归零。

## 2. Action 与 Service

| 接口 | 类型 | 功能 |
| --- | --- | --- |
| `/motion/get_up` | `GetUp` action | 起立 |
| `/motion/get_down` | `GetDown` action | 趴下 |
| `/motion/start_behavior` | `StartBehavior` action | 启动行为 |
| `/motion/switch_policy` | `SwitchPolicy` action | 切换策略 |
| `/motion/enter_passive` | `EnterPassive` service | 进入 Passive |
| `/motion/reset_fault` | `ResetFault` service | 请求软件故障复位 |

一次性请求携带非零 `request_id`，在一个 session 中用于去重和结果关联。

结果状态：

```text
Accepted → Running → Completed
                   ↘ Failed

Rejected
```

- `Rejected`：请求没有开始执行；
- `Failed`：请求已经开始，但运行期间失败或被打断。

相同 `request_id` 的重试返回已有状态，不重复执行动作。

## 3. MotionStatus

主要字段：

| 字段 | 含义 |
| --- | --- |
| `mode` | Passive / GetUp / Stand / Running / GetDown |
| `active_source` | 当前速度命令来源 |
| `behavior_name` | 当前行为 |
| `behavior_phase` | 行为内部阶段 |
| `policy_name` | 当前策略 |
| `policy_ready` | 策略可用状态 |
| `command_limits[3]` | `vx`、`vy`、`wz` 限制 |
| `error_message` | 最近错误信息 |

`behavior_phase` 和 `error_message` 用于诊断；控制流程以请求结果和稳定状态为主要依据。

## 4. RobotIOStatus

| ROS 字段 | core 字段 |
| --- | --- |
| `state` | `RobotIOStatus::state` |
| `latest_state_sequence` | 同名字段 |
| `latest_command_sequence` | 同名字段 |
| `dropped_state_frames` | 同名字段 |
| `rejected_command_frames` | 同名字段 |

## 5. StateDiagnostic

StateDiagnostic 发布 StateFrame 的低频摘要：

- schema / startup / session / sequence；
- timestamp；
- joint count；
- safety state；
- accepted / effective command sequence；
- IMU 有效状态、四元数、角速度和线加速度。

高频关节数组保留在内部 StateFrame 中。

## 6. 请求语义

### GetUp / GetDown

GetUp 成功后进入 Stand，GetDown 成功后进入 Passive。

### StartBehavior

`behavior_name` 直接映射到 MotionRuntime 的行为名称，例如 `rl_locomotion`、`retry`、`event_chain`。

### SwitchPolicy

`policy_name` 指定目标策略。需要姿态过渡时，MotionRuntime 先完成过渡，再初始化目标策略并恢复 `rl_locomotion`。

### EnterPassive

EnterPassive 可中断正在运行的主动请求。被中断请求以 Failed 结束。

### ResetFault

接口已经保留；是否支持具体故障复位由 RobotIO 能力决定。

## 7. QoS

| 数据 | QoS |
| --- | --- |
| `/cmd_vel` | reliable, depth 1, volatile |
| action / service | reliable |
| `/motion/status` | reliable, depth 1, transient local |
| `/robot_io/status` | reliable, depth 1, transient local |
| `/state/diagnostic` | best effort, depth 1, volatile |
| `/motion/result` | reliable, depth 16, volatile |

ROS callback 负责消息转换与 IPC 写入，MotionRuntime 由 `motiond` 的独立控制周期驱动。

三进程拓扑和 IPC 规则见 [ROS 2 三进程运行架构](ros2_three_process_runtime.md)。
