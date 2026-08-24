# ROS 2 接口契约与使用指南

本文说明 `quadruped_gateway` 对外提供的 ROS 2 接口、字段语义、调用前提和常用操作。
接口定义位于 `adapters/ros2/quadruped_interfaces/`，实现位于
`adapters/ros2/quadruped_gateway/`。gateway 不运行控制器，也不直接访问 MuJoCo，而是通过
本机共享内存连接 `motiond` 和 `mujoco_backendd`：

```text
ROS 2 client
    │ topic / action / service
    ▼
quadruped_gateway ── shared memory ──► motiond ──► mujoco_backendd
```

## 1. 启动与环境

gateway 启动时会打开已经存在的共享内存，因此应依次启动 backend、motion、gateway：

```bash
# 终端 1
./scripts/run/backend.sh blackW terrain

# 终端 2
./scripts/run/motion.sh blackW

# 终端 3；使用手柄时将 keyboard 改为 joystick
./scripts/run/command.sh blackW keyboard
```

在新的 CLI 终端中加载 ROS 2 环境：

```bash
source /opt/ros/humble/setup.bash
source .build/ros2/install/setup.bash

ros2 node list
ros2 topic list -t
ros2 action list -t
ros2 service list -t
```

可以直接查看自定义接口定义：

```bash
ros2 interface show quadruped_interfaces/msg/MotionStatus
ros2 interface show quadruped_interfaces/action/StartBehavior
ros2 interface show quadruped_interfaces/srv/EnterPassive
```

## 2. 接口总览

### 2.1 Topic

| 名称 | 类型 | gateway 方向 | 用途 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 订阅 | 导航速度目标 |
| `/joy` | `sensor_msgs/msg/Joy` | 订阅 | 手柄原始输入，名称可配置 |
| `/motion/status` | `quadruped_interfaces/msg/MotionStatus` | 发布 | 运动模式、行为和策略状态 |
| `/robot_io/status` | `quadruped_interfaces/msg/RobotIOStatus` | 发布 | 后端状态和帧统计 |
| `/state/diagnostic` | `quadruped_interfaces/msg/StateDiagnostic` | 发布 | StateFrame 低频摘要 |
| `/motion/result` | `quadruped_interfaces/msg/ModeResult` | 发布 | 请求状态变化事件流 |

状态类 topic 默认每 50 ms 更新一次，约为 20 Hz。它们用于监控和低频编排，不替代内部的
高频 StateFrame 和 CommandFrame。

### 2.2 Action 与 Service

| 名称 | 类型 | 功能 |
| --- | --- | --- |
| `/motion/get_up` | `quadruped_interfaces/action/GetUp` | 起立并进入 Stand |
| `/motion/get_down` | `quadruped_interfaces/action/GetDown` | 趴下并进入 Passive |
| `/motion/start_behavior` | `quadruped_interfaces/action/StartBehavior` | 启动 RL、事件链或固定姿态行为 |
| `/motion/switch_policy` | `quadruped_interfaces/action/SwitchPolicy` | 在 RL 中切换策略 |
| `/motion/enter_passive` | `quadruped_interfaces/srv/EnterPassive` | 进入 Passive，并中断活动请求 |
| `/motion/reset_fault` | `quadruped_interfaces/srv/ResetFault` | 请求 RobotIO 复位故障 |

Action 适合需要等待和反馈的动作。Service 用于短时控制操作；transport 成功不表示业务操作
一定成功，调用方仍须检查返回的 `ModeResult`。

## 3. `/cmd_vel`：导航速度命令

gateway 只使用 `Twist` 的三个字段：

| ROS 字段 | 内部字段 | 单位 | 正方向 |
| --- | --- | --- | --- |
| `linear.x` | `vx` | m/s | 机体前方 |
| `linear.y` | `vy` | m/s | 机体左侧 |
| `angular.z` | `wz` | rad/s | 绕机体 z 轴逆时针 |

其余字段忽略。NaN 或 Inf 会转换为 0，有效分量会按当前 `command_limits` 分别限幅。

持续发布示例：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

停止前也可以显式发送一帧零速度：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

默认有效期为 200 ms。发布停止后旧命令会过期并按零速度处理，因此不要只发送一次非零
速度来实现持续运动，推荐以 10～20 Hz 发布。

`/cmd_vel` 被转换为 `CommandSource::Navigation`，优先级为 100。它需要同时满足以下条件：

1. 当前选择导航输入，而不是键盘或手柄的 manual 输入；
2. 当前处于 Running，且行为接受 BaseCommand；
3. 消息持续更新，没有超过 `cmd_vel_timeout_ns`；
4. 策略已经就绪，命令没有被限幅到零。

输入选择与旧 `rl_sar` 一致：gateway 启动时为 manual；按键盘 `N` 或手柄 `X` 后进入
navigation，再次按下才返回 manual。该选择是持久状态，手柄连接、断开、超时或重连都不会
改变它。navigation 中只忽略手柄摇杆和键盘速度增量，起立、趴下、Passive、Retry、策略
切换、事件链和仿真控制等离散按键仍然有效。

`MotionStatus.command_limits` 顺序固定为 `[abs(vx), abs(vy), abs(wz)]`。策略就绪后使用策略
配置值；策略未就绪时使用回退值 `[3.0, 1.0, 3.0]`。例如限制为
`[2.0, 1.0, 3.0]` 时，`linear.x = 2.8` 会转换为 `vx = 2.0 m/s`。手柄还会把
归一化摇杆量程自动映射到这三个物理上限。

## 4. 请求结果与 `request_id`

所有一次性请求都使用 `quadruped_interfaces/msg/ModeResult`：

| 字段 | 含义 |
| --- | --- |
| `request_id` | gateway 分配的内部请求标识，用于排序、去重和关联结果 |
| `state` | 请求状态枚举 |
| `message` | 接受原因、进度说明或失败原因 |

| 状态 | 值 | 含义 |
| --- | ---: | --- |
| `ACCEPTED` | 0 | MotionRuntime 已接受请求 |
| `REJECTED` | 1 | 请求未开始执行，通常是前置条件不满足 |
| `RUNNING` | 2 | 正在执行 |
| `COMPLETED` | 3 | 成功完成 |
| `FAILED` | 4 | 开始后失败、中断或等待超时 |

```text
ACCEPTED ──► RUNNING ──► COMPLETED
                 └────► FAILED

REJECTED
```

ROS 2 Action 显示“goal accepted”只表示 gateway 接受了 goal，不代表 MotionRuntime 一定
执行。运动层仍可能随后返回 `REJECTED`，原因位于 `message`。

ROS 客户端不填写和管理 `request_id`。gateway 为 Action、Service、键盘和手柄请求统一生成
非零、单调递增的内部编号，避免多个输入节点之间发生编号冲突。该编号主要用于 IPC 去重、
Action 反馈关联和诊断；调用方读取即可，不应根据它决定请求优先级。

MotionRuntime 对相同内部编号返回已有状态而不重复执行，并拒绝延迟到达的旧编号。普通
MuJoCo 姿态 reset 不改变 motion session，也不要求 ROS 客户端做编号处理。

四个 Action 的 feedback 和 result 均包含 `ModeResult`：

```text
feedback.status  -> 当前状态
result.result    -> 最终状态
```

gateway 最多等待 10 秒获取终态。建议同一时间只维持一个需要编排的 Action。取消 ROS Action
goal 当前只会把该 ROS goal 标为 canceled，并返回 `FAILED`；它不会停止底层动作。真正停止
当前运动应调用 `/motion/enter_passive`。

## 5. Action 使用方法

以下编号都由 gateway 自动分配，CLI 只填写真正的业务字段。

### 5.1 GetUp

GetUp Goal 没有业务字段：

```bash
ros2 action send_goal /motion/get_up quadruped_interfaces/action/GetUp \
  "{}" --feedback
```

成功后进入 Stand。它要求最新状态有效且安全状态为 `CONTROL_ENABLED`。RL locomotion 中
请求 GetUp 会停止策略，并从当前反馈姿态平滑返回站姿。已经处于 GetUp 或 Stand、状态
无效，或正在运行不可中断行为时会被拒绝。

### 5.2 GetDown

```bash
ros2 action send_goal /motion/get_down quadruped_interfaces/action/GetDown \
  "{}" --feedback
```

成功后进入 Passive。GetDown 会回到记录的趴卧参考姿态；没有有效参考姿态、状态无效或安全
状态不允许控制时会被拒绝。`retry` 已锁定时不能通过 GetDown 退出，应使用 GetUp 或
EnterPassive。

### 5.3 StartBehavior

Goal 只包含 `behavior_name`：

| 行为名 | 适用范围 | 主要前置条件 |
| --- | --- | --- |
| `rl_locomotion` | black、blackW | 策略已加载，通常从 Stand 进入，有有效 BaseCommand |
| `retry` | 已配置该行为的机器人 | 状态和安全检查通过 |
| `event_chain` | 已配置事件链的机器人 | 从 Stand 或兼容 Running 行为进入 |
| `bridge_drive` | blackW | 从 Stand/兼容行为进入，有有效 BaseCommand |
| `car_drive` | blackW | 同上 |

先在另一终端持续发布 `/cmd_vel`，再启动 RL：

```bash
ros2 action send_goal /motion/start_behavior \
  quadruped_interfaces/action/StartBehavior \
  "{behavior_name: rl_locomotion}" --feedback
```

blackW 固定姿态轮驱示例：

```bash
ros2 action send_goal /motion/start_behavior \
  quadruped_interfaces/action/StartBehavior \
  "{behavior_name: bridge_drive}" --feedback
```

`retry` 进入 `locked` 后请求返回 `COMPLETED`，但机器人仍保持
`RUNNING / retry / locked`。后续用新的 GetUp 请求恢复站立，或用 EnterPassive 放松。

### 5.4 SwitchPolicy

仅允许在 `RUNNING / rl_locomotion` 中调用：

```bash
ros2 action send_goal /motion/switch_policy \
  quadruped_interfaces/action/SwitchPolicy \
  "{policy_name: obstacle}" --feedback
```

`policy_name` 也可以为 `next` 或 `toggle`，两者当前都表示切换到策略循环的下一项。若新旧
策略默认姿态不同，MotionRuntime 会先完成姿态过渡，再初始化目标策略并恢复 RL。

默认集合为 black 的 `flat、obstacle`，以及 blackW 的 `flat、obstacle、stair`。实际可用项
以 `configs/policies/*.yaml` 和模型加载结果为准。

## 6. Service 使用方法

### 6.1 EnterPassive

```bash
ros2 service call /motion/enter_passive \
  quadruped_interfaces/srv/EnterPassive "{}"
```

响应包含本请求的 `result`、`has_interrupted_request` 和 `interrupted_result`。如果中断了活动
请求，后者会给出被中断请求的 `FAILED` 结果。EnterPassive 是停止主动运动的明确接口。

### 6.2 ResetFault

```bash
ros2 service call /motion/reset_fault \
  quadruped_interfaces/srv/ResetFault "{}"
```

接口已经保留，但当前 RobotIO 没有故障复位能力，现有实现会返回 `REJECTED` 和
`fault reset is not supported by RobotIO`。不能因 service transport 成功就假定故障已清除。

## 7. 状态 Topic 详解

### 7.1 `/motion/status`

| 字段 | 含义 | 用法 |
| --- | --- | --- |
| `mode` | 粗粒度模式 | 上层主状态机判断 |
| `active_source` | 当前 BaseCommand 来源 | 排查导航/manual 抢占 |
| `behavior_name` | 当前具体行为 | 区分 RL、retry、事件链和固定姿态 |
| `behavior_phase` | 行为内部阶段 | 仅诊断和显示 |
| `policy_name` | 当前策略 | 确认策略切换 |
| `error_message` | 最近运动错误 | 配合 Action 结果分析 |
| `policy_ready` | 策略是否可推理 | 启动 RL 前检查 |
| `command_limits` | `vx/vy/wz` 绝对值上限 | 配置输入范围 |

`mode` 为 `PASSIVE=0`、`GET_UP=1`、`STAND=2`、`RUNNING=3`、`GET_DOWN=4`。
`active_source` 为 `NONE=0`、`GAMEPAD=1`、`NAVIGATION=2`、`REMOTE=3`、`TEST=4`。

```bash
ros2 topic echo --once /motion/status
```

`behavior_phase` 可能出现 `interpolating`、`starting`、`driving`、`policy_transition`、
`preparing`、`locked` 等值。这些是诊断信息，可能随实现变化；编排应以 Action 结果和稳定的
`mode/behavior_name` 为准。

### 7.2 `/robot_io/status`

| 字段 | 含义 |
| --- | --- |
| `state` | `DISCONNECTED=0`、`READY=1`、`PAUSED=2` 或 `FAULT=3` |
| `latest_state_sequence` | 最近 StateFrame 序号 |
| `latest_command_sequence` | 最近 CommandFrame 序号 |
| `dropped_state_frames` | latest-value 覆盖导致的状态丢帧累计数 |
| `rejected_command_frames` | 校验失败或过期命令累计数 |

```bash
ros2 topic echo --once /robot_io/status
```

少量 `dropped_state_frames` 不必然是故障；若持续快速增长且界面忽慢忽快，应检查进程阻塞、
调度和后端步进速度。`rejected_command_frames` 正常不应持续增长，增长时应检查 session、
时间戳、关节数和命令有效期。

### 7.3 `/state/diagnostic`

该 topic 不包含高频关节数组，主要字段如下：

| 字段 | 含义 |
| --- | --- |
| `schema_version` | IPC 数据结构版本 |
| `startup_id` / `session_id` | 后端启动与当前控制会话标识 |
| `sequence` / `timestamp_ns` | StateFrame 序号与单调时间戳 |
| `joint_count` | 当前关节数 |
| `safety_state` | 后端安全状态 |
| `last_accepted_command_sequence` | 最近通过校验的命令序号 |
| `effective_command_sequence` | 当前生效命令序号；0 表示没有 |
| `imu_valid` | IMU 数据是否有效 |
| `orientation_wxyz` | 四元数，固定为 `w, x, y, z` |
| `angular_velocity_xyz` | 角速度，rad/s |
| `linear_acceleration_xyz` | 线加速度，m/s² |

```bash
ros2 topic echo /state/diagnostic
```

`timestamp_ns` 是仿真/后端单调时间，不是 Unix wall-clock，不能与 `date +%s%N` 直接比较。
只能在同一 startup/session 内比较时间戳或 sequence。

### 7.4 `/motion/result`

它发布 `ACCEPTED`、`RUNNING` 和终态等所有状态变化，不只是最终结果：

```bash
ros2 topic echo /motion/result
```

该 topic 为 volatile，晚加入的订阅者收不到历史事件。消费者应按 `request_id + state` 去重；
可靠获取自己请求的结果应使用 Action result 或 Service response。

## 8. 完整控制流程

下面演示一次新 session 中起立、进入 RL、切换策略和趴下：

```bash
# 1. 起立。
ros2 action send_goal /motion/get_up quadruped_interfaces/action/GetUp \
  "{}" --feedback

# 2. 在独立终端持续发布速度。
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 3. 启动 RL。
ros2 action send_goal /motion/start_behavior \
  quadruped_interfaces/action/StartBehavior \
  "{behavior_name: rl_locomotion}" --feedback

# 4. 切换策略。
ros2 action send_goal /motion/switch_policy \
  quadruped_interfaces/action/SwitchPolicy \
  "{policy_name: obstacle}" --feedback

# 5. 停止速度发布后趴下。
ros2 action send_goal /motion/get_down quadruped_interfaces/action/GetDown \
  "{}" --feedback
```

若希望立即放松而不是执行趴下插值，用 EnterPassive 替代最后一步。

## 9. Python Action 客户端示例

下面是最小 GetUp 客户端，展示 feedback 与嵌套 result 的读取方法：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from quadruped_interfaces.action import GetUp


class GetUpClient(Node):
    def __init__(self):
        super().__init__('get_up_example')
        self._client = ActionClient(self, GetUp, '/motion/get_up')

    def send(self):
        self._client.wait_for_server()
        goal = GetUp.Goal()
        return self._client.send_goal_async(goal, feedback_callback=self.on_feedback)

    def on_feedback(self, message):
        status = message.feedback.status
        self.get_logger().info(
            f'request={status.request_id} state={status.state} message={status.message}'
        )


rclpy.init()
node = GetUpClient()
send_future = node.send()
rclpy.spin_until_future_complete(node, send_future)

goal_handle = send_future.result()
if goal_handle is None or not goal_handle.accepted:
    raise RuntimeError('gateway rejected the ROS goal')

result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future)
result = result_future.result().result.result
node.get_logger().info(
    f'final request={result.request_id} state={result.state} message={result.message}'
)

node.destroy_node()
rclpy.shutdown()
```

正式客户端还应处理 server 等待超时、session 变化和 Action transport 异常。

## 10. QoS 契约

| 数据 | Reliability | History / Depth | Durability |
| --- | --- | --- | --- |
| `/cmd_vel` | reliable | keep last / 1 | volatile |
| `/joy` | best effort | keep last / 1 | volatile |
| `/motion/status` | reliable | keep last / 1 | transient local |
| `/robot_io/status` | reliable | keep last / 1 | transient local |
| `/state/diagnostic` | best effort | keep last / 1 | volatile |
| `/motion/result` | reliable | keep last / 16 | volatile |
| Action / Service | reliable | ROS 2 管理 | volatile |

前两个状态 topic 使用 transient local，新订阅者可立即收到 gateway 保存的最新状态。诊断和
结果 topic 是实时流，不补发订阅者离线期间的数据。若 CLI 无法收到状态，可显式指定 QoS：

```bash
ros2 topic echo /motion/status quadruped_interfaces/msg/MotionStatus \
  --qos-reliability reliable --qos-durability transient_local
```

## 11. gateway 参数

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `shared_memory_name` | `/quadruped_control_black` | 共享内存名称 |
| `cmd_vel_timeout_ns` | `200000000` | `/cmd_vel` 有效期，ns |
| `joy_timeout_ns` | `250000000` | 手柄消息超时，ns |
| `keyboard_enabled` | `true` | 是否启用键盘轮询 |
| `fixed_drive_keys_enabled` | `false` | 是否启用 blackW 固定姿态快捷键 |
| `joy_require_connection_frame` | `true` | 是否要求手柄连接确认帧 |
| `joy_topic` | `/joy` | 手柄输入 topic |

参数只在启动时读取，当前不支持动态修改。通过脚本覆盖参数：

```bash
./scripts/run/command.sh black keyboard --ros-args \
  -p cmd_vel_timeout_ns:=300000000
```

也可以直接启动节点：

```bash
ros2 run quadruped_gateway ros2_gateway --ros-args \
  -p shared_memory_name:=/quadruped_control_blackW \
  -p keyboard_enabled:=false
```

超时值小于等于 0 时会回退到默认值，并不表示禁用超时。

## 12. 常见问题

### 看不到接口

确认 gateway 正在运行，当前终端已加载 `.build/ros2/install/setup.bash`。如果 gateway 无法
打开共享内存，应先启动 backend，并确认 `shared_memory_name` 一致。

### `/cmd_vel` 有数据但机器人不动

依次检查 `mode` 是否为 Running、行为是否接受速度、`active_source` 是否为 Navigation、
`policy_ready` 是否为 true、发布是否超过 5 Hz，以及 `command_limits` 是否符合预期。

### Action 接受后立即 Rejected

ROS Action 层接受不等于运动层通过前置检查。读取 `ModeResult.message`，检查 mode、behavior、
安全状态、BaseCommand，以及行为或策略是否已经配置。

### ROS、键盘和手柄同时发送模式请求

调用方不需要协调编号；gateway 会统一分配内部 `request_id`。如果请求的业务语义冲突，
MotionRuntime 根据当前模式和行为前置条件接受、拒绝或中断请求，调用方必须检查结果。

### 取消 Action 后仍在运动

ROS goal cancel 不停止底层运动。应调用 `/motion/enter_passive`。

### `reset_fault` 总是 Rejected

这是当前实现的预期行为，因为 RobotIO 尚未提供故障复位操作。应排查后端故障原因，不能
忽略响应状态。

### 状态跳变或画面忽慢忽快

观察 `/robot_io/status` 的序号和丢帧统计，以及 `/state/diagnostic` 的 startup、session、
sequence 和 timestamp。时间戳必须按后端单调时间解释；startup 或 session 变化后，旧请求、
旧命令和旧状态都不能继续沿用。

## 13. 使用原则

- 用 Action result 或 Service response 判断本次请求是否完成；
- 用 `/motion/status` 判断稳定运行状态；
- 用 `/motion/result` 做事件观察和日志，不把它当作结果存储；
- 用 `/robot_io/status` 和 `/state/diagnostic` 排查后端、session 和时序；
- 持续发布 `/cmd_vel`，并尊重策略给出的 `command_limits`；
- 不依赖 `behavior_phase` 或错误字符串驱动正式状态机；
- 多机器人除了使用不同共享内存，还须完整 remap 当前的绝对 ROS 名称，避免 gateway 共享
  同名接口。

三进程拓扑、共享内存所有权和故障语义见
[运行时与 IPC](runtime_ipc.md)。
