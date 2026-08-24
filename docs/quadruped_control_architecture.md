# quadruped_control 架构与运行语义

本文描述当前仓库已经采用的模块边界、数据契约和运行语义。它面向仿真、测试和回放，不展开
尚未实现的实机通信或下位机方案。

## 1. 设计原则

1. `core/` 不依赖 ROS 2、Torch、MuJoCo、YAML 或设备 SDK；
2. MotionRuntime 只通过 `RobotIO` 读取状态和提交关节命令；
3. 机器人差异通过 RobotModel、ControllerConfig、行为配置和策略配置表达；
4. 高频状态与命令采用最新值语义，一次性操作采用请求—结果语义；
5. 关节顺序、角色、位置限制和单位必须显式定义；
6. 控制周期内不加载文件、不分配日志缓冲、不等待界面或 ROS callback；
7. 未知枚举、NaN/Inf、过期命令、旧 session 和关节数量不匹配必须在使用前拒绝。

## 2. 依赖方向

```text
core
 ├── motion
 │    └── policy/torch
 ├── backends/mujoco
 ├── backends/replay
 ├── adapters/ipc
 └── config_loader

apps
 ├── runtime_daemons
 ├── mujoco_sim
 └── replay

adapters/ros2
 └── 只连接 IPC 与 ROS 2，不进入 core
```

依赖只能沿图中方向增加。公共 core 头文件不能暴露第三方类型，motion 不能直接包含 backend、
ROS 2 或 MuJoCo 接口。

![当前控制系统总览](diagrams/quadruped_control_overview.svg)

## 3. 运行拓扑

### 3.1 三进程正式仿真

```text
keyboard / joystick / /cmd_vel / action / service
                         │
                         ▼
                    ros2_gateway
                         │  BaseCommand / ModeRequest
                         ▼
                       motiond
                         │  RemoteRobotIO
                         ▼
                  mujoco_backendd
                         │
                         ▼
                       MuJoCo
```

- `ros2_gateway`：输入、ROS 接口、低频状态发布；
- `motiond`：MotionRuntime、行为状态机、策略推理；
- `mujoco_backendd`：物理步进、命令执行、状态和 heartbeat 发布。

三者通过固定容量 POSIX 共享内存连接。ROS callback 不执行控制周期，MuJoCo backend 不依赖
ROS 2，motiond 不依赖 MuJoCo API。

### 3.2 单进程调试

`apps/mujoco_sim/` 把 MotionRuntime 与 MujocoRobotIO 放在同一进程，适合检查模型、策略、
按键、reset、鼠标扰动力和 GUI。它与三进程链路使用相同 RobotIO 和运动语义，但不代替
ROS 2/IPC 端到端测试。

### 3.3 回放与测试

- `ReplayRobotIO` 按日志提供 StateFrame，并在进程内记录生成的 CommandFrame；
- `FakeRobotIO` 由测试控制返回码、状态和命令提交结果；
- 两者都通过 RobotIO 驱动同一 MotionRuntime，不建立第二套状态机。

![仿真与回放入口](diagrams/simulation_and_debug.svg)

## 4. 核心数据契约

### 4.1 通用规则

- C++17；
- 关节位置 rad、速度 rad/s、力矩 N·m、线速度 m/s；
- 四元数顺序 `w,x,y,z`；
- 时间戳使用本机单调纳秒；
- 固定容量 `kMaxJoints == 16`；
- black 为 12 关节，blackW 为 16 关节；
- 逻辑顺序固定为 FL、FR、RL、RR，每条腿为 hip、thigh、calf、可选 wheel。

不能根据数组长度猜测机器人类型。应用启动时必须同时校验 robot name、joint count 和有序
关节名称。

### 4.2 FrameHeader

StateFrame 和 CommandFrame 共享：

- `schema_version`：core 帧格式版本；
- `startup_id`：进程本次启动身份；
- `session_id`：当前控制会话；
- `sequence`：会话内递增序号；
- `timestamp_ns`：完整帧生成时间。

CommandFrame 另有 `expires_at_ns`。旧 startup/session、未来时间戳和过期命令不能使用。

### 4.3 StateFrame

StateFrame 包含：

- 每个关节的位置、速度、估算力矩、温度、错误码、online 和 valid；
- IMU 四元数、角速度、线加速度及有效性；
- SafetyState；
- 最近接受和实际生效的命令序号；
- 总线、丢帧和循环诊断字段。

MotionRuntime 只有在整帧通过 RobotModel 和数值校验后，才更新当前姿态、会话和行为状态。

### 4.4 CommandFrame

每个关节命令显式包含：

- `ControlMode`；
- target position / velocity；
- KP、KD；
- feedforward effort。

不能从增益推断模式。轮子通常使用 `JointImpedance`、`KP=0`、当前轮角、目标轮速和非零 KD；
这仍然不是 `Velocity` 或 `Damping` 模式。

### 4.5 BaseCommand

BaseCommand 表示 `vx`、`vy`、`wz`、来源、优先级、时间和有效期。每个来源只保留最新值。
速度命令过期后归零，但不会因此退出正在运行的 RL 行为。

### 4.6 ModeRequest、ModeResult 与 MotionStatus

一次性请求包括 GetUp、GetDown、StartBehavior、SwitchPolicy、EnterPassive 和 ResetFault。
每个请求使用非零 request_id，并遵循：

```text
Accepted → Running → Completed
                   ↘ Failed
Rejected（未开始执行）
```

重复 request_id 返回已有状态，不重复动作；更小的旧编号不能覆盖更新请求。

MotionStatus 只保存稳定模式和诊断：

- `mode`：Passive、GetUp、Stand、Running、GetDown；
- `behavior_name`：如 `rl_locomotion`、`retry`、`event_chain`；
- `behavior_phase`：行为内部阶段，仅供显示和诊断；
- `policy_name`、`policy_ready`、`command_limits`；
- active source 和最近错误。

## 5. MotionRuntime

### 5.1 稳定模式

```text
Passive ──GetUp──> GetUp ──完成──> Stand
Stand ──StartBehavior──> Running
Stand/Running ──GetDown──> GetDown ──完成──> Passive
任意主动模式 ──EnterPassive/运行错误──> Passive
```

GetUp、GetDown 和策略姿态过渡均从最新关节位置开始插值。reset 加载仿真姿态时保持 session
和当前行为；管理级 session reset 才清除旧请求、速度和调度状态。

### 5.2 行为

当前行为：

- `rl_locomotion`：black/blackW；
- `retry`：black/blackW 共用；
- `event_chain`：公共接口，blackW 有事件，black 为空配置；
- `bridge_drive`、`low_bar_drive`、`car_drive`：blackW。

行为不是新的 MotionMode。共同的打断、错误和 ModeResult 语义集中在 MotionRuntime，机器人
差异由配置和 JointRole 决定。

### 5.3 RL 数据路径

RlController 在启动期读取固定容量配置：观测维度、历史帧、策略关节映射、轮索引、动作
缩放、默认姿态、增益和命令限制。控制周期内只执行：

```text
StateFrame
  → observation
  → history
  → Torch policy（按 decimation）
  → action conversion
  → CommandFrame
```

未到推理周期时复用最近动作目标，CommandFrame 仍按控制周期提交。策略切换会清空目标策略
历史和旧动作；默认姿态不同则先完成位置阻抗过渡。

### 5.4 错误退路

活动模式遇到以下问题会结束请求并进入 Passive：

- RobotIO 确认断开或无状态；
- StateFrame 无效、session 改变或时间错误；
- policy forward/shape/数值/超时错误；
- CommandFrame 生成、校验或提交失败。

错误文本保留在 MotionStatus，直到显式进入新动作或 EnterPassive 清除。IPC 的瞬时 latest
slot 锁竞争会在同一 backend 会话内复用最后有效快照，不会被误判为真实断线。

## 6. IPC 与会话

共享内存保存两类通道：

| 数据 | 通道 | 语义 |
| --- | --- | --- |
| StateFrame、CommandFrame、BaseCommand | latest slot | 新值覆盖旧值 |
| MotionStatus、RobotIOStatus、heartbeat | latest slot | 最新状态快照 |
| ModeRequest、ModeResult | 固定容量 SPSC | FIFO，不覆盖请求 |
| backend control request/result | 固定容量 SPSC | reset、pause 等请求应答 |

latest slot 使用共享自旋锁和有界 CAS 重试。RemoteRobotIO 缓存同一 backend startup/session
内最后有效 heartbeat 与状态；heartbeat 超过 500 ms 或明确 offline 后仍返回 Disconnected。

backend 首次启动建立 session 1。管理级 reset 递增 session，并使旧命令、请求和结果失效。
GUI/command 的姿态 reset 只加载 keyframe，不建立新 session，也不退出当前 RL 行为。

![三进程仿真数据流](diagrams/data_communication.svg)

## 7. 配置职责

| 路径 | 职责 |
| --- | --- |
| `configs/robots/*.yaml` | 机器人名称、关节顺序、角色和命令限制 |
| `configs/controllers/*.yaml` | 控制周期、有效期、基础姿态和固定增益 |
| `configs/policies/<robot>/*.yaml` | 策略张量、模型、动作映射和命令限制 |
| `configs/policies/<robot>/policy_switch.yaml` | 启动加载白名单和切换顺序 |
| `configs/behaviors/<robot>/*.yaml` | Retry、Event chain、固定姿态轮驱 |
| `configs/simulation/*.yaml` | 仿真实时倍率、显示同步和 VSync |
| `configs/input/gamepads.yaml` | 物理手柄到规范化 Joy 布局的映射 |

所有按关节排列的配置都必须携带 `joint_names` 并在启动期校验。策略模型和 YAML 在启动时
加载、检查和预热，控制周期内不访问文件。

## 8. 周期与线程边界

| 环节 | 当前典型频率 |
| --- | ---: |
| black MuJoCo physics | 500 Hz |
| blackW MuJoCo physics | 500 Hz |
| MotionRuntime | 200 Hz |
| TorchScript policy | 50 Hz |
| gateway 状态发布 | 20 Hz |
| BaseCommand 默认有效期 | 200 ms |
| backend/motion heartbeat 超时 | 500 ms |

物理线程、motion 控制周期和 ROS executor 独立运行。MuJoCo GUI 使用显示副本和非阻塞同步；
渲染线程忙时可以跳过一帧画面，但不能阻塞物理步进和 heartbeat。

## 9. 测试策略

- core：枚举、帧、数值、关节顺序和过期语义；
- config：black/blackW RobotModel、Controller、策略和行为 YAML；
- motion：状态机、请求生命周期、Retry、Event chain、固定轮驱和错误退路；
- RL：观测、历史、动作转换、策略切换和命令限制；
- IPC：wire 转换、latest slot、队列、session、heartbeat 和锁竞争；
- MuJoCo：映射、混合腿轮命令、reset、闭环动作和命令过期；
- replay：日志身份、CSV 往返和确定性命令对比；
- ROS 2 headless：三进程启动、请求、状态、reset 和单进程退出退路。

最小验证命令按改动范围选择；涉及完整仿真链路时使用：

```bash
./scripts/build.sh --rl
./scripts/build.sh --target command
./scripts/test/ros2_headless.sh
```

## 10. 架构图维护

可编辑 Graphviz 源文件和生成的 SVG 位于 `docs/diagrams/`。修改 `.dot` 后运行：

```bash
for src in docs/diagrams/*.dot; do
  dot -Tsvg "$src" -o "${src%.dot}.svg"
done
```
