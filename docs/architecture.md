# 系统架构

`quadruped_control` 将运动控制、机器人后端和外部接口分开，MotionRuntime 通过统一的 `RobotIO` 与具体机器人或仿真环境连接。

## 1. 模块关系

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
 └── IPC ↔ ROS 2
```

主要模块职责：

| 模块 | 职责 |
| --- | --- |
| `core/` | 公共数据结构、RobotIO、状态与请求语义 |
| `motion/` | MotionRuntime、基础动作、行为与 RL 控制 |
| `policy/` | TorchScript 策略适配 |
| `backends/` | MuJoCo、Replay 等 RobotIO 实现 |
| `adapters/ipc/` | 三进程本机共享内存通信 |
| `adapters/ros2/` | ROS 2 topic/action/service 与 IPC 的转换 |
| `config_loader/` | YAML 配置加载 |
| `apps/` | 正式运行、调试与回放程序 |

`core/` 保持独立，不引入 ROS 2、Torch、MuJoCo、YAML 或设备 SDK 类型。运动层只依赖 `RobotIO`，从而让 MuJoCo、回放和后续实机后端共享同一套 MotionRuntime。

## 2. 运行方式

### 三进程仿真

```text
keyboard / joystick / ROS 2
             │
             ▼
        ros2_gateway
             │
             ▼
           motiond
             │
             ▼
     mujoco_backendd
             │
             ▼
           MuJoCo
```

- `ros2_gateway`：外部输入和 ROS 2 接口；
- `motiond`：MotionRuntime、行为状态机和策略推理；
- `mujoco_backendd`：物理步进、状态读取和关节命令执行。

三个进程通过 POSIX 共享内存通信。详细 IPC、futex、session 和调度规则见
[运行时与 IPC](runtime_ipc.md)。

### 单进程调试

`apps/mujoco_sim/` 将 MotionRuntime 与 `MujocoRobotIO` 放在同一进程，适合模型、策略和 GUI 调试。

### 回放

`ReplayRobotIO` 从日志提供 `StateFrame`，并记录 MotionRuntime 生成的 `CommandFrame`。
回放格式见 [诊断与回放](diagnostics_replay.md)。

## 3. 核心数据

### StateFrame

包含：

- 关节位置、速度、估算力矩、温度、错误码和有效状态；
- IMU 四元数、角速度和线加速度；
- SafetyState；
- 命令序号与运行诊断信息。

### CommandFrame

每个关节命令包含：

- `ControlMode`；
- target position / velocity；
- KP、KD；
- feedforward effort。

控制模式由 `ControlMode` 给出。增益只描述该模式下的参数。

### FrameHeader

StateFrame 与 CommandFrame 使用共同的帧标识：

- `schema_version`
- `startup_id`
- `session_id`
- `sequence`
- `timestamp_ns`

CommandFrame 额外包含 `expires_at_ns`。

### BaseCommand

BaseCommand 保存机体速度目标：

```text
vx, vy, wz
```

同时记录来源、优先级、时间和有效期。各输入来源采用 latest-value 语义。

### ModeRequest / ModeResult

GetUp、GetDown、StartBehavior、SwitchPolicy、EnterPassive 等一次性操作使用 request/result 模型：

```text
Accepted → Running → Completed
                   ↘ Failed

Rejected
```

`request_id` 由 gateway 统一生成，用于内部请求去重和结果关联。ROS 客户端不负责维护该
编号。

## 4. MotionRuntime

稳定模式：

```text
Passive ──GetUp──> GetUp ──> Stand
Stand ──StartBehavior──> Running
Stand/Running ──GetDown──> GetDown ──> Passive
```

`Running` 下再通过 `behavior_name` 区分具体行为，例如：

- `rl_locomotion`
- `retry`
- `event_chain`
- `bridge_drive`
- `low_bar_drive`
- `car_drive`

机器人之间的差异主要由 RobotModel、ControllerConfig、策略配置、行为配置和 `JointRole` 表达。

### RL 数据路径

```text
StateFrame
  → observation
  → history
  → Torch policy
  → action conversion
  → CommandFrame
```

MotionRuntime 以控制周期提交 CommandFrame，策略按照 decimation 更新动作。切换策略时会重置目标策略的历史和旧动作；默认姿态差异较大时先执行姿态过渡。

black 与 blackW 的观测维度、动作转换和行为差异见
[机器人模型与行为](robot_models_and_behaviors.md)。

## 5. 数据约定

- C++17；
- 关节位置：rad；
- 关节速度：rad/s；
- 力矩：N·m；
- 线速度：m/s；
- 四元数顺序：`w,x,y,z`；
- 时间戳：本机 monotonic nanoseconds；
- `kMaxJoints == 16`；
- 逻辑腿顺序：FL、FR、RL、RR。

机器人身份由 robot name、joint count 和有序关节名共同确定。按关节排列的配置使用明确的 `joint_names`。

## 6. 配置

| 路径 | 内容 |
| --- | --- |
| `configs/robots/*.yaml` | 关节顺序、角色和命令限制 |
| `configs/controllers/*.yaml` | 控制周期、基础姿态和固定增益 |
| `configs/policies/<robot>/*.yaml` | 策略模型、张量和动作参数 |
| `configs/policies/<robot>/policy_switch.yaml` | 策略加载与切换顺序 |
| `configs/behaviors/<robot>/*.yaml` | Retry、Event chain 和固定轮驱 |
| `configs/simulation/*.yaml` | MuJoCo 运行参数 |
| `configs/input/gamepads.yaml` | 手柄映射 |

策略和配置在启动阶段加载与校验，周期控制路径不访问配置文件。

## 7. 典型频率

| 环节 | 频率 |
| --- | ---: |
| MuJoCo physics | 500 Hz |
| MotionRuntime | 200 Hz |
| TorchScript policy | 50 Hz |
| gateway 状态发布 | 20 Hz |
| BaseCommand 默认有效期 | 200 ms |
| backend / motion heartbeat 超时 | 500 ms |

物理线程、MotionRuntime 和 ROS executor 独立运行。GUI 的渲染同步不会参与控制周期。

## 8. 测试

测试覆盖：

- core 数据与状态语义；
- black / blackW 配置；
- MotionRuntime 状态机和行为；
- RL 观测、历史、动作与策略切换；
- IPC、session 和 heartbeat；
- MuJoCo 映射与命令执行；
- replay；
- ROS 2 三进程 headless 链路。

常用入口：

```bash
./scripts/build.sh
./scripts/test/ctest.sh
./scripts/test/ros2_headless.sh
```
