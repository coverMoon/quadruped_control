# 实机后端迁移架构参考

> 适用仓库：`N-W-wolf/quadruped_control`  
> 参考旧实现：`N-W-wolf/real_robot_black-W`  
> 文档定位：用于指导 black / blackW 实机控制链路迁移与后端设计。本文只固定整体结构、职责边界和实施顺序，不规定具体类内部实现。

---

## 1. 目标

将旧仓库中已经经过实机验证的电机通信、IMU、零位校准和安全保护能力迁移到 `quadruped_control`，并接入现有 `RobotIO + MotionRuntime + IPC` 架构。

迁移后的实机链路应与 MuJoCo 后端保持一致的上层接口：

```text
ROS 2 / External Control
          │
          ▼
     ros2_gateway
          │
      Shared IPC
          │
          ▼
       motiond
    MotionRuntime
          │
     CommandFrame
          │
          ▼
    real_backendd
          │
      RealRobotIO
          │
          ▼
     Physical Robot
```

核心目标：

- `MotionRuntime` 不感知 MuJoCo 或实机差异；
- black 与 blackW 共用同一套实机后端框架；
- 厂商 SDK、串口、电机 ID、方向、传动比、零位等硬件细节全部限制在 backend 内；
- ROS 2 不直接参与电机和 IMU 的底层通信；
- 实机 backend 在 `motiond` 或 ROS 2 异常时仍具备独立安全停机能力。

---

## 2. 总体模块关系

推荐结构：

```text
                  RobotModel
              black / blackW
                     │
                     ▼
                MotionRuntime
                     │
                CommandFrame
                     │
                     ▼
                  IPC
                     │
                     ▼
               real_backendd
                     │
                RealRobotIO
          ┌──────────┼──────────┐
          │          │          │
          ▼          ▼          ▼
   MotorBusGroup   IMU      HardwareSafety
          │          │
          ▼          ▼
   Actuator Codec  AB5465
          │          │
          ▼          ▼
     Unitree SDK    VQF
          │          │
          └────┬─────┘
               ▼
         Physical Robot
```

其中：

- `motiond`：运动控制、状态机、RL、行为切换；
- `real_backendd`：实机 backend 进程生命周期、IPC、heartbeat、session、命令接收；
- `RealRobotIO`：物理机器人统一 I/O；
- `MotorBusGroup`：多路电机总线管理；
- `Actuator Codec`：电机侧量与逻辑关节量之间的转换；
- `IMU`：串口、协议解析和姿态估计；
- `HardwareSafety`：实机底层安全保护。

---

## 3. 与现有 backend 的关系

新仓库已经存在：

```text
backends/
├── mujoco/
└── replay/
```

实机应作为同级 backend：

```text
backends/
├── mujoco/
├── replay/
└── real/
```

三种 backend 对上层统一暴露 `RobotIO` 语义：

```text
MujocoRobotIO ─┐
ReplayRobotIO ─┼─> RobotIO
RealRobotIO   ─┘
```

因此不再建立独立的“real motion runtime”或“real ROS node”。

---

## 4. RealRobotIO 的职责

`RealRobotIO` 是实机硬件的统一入口，负责协调底层设备，但不负责运动策略。

主要职责：

```text
RealRobotIO
├── 初始化 / 关闭物理设备
├── 接收 CommandFrame
├── 输出 StateFrame
├── 管理 motor bus
├── 管理 IMU
├── 维护最新硬件状态
├── 执行底层安全保护
└── 上报 RobotIOStatus
```

不应负责：

- RL policy 推理；
- GetUp / Stand / GetDown 状态机；
- ROS 2 topic / service / action；
- 用户输入；
- 行为切换逻辑。

这些仍由现有 motion / adapter 层负责。

---

## 5. 电机通信层

旧代码中的 `SerialPack` 同时包含：

- 串口；
- Unitree 电机 SDK；
- motor ID；
- black / blackW 差异；
- 方向；
- calf 额外传动；
- wheel 控制；
- 电机状态换算。

迁移后建议拆成两层。

### 5.1 MotorBus

负责纯硬件通信：

```text
MotorBus
├── serial device
├── baudrate
├── motor IDs
├── send
├── receive
├── vendor SDK
└── raw motor state
```

MotorBus 不应知道：

```text
FL_hip
FR_calf
wheel
zero_offset
robot model
default pose
```

---

### 5.2 Actuator Codec / Mapping

负责把逻辑关节与实际执行器建立映射：

```text
JointCommand
      │
      ▼
Actuator Codec
      │
      ▼
MotorCommand
```

以及：

```text
MotorState
     │
     ▼
Actuator Codec
     │
     ▼
JointState
```

这里处理：

- 电机方向；
- 零位；
- 厂商单位；
- 减速器；
- calf 外部传动；
- wheel 特殊控制方式；
- black / blackW 执行器差异。

第一版应优先复现旧实机代码的实际数学关系，确认实机行为一致后再进一步简化公式。

---

## 6. black 与 blackW 的关系

不为 black 和 blackW 分别复制一套 backend。

共用：

```text
RealRobotIO
MotorBus
Unitree SDK
IMU
HardwareSafety
IPC
real_backendd
```

差异通过配置描述：

```text
RobotConfig
HardwareConfig
JointRole
Actuator Mapping
```

逻辑模型：

### black

```text
FL: hip thigh calf
FR: hip thigh calf
RL: hip thigh calf
RR: hip thigh calf
```

共 12 个 actuator。

### blackW

```text
FL: hip thigh calf wheel
FR: hip thigh calf wheel
RL: hip thigh calf wheel
RR: hip thigh calf wheel
```

共 16 个 actuator。

---

## 7. RobotConfig 与 HardwareConfig

两类配置需要保持职责分离。

### RobotConfig

描述机器人控制语义：

```text
joint name
joint order
JointRole
default pose
控制相关参数
运动学相关信息
```

例如：

```text
FL_hip
FL_thigh
FL_calf
```

---

### HardwareConfig

描述某台真实机器人实际如何连接：

```text
串口设备
baudrate
motor bus
motor id
zero offset
direction
传动关系
IMU device
IMU baudrate
安全参数
```

推荐目录：

```text
configs/
├── robots/
│   ├── black.yaml
│   └── blackW.yaml
│
└── hardware/
    ├── black.yaml
    └── blackW.yaml
```

HardwareConfig 的结构可逐步演进，初期不需要为了通用性设计过多抽象。

---

## 8. 零位与校准

旧仓库中的 `motor_calibration.conf` 依赖固定数组顺序解释零位。

新配置应改为显式关节名，例如：

```yaml
zero_offset:
  FL_hip: ...
  FL_thigh: ...
  FL_calf: ...
  FR_hip: ...
```

这样可以避免 actuator 顺序变化后产生静默错误。

零位工具单独作为 hardware tool，不进入 backend 主程序：

```text
apps/hardware_tools/
├── motor_zero
└── motor_test
```

---

## 9. IMU

第一版优先保持旧实机链路：

```text
/dev/IMU_Link
      │
   460800
      │
      ▼
AB5465 Parser
      │
      ├── Gyroscope
      └── Accelerometer
             │
             ▼
            VQF
             │
             ▼
        StateFrame IMU
```

初期不更换姿态估计算法。

迁移时重点保证输出语义与 MuJoCo backend 一致：

- 坐标轴；
- body frame；
- gyro 单位；
- acceleration 单位；
- quaternion 顺序；
- gravity 方向；
- IMU 安装方向。

后续如果引入 EKF、其他 state estimation 或直接使用 IMU 自带姿态，再单独设计 estimator 层。

---

## 10. 控制与通信频率

旧实机代码已经隐含两层频率：

```text
Motor I/O Thread
     ≈ 500 Hz
         │
         ▼
 latest hardware state
         │
         ▼
RealRobotIO / Backend
       200 Hz
         │
         ▼
MotionRuntime
       200 Hz
         │
         ▼
RL Policy
        50 Hz
```

因此不要求电机通信、MotionRuntime 和 policy 共用一个循环。

建议保留：

- 电机总线独立通信线程；
- backend 固定周期读取最新状态；
- MotionRuntime 保持现有 200 Hz；
- policy 保持现有 decimation / 50 Hz。

实际串口通信频率和 jitter 需要在新 backend bring-up 时实测，不直接假定 `sleep 2 ms` 就等于严格 500 Hz。

---

## 11. 安全职责

安全分成两层。

### Motion Safety

位于 `MotionRuntime`，负责：

- 模式切换；
- 行为状态；
- command limit；
- 控制目标合法性；
- Passive / Stand / Running 等运动状态。

### Hardware Safety

位于 `real_backendd / RealRobotIO`，负责：

- CommandFrame 超时；
- motiond heartbeat 丢失；
- motor communication timeout；
- motor fault；
- IMU timeout；
- NaN / invalid state；
- 姿态异常；
- backend shutdown；
- 紧急阻尼。

基本原则：

```text
motiond 故障
    │
    ▼
real_backendd
仍必须能够主动进入安全状态
```

命令优先级建议：

```text
Hardware Fault
      ↓
Emergency Damping

Command Timeout
      ↓
Safe Damping

Valid CommandFrame
      ↓
Normal Control
```

实机安全不依赖 ROS 2 或上层状态机完成最后一次命令发送。

---

## 12. real_backendd

`real_backendd` 应尽量复用 `mujoco_backendd` 已建立的 daemon 模式。

共用概念：

```text
startup_id
session_id
heartbeat
RobotIOStatus
StateFrame
CommandFrame
command version
backend control request
IPC
```

实机额外增加：

```text
hardware initialization
motor bus lifecycle
IMU lifecycle
hardware watchdog
safe shutdown
```

不要重新建立一套 real-only IPC 协议。

---

## 13. 推荐目录结构

初期推荐保持简单：

```text
quadruped_control/
│
├── backends/
│   ├── mujoco/
│   ├── replay/
│   └── real/
│       ├── include/quadruped/backends/real/
│       │   ├── real_robot_io.hpp
│       │   ├── motor_bus.hpp
│       │   ├── actuator_codec.hpp
│       │   ├── imu.hpp
│       │   └── hardware_config.hpp
│       │
│       └── src/
│           ├── real_robot_io.cpp
│           ├── motor_bus.cpp
│           ├── actuator_codec.cpp
│           ├── imu.cpp
│           └── hardware_config.cpp
│
├── apps/
│   ├── runtime_daemons/
│   │   ├── motiond.cpp
│   │   ├── mujoco_backendd.cpp
│   │   └── real_backendd.cpp
│   │
│   └── hardware_tools/
│       ├── motor_zero/
│       ├── motor_test/
│       └── imu_monitor/
│
└── configs/
    ├── robots/
    │   ├── black.yaml
    │   └── blackW.yaml
    │
    └── hardware/
        ├── black.yaml
        └── blackW.yaml
```

如果初期文件规模较小，不需要预先创建 `motor/`、`imu/`、`safety/` 等多层目录；等代码自然增长后再拆。

---

## 14. 旧仓库迁移关系

| 旧实现 | 新位置 | 处理 |
|---|---|---|
| `real_runner.cpp` | `RealRobotIO + real_backendd` | 拆分 |
| `SerialPack` | `MotorBus + ActuatorCodec` | 重构 |
| Unitree SDK | `backends/real` | 保留 |
| serial / IOPort | `MotorBus` | 保留并整理 |
| `motor_calibration.conf` | `configs/hardware` | 改为带关节名配置 |
| `set_zero` | `apps/hardware_tools` | 迁移 |
| AB5465 parser | `IMU` | 保留 |
| VQF | `IMU` | 初期保留 |
| IMU monitor | `apps/hardware_tools` | 迁移 |
| `secure_protect` | `HardwareSafety` | 重构 |
| ROS wrapper | 无 | 不迁移 |
| `midware` | 无 | 不迁移 |
| `_lowCmd / _lowState` | `CommandFrame / StateFrame` | 替换 |
| 旧 MuJoCo runner | `backends/mujoco` | 不迁移 |
| 旧 joystick / teleop | adapters/input | 不作为实机 backend 内容迁移 |

---

## 15. 推荐实施顺序

### 1. Hardware Baseline

完整整理旧代码中的：

- 串口设备；
- baudrate；
- motor ID；
- leg / actuator 对应关系；
- direction；
- zero offset；
- transmission；
- Unitree SDK 单位；
- IMU 协议；
- IMU 坐标；
- 安全逻辑；
- 真实通信周期。

输出一份明确的硬件行为基线。

### 2. Real backend skeleton

建立：

```text
RealRobotIO
real_backendd
HardwareConfig
```

先接入现有 IPC、heartbeat、session 和 RobotIO 生命周期，不控制真实电机。

### 3. Motor read-only

完成四路串口、电机状态读取和 actuator mapping。

只产生 `StateFrame`，禁止有效力矩输出。

### 4. IMU

移植：

```text
AB5465
VQF
```

完成完整实机 `StateFrame`。

### 5. Safe command bring-up

依次验证：

```text
单电机
→ 单腿
→ 全部关节
```

重点确认：

```text
direction
zero
q
dq
tau
Kp
Kd
command timeout
safe damping
```

### 6. MotionRuntime

依次接入：

```text
Passive
→ GetUp
→ Stand
→ GetDown
```

这一阶段原则上不修改 MotionRuntime 架构。

### 7. RL 与 blackW

完成：

```text
RL locomotion
policy switching
wheel actuator
blackW behaviors
```

实机链路稳定后再移除旧 runtime。

---

## 16. 设计原则

### 原则 1：硬件细节止于 backend

以下内容不得进入 MotionRuntime：

```text
vendor SDK
serial device
motor id
motor direction
gear ratio
zero offset
raw motor unit
```

---

### 原则 2：StateFrame / CommandFrame 是边界

实机与仿真都使用统一：

```text
StateFrame
CommandFrame
RobotIOStatus
```

上层不得因为 backend 不同而建立特殊数据结构。

---

### 原则 3：black / blackW 共用实现

型号差异由配置和 actuator mapping 表达，避免复制：

```text
RealRobotIOBlack
RealRobotIOBlackW
```

除非后续硬件架构发生本质变化。

---

### 原则 4：先复现，再整理

迁移初期优先保证：

```text
旧硬件行为
      ↓
新 backend
```

结果一致。

旧代码中的方向、减速比、单位换算等关系在完全理解前不主动“简化”。

---

### 原则 5：实机安全独立于上层

backend 必须能够处理：

```text
motiond crash
ROS 2 crash
command timeout
motor fault
IMU fault
process shutdown
```

并自行进入安全状态。

---

### 原则 6：控制逻辑不进入硬件线程

电机通信线程主要负责：

```text
send
receive
timestamp
latest state
communication status
```

运动状态机和策略执行继续留在 MotionRuntime。

---

## 17. 当前需要进一步确认的内容

进入具体实现前，还需要从旧仓库和硬件资料中确认：

1. Unitree SDK 中 q / dq / tau 的实际单位和内部减速比语义；
2. `6.33`、`9.1`、`2.5` 等历史系数分别属于电机内部减速器还是外部机械传动；
3. black / blackW 每个电机的实际 ID；
4. 每条腿串口与 actuator 的准确映射；
5. wheel 指令的方向和速度单位；
6. motor calibration 文件中两行数据各自的具体含义；
7. IMU 安装方向与 body frame 的准确变换；
8. VQF 输入的实际采样频率；
9. 旧 `secure_protect` 中所有触发条件及最终电机行为；
10. shutdown、掉线和串口异常时旧系统实际执行的安全动作。

这些内容确认后，再正式确定 `HardwareConfig` 字段和 `ActuatorCodec` 数学模型。

---

## 18. 最终目标

完成迁移后，项目应能够只替换 backend 即切换运行环境：

```text
                 MotionRuntime
                      │
                  RobotIO
          ┌───────────┼───────────┐
          ▼           ▼           ▼
       MuJoCo       Replay       Real
```

black / blackW 的运动控制、RL policy、ROS 2 控制接口和行为系统继续共用同一套上层架构。

实机 backend 只负责把：

```text
CommandFrame
```

可靠、安全地转化为物理执行器命令，并把真实硬件状态重新整理成：

```text
StateFrame
```

供上层使用。
