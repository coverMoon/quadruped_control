# blackW 与共同行为后续开发指导

## 1. 文档目的

本文档补充《仿真部分后续指导.md》，用于冻结 blackW 接入以及 black/blackW 共同行为的后续开发边界。当前只完成规格、接口和实施顺序设计，不在本次修改中接入 blackW 代码、策略或 MuJoCo 场景。

本阶段的核心决定是：

- black 和 blackW 继续共用 `core → motion → RobotIO → backend` 运行链路；
- Retry 不是 blackW 专属逻辑，应设计为可由 black 和 blackW 共用的行为；
- Event chain 也使用公共行为接口，但 black 当前只预留与 blackW 相同的接口，不根据 12 关节 black 猜测或伪造轮足事件；
- Car、Bridge、Low-bar 是 blackW 的轮足专用行为，暂不扩展为 black 的公共能力；
- `policy_switch.yaml` 只控制 RL 策略循环，不承载 Retry、Event chain 或固定姿态行驶模式。

## 2. 当前实现基线和主要缺口

### 2.1 black 与 blackW 的关节模型

black 使用 12 个腿关节，blackW 使用 16 个关节，每条腿增加一个轮关节。建议冻结 blackW 的逻辑顺序为：

```text
FL_hip_joint,   FL_thigh_joint, FL_calf_joint, FL_wheel_joint
FR_hip_joint,   FR_thigh_joint, FR_calf_joint, FR_wheel_joint
RL_hip_joint,   RL_thigh_joint, RL_calf_joint, RL_wheel_joint
RR_hip_joint,   RR_thigh_joint, RR_calf_joint, RR_wheel_joint
```

该顺序必须由 `RobotModel` 和策略映射显式定义。不能使用 URDF/MJCF 的声明顺序；当前 blackW URDF 的声明顺序包含 RR 在 RL 前面的情况。

四个轮关节必须显式声明：

```yaml
role: wheel
position_limited: false
```

轮关节虽然在 URDF 中是 revolute，但不能按普通腿关节处理，也不能根据超大位置范围自动推断为轮子。

### 2.2 当前代码对 blackW 的准备程度

当前 core 已经具备：

- `kMaxJoints == 16`；
- `JointRole::Wheel`；
- 显式 `ControlMode`；
- `JointImpedance` 的目标速度字段；
- 固定容量 StateFrame 和 CommandFrame。

当前仍然缺少：

- blackW RobotModel、ControllerConfig 和 SimulationConfig；
- blackW MuJoCo 场景、关节和 actuator 资源；
- 16 关节 RL 观测、动作和历史配置；
- wheel 的策略动作映射和动作缩放；
- MotionRuntime 的腿/轮混合行为命令生成；
- 共用 Retry 和 Event chain 的行为注册、执行和结果语义。

当前 MuJoCo 后端实际支持的控制模式仍以 Disabled、Damping 和 JointImpedance 为主。第一阶段不必强行引入独立的轮 Velocity 执行路径。按照公共约束，轮子可以使用：

```text
JointImpedance
kp = 0
target_position = 当前轮角
target_velocity = 目标轮速
kd = 轮速阻尼
```

但 MotionRuntime 需要能够在同一个 CommandFrame 中同时生成腿部位置目标和轮子速度目标。

## 3. 行为分类和公共边界

### 3.1 公共行为名称

公共请求接口通过 `ModeRequest.behavior_name` 选择行为，建议冻结以下名称：

```text
rl_locomotion
retry
event_chain
car_drive
bridge_drive
low_bar_drive
```

其中：

| 行为 | black | blackW | 第一阶段定位 |
|---|---|---|---|
| `rl_locomotion` | 支持 | 支持 | RL 策略行为 |
| `retry` | 支持 | 支持 | 公共恢复行为 |
| `event_chain` | 预留接口 | 支持 | 公共事件链接口 |
| `car_drive` | 不支持 | 支持 | 轮足专用行为 |
| `bridge_drive` | 不支持 | 支持 | 轮足专用行为 |
| `low_bar_drive` | 不支持 | 支持 | 轮足专用行为 |

不增加以下公共枚举：

```text
MotionMode::Retry
MotionMode::Car
MotionMode::Bridge
MotionMode::EventChain
```

这些行为在状态层统一归入：

```text
MotionMode::Running
```

具体功能通过：

```text
MotionStatus.behavior_name
MotionStatus.behavior_phase
MotionStatus.policy_name
```

展示。例如：

```text
mode = Running
behavior_name = retry
behavior_phase = locked
policy_name = himloco_flat
```

### 3.2 行为与策略分离

`configs/policies/<robot>/policy_switch.yaml` 只列出 RL 策略，例如：

```text
himloco_flat
himloco_obstacle
himloco_stair
```

以下内容不能放入策略循环：

```text
retry
car_drive
bridge_drive
low_bar_drive
event_chain
```

行为可以在内部引用当前 RL 策略的默认姿态，例如从 Car 返回 RL 时读取当前策略的 `default_joint_positions`，但行为本身不应伪装成一个策略名称。

## 4. Retry 公共行为设计

### 4.1 目标

Retry 用于机器人进入不适合继续运行策略的姿态后，平滑进入可人工搬运的恢复状态。其控制逻辑在 black 和 blackW 中保持一致，差异只来自机器人配置和关节角色。

### 4.2 公共生命周期

```text
当前主动行为
    ↓ Retry 请求
清除 RL 输出、策略切换和速度命令
    ↓
从当前姿态插值到 retry_default_joint_positions
    ↓
Retry ready / locked
    ↓
保持恢复姿态，允许人工搬运
```

Retry 期间必须：

- 清零或拒绝新的 BaseCommand；
- 清除 pending policy switch；
- 禁止 navigation mode；
- 不自动退出；
- 允许 `GetUp` 和 `EnterPassive` 打断；
- 命令提交失败时进入安全终态并返回明确的 ModeResult。

### 4.3 关节命令规则

对所有 position-limited 关节：

```text
target_position = 当前姿态到 retry 姿态的插值
target_velocity = 0
kp = retry 配置
kd = retry 配置
```

对 Wheel 关节：

```text
target_position = 当前轮角
target_velocity = 0
kp = 0
kd = retry 配置
```

black 没有 Wheel 关节，因此不需要轮子分支；公共 Retry 行为应根据 `RobotModel.joints[i].role` 生成命令，而不是复制 black 和 blackW 两份实现。

### 4.4 配置

建议使用：

```text
configs/behaviors/black/retry.yaml
configs/behaviors/blackW/retry.yaml
```

两份配置共享同一字段结构：

```yaml
prepare_cycles: 100
retry_default_joint_positions: [...]
kp: [...]
kd: [...]
```

配置数组必须和对应 RobotModel 的显式关节顺序一致。配置加载器必须校验关节数、关节名和每个数组长度。

### 4.5 Retry 验收

至少覆盖：

- black 进入 Retry；
- blackW 进入 Retry；
- Retry 期间速度命令不会继续控制机器人；
- Retry 期间策略切换请求被清除或拒绝；
- 轮子保持当前角度且速度为零；
- `A/Num0` 进入 GetUp；
- `LB+X/P` 进入 Passive；
- 命令过期后不会继续输出旧的主动命令。

## 5. Event chain 公共接口设计

### 5.1 设计目标

Event chain 是一组按顺序执行的姿态、轮驱动和姿态加轮驱动事件。black 和 blackW 使用相同的请求、配置和状态接口，但每个机器人是否支持具体事件由机器人能力和配置决定。

当前不要求 black 立即实现旧 blackW 的翻墙动作。black 先保留公共接口和校验语义，不能因为 black 没有轮子就复制一套不同的事件链协议。

### 5.2 公共配置结构

建议统一使用：

```text
configs/behaviors/black/event_chain.yaml
configs/behaviors/blackW/event_chain.yaml
```

配置结构保持一致：

```yaml
exit_to_rl_cycles: 300
interpolation: smoothstep
joint_names: [...]
wheel_radius: 0.103
kp: [...]
kd: [...]
wheel_velocity_sign: [...]
events:
  - name: settle
    type: pose
    transition_cycles: 200
    hold_cycles: 100
    dof_pos: [...]

  - name: drive_forward
    type: drive
    wheel_group: rear
    distance_m: 0.4
    speed_mps: 0.8
    timeout_cycles: 600

  - name: pose_and_drive
    type: pose_drive
    wheel_group: front
    distance_m: 0.2
    speed_mps: 0.2
    timeout_cycles: 300
    dof_pos: [...]
```

公共事件类型冻结为：

```text
pose
drive
pose_drive
```

所有事件包含 `name` 和 `type`；其余字段按类型要求：

```text
pose: transition_cycles, dof_pos, 可选 hold_cycles
drive: wheel_group, distance_m, speed_mps, timeout_cycles, 可选 hold_cycles
pose_drive: 上述 pose 与 drive 字段，可选 hold_cycles
```

### 5.3 black 的当前边界

black 当前没有 Wheel 关节，因此：

- `pose` 事件可以使用相同接口，但暂不承诺具体任务动作；
- `drive` 和 `pose_drive` 如果要求 wheel_group，必须在启动期被明确拒绝；
- 拒绝原因必须通过配置校验或 ModeResult 返回，不能静默忽略轮子字段；
- 不为 black 伪造 wheel group、轮速或位移估计；
- 不改变 Event chain 的公共字段名称和请求语义。

后续如果 black 出现不依赖轮子的多阶段动作，可以直接复用 Event chain 调度器，不需要改变上层接口。

### 5.4 Event chain 状态语义

当前运行时使用事件名直接暴露细粒度阶段，并保留公共终态和返回阶段：

```text
behavior_name = event_chain
behavior_phase = <event.name>
behavior_phase = policy_transition
behavior_phase = completed
behavior_phase = failed
```

事件链完成或失败后必须有明确结果：

- 完成：进入配置指定的最终行为，通常是 Stand 或 RL pose transition；
- 被 Passive、GetDown 或 Retry 打断：当前事件链请求返回被中断；
- 事件超时：返回 Failed，并进入安全处理；
- 目标关节数量、轮组或速度参数非法：启动期拒绝，不进入运行态。

### 5.5 Event chain 当前边界

事件序列、仿真轮半径、轮方向、距离、速度、周期和超时已直接取自 `rl_sar`。当前仍不把
以下项目声明为已验收：

- 轮距；
- 真实轮胎打滑下编码器位移到机体位移的精确换算；
- 当前 MuJoCo 场景中完整 0.30 m 墙体链的碰撞接触成功判定；
- 真实硬件承载能力；
- 翻墙失败后的自动恢复动作。

调度器已经覆盖平均编码器位移、正负距离、超时失败和返回 RL；上述边界仍需专用墙体场景
与真实规格验证。

## 6. blackW 专用行为边界

### 6.1 Car、Bridge、Low-bar

这三种行为共享一个差速轮行为实现，配置不同：

```text
目标姿态
prepare_cycles
exit_to_rl_cycles
max_x
max_yaw
wheel_velocity_scale
yaw_to_wheel_velocity
wheel group 和 sign
```

它们都采用：

```text
腿部：位置阻抗
轮子：kp=0 的速度目标阻抗
```

进入行为：

```text
当前姿态 → 特殊姿态插值 → 驱动
```

返回 RL：

```text
当前特殊姿态 → 当前 RL 策略默认姿态插值
轮速清零
进入 rl_locomotion
```

不应通过下标判断左右轮，例如：

```cpp
i == 3 || i == 11
```

必须通过显式配置取得轮组和左右侧信息。

### 6.2 输入和状态切换

为了复刻旧 `rl_sar` 的交互语义，后续 command 层可以继续使用：

```text
Num0 / A             GetUp
P / LB+X             Passive
Num1 / RB+DPadUp     RL locomotion
Num2 / RB+DPadRight  Bridge drive
Num3 / RB+DPadDown   Low-bar drive
Num4 / RB+DPadLeft   Car drive
Num5 / B             Retry
Num6 / LB+DPadUp     Event chain
Num9 / RB+B          GetDown
```

GetUp 和 GetDown 期间保留待进入行为：

```text
pending_behavior_name
```

例如用户在 Passive 时选择 Car：

```text
记录 pending_behavior_name = car_drive
执行 GetUp
GetUp 完成后自动进入 car_drive
```

Retry 和 Event chain 也可以通过同一套 StartBehavior 和中断语义接入，不需要新增一套 FSM 框架。

## 7. 后续阶段安排

### 阶段 9：blackW 规格和公共行为接口冻结

只做事实确认、配置结构和接口文档，不接 Torch，不实现翻墙动作。

产物：

```text
configs/robots/blackW.yaml
configs/controllers/blackW.yaml
configs/simulation/blackW_mujoco.yaml
configs/policies/blackW/...
configs/behaviors/black/retry.yaml
configs/behaviors/black/event_chain.yaml
configs/behaviors/blackW/retry.yaml
configs/behaviors/blackW/event_chain.yaml
docs/blackW_规格记录.md
```

验收：

- blackW 关节顺序冻结；
- Leg/Wheel 角色冻结；
- MuJoCo joint、actuator、IMU 名称确认；
- 策略输入输出尺寸有可靠来源；
- Retry 配置结构在 black 和 blackW 间一致；
- Event chain 配置结构在 black 和 blackW 间一致；
- 所有未确认的轮方向、减速比、轮半径和位移判断都列为阻塞项。

### 阶段 10：RobotIO、基础动作和共用 Retry

先完成：

- blackW 16 关节 RobotModel；
- blackW MuJoCo 映射；
- 混合腿轮 CommandFrame；
- blackW Passive、GetUp、Stand、GetDown；
- black 和 blackW 共用 Retry 行为；
- Event chain 公共配置加载、能力检查和请求接口。

本阶段不接 blackW Torch RL，不实现复杂翻墙事件。

### 阶段 11：RL 数据路径配置化

当前状态：已完成。black 和 blackW 使用同一固定容量、启动期配置的数据路径，blackW
三项 TorchScript 策略及运行中切换已通过 MuJoCo 闭环测试。

将当前 black 的固定：

```text
12 joints / 45 observations / 6 history frames / 12 actions
```

改为启动期配置的固定容量实现，支持：

- black 现有 12 关节行为不回归；
- blackW 16 关节策略；
- policy_dof_indices；
- 每个动作独立缩放；
- 腿/轮混合动作；
- wheel 动作限幅和符号转换；
- 运行中策略切换。

### 阶段 12：blackW 轮足行为

当前四种运行时行为均已完成，专用障碍场景验收仍待完成。阶段内容为：

1. Car drive（已完成）；
2. Bridge drive（已完成）；
3. Low-bar drive（已完成）；
4. Event chain 的轮驱动事件（已完成）；
5. 事件完成、超时、失败和返回 RL（已完成）；
6. MuJoCo 编码器位移和差速轮闭环验证（已完成），各专用障碍接触验证（待完成）。

如果 Event chain 的 pose-only 事件适用于 black，可在不改变接口的前提下单独启用；依赖轮子的事件继续只在 blackW 能力配置中开放。

## 8. 测试要求

### 公共行为测试

- 行为名称注册和未知行为拒绝；
- Retry 的进入、锁定、GetUp、Passive 和中断；
- black/blackW 使用同一 Retry 生命周期；
- Event chain 配置字段和事件类型校验；
- black 对 wheel 事件明确拒绝，不静默执行；
- Event chain 超时、失败和中断结果正确。

### 机器人模型测试

- 12/16 关节数量校验；
- 关节名和顺序校验；
- Wheel role 显式校验；
- position_limited 语义校验；
- wheel 目标速度和位置不被错误截断；
- mixed leg/wheel command 的 KP、KD、位置和速度目标正确。

### 回归测试

每次 blackW 行为改动至少运行：

```text
./scripts/build.sh
./scripts/build.sh --mujoco
./scripts/test/ctest.sh default
./scripts/test/ctest.sh mujoco
```

涉及 RL 数据路径时再运行：

```text
./scripts/build.sh --rl
./scripts/test/ctest.sh rl
```

必须保持现有 black flat、obstacle 和三进程 headless 回归不受影响。

## 9. 未冻结事项

在进入 blackW RobotIO 和轮足行为实现前，仍需确认：

1. 16 个逻辑关节到真实 motor ID 的映射；
2. 四个轮子的正方向和最终 sign；
3. 轮半径、轮距和减速比；
4. 轮子目标速度单位；
5. 新 URDF 与真实机械零位的对应关系；
6. calf、thigh、hip 的真实限制；
7. blackW 当前使用的策略模型文件；
8. blackW 策略是否为 16 维动作或使用 policy_dof_indices；
9. Car、Bridge、Low-bar 姿态是否适用于当前新模型；
10. Event chain 的障碍物、接触和失败判定；
11. Retry 姿态在 black 当前机械限制下是否需要单独配置。

未冻结的字段不得在代码中使用猜测值。对于没有可靠来源的行为，应先保留接口并在启动期明确报告未配置或不支持。
