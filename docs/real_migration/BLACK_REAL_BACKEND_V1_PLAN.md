# Black real backend v1：冻结契约与实施顺序

本文件是当前 Black 实机接入的交接入口。训练侧契约来自
`alldog_mjlab/.ai/MIGRATION.md` §19.13–§19.18；旧硬件源码事实见
[REAL_HARDWARE_BASELINE.md](REAL_HARDWARE_BASELINE.md)，通用后端边界见
[REAL_ROBOT_BACKEND_MIGRATION_REFERENCE.md](REAL_ROBOT_BACKEND_MIGRATION_REFERENCE.md)。
本文件记录已决定的 Black v1 行为、仍需实机确认的数值，以及 Unit 0–7 的顺序。
RealRobotIO 和 real_backendd 尚未实现。

## 1. 来源与当前状态

| 来源 | 权责 |
|---|---|
| `alldog_mjlab` | 训练侧 observation、action、joint order 与 policy cadence 的权威 |
| `rl_sar-for-super-dog` | legacy Black policy runtime 的兼容性参照 |
| `real_robot` | 旧实机已验证的通信、电机换算、标定与 IMU 路径 |
| `quadruped_control` | 新部署运行时；负责规范化状态、命令、IPC 与执行安全 |

`quadruped_control` 不反向定义训练 observation/action contract。当前可用的正式
运行链路是 MuJoCo/replay；Black 实机链路尚无 production backend。

**Unit 0 当前受阻。**工作树留有未提交的 timing metadata 实施尝试，不能作为
已完成能力引用。该尝试的 core/motion/IPC/MuJoCo 测试 8/8 通过，但三进程
`ros2_headless.sh` 在 motiond 故障场景失败（`effective_command_sequence=0`）。
原因是尝试让 IPC `RemoteRobotIO` 一律返回 host monotonic 时间，而当前
`mujoco_backendd` 的 StateFrame 和命令校验使用 MuJoCo 仿真时间。命令被判为来自
未来。Unit 0 继续前必须先冻结**跨 IPC 的控制时钟域选择**，保持同一连接的状态、
命令和 target 时间可比较；heartbeat 的 host monotonic 时间属于独立的存活检测。
不得靠把 MuJoCo/replay 改成 host wall/steady 时间消除测试失败。

## 2. Black policy 部署契约

- 关节和 action 顺序：FL → FR → RL → RR；每腿 hip → thigh → calf，共 12 个。
- actor 输入为单帧 45-D：command 3、body angular velocity 3、projected gravity 3、
  `q - q_default` 12、`dq` 12、previous action 12。PPO 路径无 HIM history。
- actor 输出为 12-D raw action；`q_policy = q_default + 0.25 * raw_action`。
- 每 20 ms（50 Hz）生成新 policy target；5 ms 控制刷新可在中间保持该 target。
- 默认第一帧 previous action 为零；position clamp 与最大 target jump 属部署安全层。

## 3. 电机映射、标定与换算

canonical policy/RobotModel index 与旧实机 message/motor index 的映射：

```text
policy index 0..11（FL,FR,RL,RR）:
real_robot index = [3,4,5, 0,1,2, 9,10,11, 6,7,8]
bus = /dev/leg_{real_robot_index / 3}
SDK requested motor ID = real_robot_index
```

每腿一条总线。以上是旧软件请求关系；端口实际接线、各电机 ID 与正方向仍须
bring-up 逐个确认。hip/thigh 的传动比 `G=6.33`、方向 `s=+1`；calf 为
`G=15.825`、`s=-1`。按旧实机 motor index `i`，标定包含 rotor-side
`straight_i` 和 `creep_i`，首次有效反馈 `raw_start_i` 后计算：

```text
R_i = -round((raw_start_i - creep_i) / (2π)) * 2π
O_i = R_i - straight_i + calf_correction_i
calf correction: i=5/11 为 +46.66°*15.825，i=2/8 为 -46.66°*15.825
```

```text
feedback: q=s*(motor.Pos+O_i)/G, dq=s*motor.W/G, tau=s*motor.T*G
command:  motor.Pos=s*G*q_target-O_i, motor.W=s*G*dq_target
          motor.K_P=Kp_joint/G², motor.K_W=Kd_joint/G²
          motor.T=s*tau_ff_joint/G
```

持久标定按 robot identity 和关节名绑定，12 项完整、有限、版本有效；启动多圈
`R_i` 是当次运行状态。标定缺失、错位或无效时 **fail closed**，不沿用旧程序
零数组继续运行的退路。bus、sign、gear、固定 calf 修正、标定和 runtime offset
均属于 `backends/real`，MotionRuntime 与 policy 只见 canonical joint 单位。

## 4. 执行器、IMU 与尚未确认的硬件上限

| 数值 | 确切语义 |
|---|---|
| 20 N·m | MjLab training actuator effort limit |
| 33.5 N·m | `rl_sar ComputeOutput` 诊断用 torque clamp；未证明为实机输出限幅 |
| 23.7 / 59.25 N·m | 当前 RobotModel hip/thigh 与 calf 的 joint-side `max_effort` 配置；未证明为固件/实体 ceiling |
| 实机 current/effort ceiling | **UNCONFIRMED**；主动输出前须确认 SDK/固件与保护位置 |

旧 Black RL 实际发送 joint-side `q_target`、`dq_target=0`、`Kp=40`、
`Kd=1.2`、`tau_ff=0`，再按上节 `/G²` 转为 motor gain。`motor.T=0`
仍可由电机侧 PD 产生力矩；不能用前馈为零证明输出安全。

当前实体 Black 的 AB5465 安装和配置已确认与 legacy `real_robot` 相同。
v1 保留 `diag(1,-1,-1)` sensor→body 轴变换、gyro deg/s→rad/s、
VQF 6D orientation；canonical 输出为 body→world `wxyz` quaternion 和
body-frame rad/s 角速度。不额外引入 Black 安装旋转。原始 accel 物理语义、
采样时间质量、VQF 启动就绪、丢包、振动和 bias 仍需 bring-up；它们不是
IMU mounting blocker。

## 5. 时序、新鲜度与失败边界

MotionRuntime 控制刷新为 5 ms，Black policy 更新为 20 ms，现有
CommandFrame 帧有效期为 10 ms。高层速度命令过期时 `vx/vy/wz=0`，不因此
直接退出 RL；joint 或 IMU stale 属状态完整性失败，不能仅清零速度后继续推理。
单次有效但迟到的推理是有限保持上一已验证 target 的候选；持续失约、异常、
错维或 NaN/Inf 输出必须退出 RL。当前 MotionRuntime 对 >20 ms 的推理在返回后
转 Passive/Disabled；实机最终 watchdog 必须在独立执行侧，即使 motiond/Torch
停住仍检查 frame、target、反馈、fault 和 session。具体 joint/IMU stale 门槛、
采样偏差及安全阻尼值待 bring-up，不照搬旧的 0.5 s IMU timeout。

`SAFE HOLD` 为经验证的 Damping：`Kp=0,dq_target=0,Kd=安全值,tau_ff=0`，
不继续 RL position target；`Disabled` 无主动 PD/FF 输出，不预设为物理断电。
外部 E-stop 覆盖所有软件模式，具体通路另作实机验证。

## 6. v1 进程与线程

```text
ros2_gateway → motiond(MotionRuntime + 同步 Policy::forward)
             → IPC → real_backendd → RealRobotIO
                                      ├─ 4 × motor bus worker
                                      ├─ 1 × AB5465 / VQF worker
                                      └─ 1 × execution / safety supervisor
```

`real_backendd` 持有共享内存、session、heartbeat、命令接收与状态发布，
主循环不做电机事务。`read_latest()` 只快照最新有效样本、映射 canonical
StateFrame 并计算各样本 age；`submit()` 只校验并缓存完整命令。两者均非阻塞，
接受命令不等于物理执行。四条 bus 各自独占 fd/SDK；一个 bus 阻塞不能拖停
其他 bus。supervisor 独立于 motiond/Torch，决定真正可发送的命令和安全回退。
只有四个所需 bus 都成功接受/发送同一 snapshot，才可推进全帧
`effective_command_sequence`；无 actuator ACK 时不能宣称物理执行已确认。

v1 保留同步 `Policy::forward()`、现有 history 和 previous_action 行为；
**前提**是硬件执行与最终 watchdog 独立运行于 real_backendd/RealRobotIO。
实测同步推理抖动仍不可接受时，另开异步调度 v2，不在第一版重写 policy API。
v1 先支持 Disabled、Damping、JointImpedance；Velocity/Torque 明确拒绝。

## 7. Unit 0：下一 production 单元

通用 CommandFrame 必须区分帧新鲜度与 semantic target 新鲜度，增加等价于
`target_generated_at_ns`、`target_expires_at_ns` 的时间语义。传感器采样、
StateFrame 生成、成功推理后的 target 生成、CommandFrame 实际生成和执行时间
须分开；不能把同步推理前的 state timestamp 当作命令生成时间。

RL 每 20 ms 成功生成新 target；其余三个 5 ms 周期可重包装 CommandFrame，
但不刷新 target age。允许一次 missed update 的 Black target hard lifetime
按 `5 ms × 4 × 2 ≈ 40 ms` 推导，不能被重包装无限延长。GetUp、Stand、
GetDown、Passive 等普通周期命令默认在每周期生成新 semantic target，
不暗中获得 RL hold 寿命。这些是 core 通用字段，不是 RL-only hack。

Unit 0 的首个未决前提是 IPC 控制时钟域：同一 backend 的 StateFrame、target、
CommandFrame 与执行侧校验必须同域；MuJoCo 保持 simulation time、replay 保持
logical time，实机 safety 使用 host monotonic。当前 IPC 代理一律使用 host
monotonic 的未提交尝试已在 headless 测试失败，不能据此标记 Unit 0 COMPLETE。
先确定 IPC 如何向 motiond 提供当前 backend 的控制时钟，再继续 schema、
validation、session/reset、clock 和三进程回归；heartbeat 可继续使用独立
host monotonic 存活计时。

## 8. 后续实施顺序

| Unit | 范围 | 禁止事项与完成门槛 |
|---|---|---|
| 0 — core target timing | 通用 target 时间、同域 clock、IPC wire/schema、RL reuse/reset | 不接硬件；core/motion/IPC/MuJoCo/replay 与三进程 headless 全通过 |
| 1 — hardware config + pure conversion | bus/ID、gear/sign、calf correction、版本化标定、multi-turn、IMU 静态配置 | 不打开 serial；错误身份/缺项/NaN 与数学 round trip 离线验证 |
| 2 — read-only RealRobotIO | fake transport worker、latest sample cache、age/fault/StateFrame | 不做主动电机输出；验证非阻塞、坏包与单 bus/IMU 失效 |
| 3 — real_backendd + IPC read-only | session、heartbeat、状态与诊断发布 | 不做主动电机输出；读取完整 canonical state 的端到端链路成立 |
| 4 — execution supervisor | frame/target watchdog、有限 hold、模式回退、partial bus、fault latch | 先只用 fake transport；motiond 停滞与 session 变化测试通过 |
| 5 — actual transport | 固定版本 motor SDK、AB5465、VQF、serial | 先只读；核对 12 电机与 IMU 反馈/故障，不直接启用 RL |
| 6 — hardware safety bring-up | Disabled→单电机 Damping→单腿→四腿→GetUp→Stand→GetDown | 每步台架记录，确认固件失联与 E-stop 行为后再放行 |
| 7 — Black PPO real enable | 零速度命令，再低速前进 | 前序门槛全通过；记录时延、freshness、故障与安全退出 |

首个实机端到端成果是 **12 motor feedback + IMU → canonical StateFrame →
real_backendd/IPC → motiond/gateway**，主动电机输出 Disabled，并能观察
freshness、fault 和 calibration validity。第一目标不是 RL 行走。

实体接线、逐电机 ID/方向、标定身份与允许上电姿态、实机 effort/current ceiling、
安全 Kd、SDK Disabled 编码、firmware timeout、E-stop 通路和传感器时限均属
hardware/bring-up 决策。它们不能由训练限幅、诊断限幅或离线测试代替。
