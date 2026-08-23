# 当前状态与参考工程差距

## 1. 一句话结论

当前仓库已经完成 **black 的 MuJoCo + TorchScript + ROS 2 三进程仿真控制链路**。
`ros2_gateway`、`motiond` 和 `mujoco_backendd` 通过固定容量本机 IPC 独立运行，并已覆盖
正常运动、策略切换、reset 和单进程退出后的安全退路。它仍不是 `rl_sar + real_robot`
的完整替代品，主要缺口转向轮足专用行为和实机执行链路。

按不同目标估算完成度：

| 目标 | 当前完成度 | 说明 |
| --- | ---: | --- |
| black MuJoCo RL 行走 | 约 90% | flat/obstacle 推理与切换完成，仍需更长期稳定性和扰动验证 |
| black 的 ROS 2 仿真替代 | 约 95% | headless/GUI 三进程链路、launch 和进程监督完成，仍需更长期稳定性验证 |
| black 实机部署 | 约 25% | 缺硬件 RobotIO、IMU、通信、校准、安全监督和部署程序 |
| blackW、机械臂及其他行为 | 约 60% | blackW MuJoCo 基础动作、Retry 和 16 维 RL 已完成，轮足事件链与机械臂未实现 |

这些比例是功能范围估计，不是代码行数进度。

## 2. 已经具备的运行链路

```text
/cmd_vel + action/service
          ↓
     ros2_gateway
          ↓  BaseCommand / ModeRequest
       motiond
          ↓  RobotIO StateFrame / CommandFrame
  mujoco_backendd
          ↓
   MuJoCo headless
```

已经实现：

- 12/16 关节固定容量核心帧、单调时间戳、显式控制模式和完整数值校验；
- `RobotIO` 作为 motion 与仿真/硬件之间的唯一边界；
- black/blackW 机器人、控制器和共同行为参数的启动期 YAML 加载；
- black 的 `Passive → GetUp → Stand → RL Running → GetDown`；
- blackW 无 Torch 的 `Passive → GetUp → Stand → Retry → GetDown`；
- black/blackW 固定容量、启动期配置的观测、历史、策略关节映射和动作转换；
- black 的 flat/obstacle 与 blackW 的 flat/obstacle/stair TorchScript 策略；
- 两种机器人共用的运行时策略切换、历史清零和请求终态管理；
- MuJoCo 无头测试和单进程官方 Simulate 调试界面；
- 固定容量共享内存 wire schema、latest slot、SPSC 请求队列和远程 `RobotIO`；
- schema、robot identity、ordered joint names、startup 和 session 校验；
- `/cmd_vel`、GetUp、GetDown、StartBehavior、SwitchPolicy、EnterPassive 和 ResetFault；
- MotionStatus、RobotIOStatus、StateDiagnostic 和 ModeResult 发布；
- backend reset 后 session 更新及跨 session 旧数据拒绝；
- gateway、motiond 或 backend 退出时的命令超时和活动请求失败语义；
- 不依赖 ROS 2 的故障注入矩阵、固定字段运动诊断和命令过期恢复测试；
- 带机器人身份校验的 StateFrame CSV、只读 ReplayRobotIO 和目标/反馈诊断 CSV；
- 三进程 ROS 2 headless 端到端测试。

## 3. 与 rl_sar 的差距

| 能力 | 当前状态 | 后续原则 |
| --- | --- | --- |
| black flat/obstacle 推理 | 已完成 | 保持现有数据路径回归测试 |
| flat/obstacle 运行时切换 | 已完成 | 复用既有请求和姿态过渡语义 |
| himloco、down、MoE、arm 等策略 | 未接入 | 按实际任务逐个增加 |
| 完整 FSM 和组合按键 | 仅当前基础动作与行为请求 | 扩展 MotionRuntime，不复制旧 FSM 框架 |
| `/cmd_vel` | 已接入 ROS 2 gateway | 保持最新值和 200 ms 默认超时语义 |
| 手柄输入 | 已接入 command 侧 `controller_input` 和规范化 `/joy` | 继续做真实设备和长期稳定性验证 |
| blackW 轮足控制 | 基础动作、Retry 和 16 维 RL 已完成 | 阶段 12 后按真实需求实现轮足事件链 |
| CSV、绘图和状态 UI | 已有低频诊断 CSV 和只读回放；无绘图工具 | 后续按真实分析需求增加绘图 |

## 4. 与 real_robot 的差距

ROS 2 顶层接口和三进程本机运行架构已经具备，尚未实现的实机侧能力包括：

- 串口、CAN 或厂商 SDK 对应的硬件 `RobotIO`；
- 电机零点、方向、减速比、扭矩换算和最终执行侧限幅；
- IMU 驱动、姿态估计和传感器健康状态；
- 急停、通信看门狗、失联阻尼、启动握手和故障锁存；
- 实机进程监督、部署配置、日志与回放；
- 里程计和真实环境所需的额外感知接口；
- blackW 的实机执行链路，以及 black_with_arm 的模型、配置和执行链路；
- 跨机器通信协议。

## 5. 当前明确边界

阶段 12 已在 black/blackW 仿真链路上完成测试专用故障注入、诊断字段、CSV 状态日志、
只读 ReplayRobotIO 和命令对比入口。回放生成的命令只保存在回放进程中，不连接执行设备。
blackW Event chain 已直接对照 `rl_sar` 的 10 个事件接入公共 MotionRuntime：支持
`pose`、`drive`、`pose_drive`、smoothstep、轮组平均编码器位移、超时失败、最终姿态保持
以及显式返回当前 RL 策略。black 仍保留空配置，且会在启动期拒绝执行。
Car、Bridge、Low-bar 也已直接采用 `rl_sar` 的姿态、周期、速度换算和限幅，共用一个
固定姿态差速轮实现；左右轮归属改为显式配置，不沿用参考代码的关节下标判断。
以下内容仍属于后续阶段：

- GUI 长时间运行、窗口关闭和多显示环境的稳定性压力验证；
- 真实手柄设备的 SDL/pygame 现场验证；
- blackW 完整 0.30 m 墙体接触链的场景布置、端到端越墙验收和实机参数复核；
- Car、Bridge、Low-bar 对应真实障碍尺寸和专用 MuJoCo 接触场景验收；
- 真实硬件后端。

现有 `apps/mujoco_sim/` 仍是单进程本地调试入口，不等同于三进程 ROS 2 GUI。
blackW 可用自身的 scene、robot、controller、simulation、retry 和 policy 配置启动该入口；
加 `--event-chain-config configs/behaviors/blackW/event_chain.yaml` 后，终端按键 `6` 进入
Event chain；加 `--fixed-drive-config-dir configs/behaviors/blackW` 后，`2`/`3`/`4` 分别进入
Bridge/Low-bar/Car。按键 `1` 可从这些行为平滑返回 RL；`--no-policy` 时仍可验证基础动作和
轮足行为。

## 6. 推荐补齐顺序

1. 阶段 7：已完成三进程 MuJoCo GUI 模式、ROS 2 launch 和统一启动参数；
2. 阶段 8：已完成三进程脚本模块化、构建 target、headless E2E 和 command 输入接线；
3. 阶段 9：已完成 blackW 关节、执行器、传感器、策略规格和配置资产冻结；
4. 阶段 10：已完成 blackW MuJoCo RobotIO、腿轮混合基础动作和共用 Retry；
5. 阶段 11：已配置化 RL 数据路径并接入 blackW 三项策略，black 回归保持通过；
6. 仿真主线阶段 12：已完成故障注入、低频诊断日志和只读回放基线；
7. 在仿真边界稳定后实现目标硬件 `RobotIO`、校准和安全监督。

这个顺序优先保持一条可运行链路，不把 `rl_sar` 和 `real_robot` 的历史包袱重新搬入仓库。
《blackW 与共同行为开发指导》中同名的“阶段 12”指轮足专用行为路线；其四种运行时行为
均已完成，剩余的是各专用障碍场景和真实硬件验收，不与本节仿真主线的故障/回放阶段混为
同一能力。
