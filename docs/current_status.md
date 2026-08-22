# 当前状态与参考工程差距

## 1. 一句话结论

当前仓库已经完成 **black 的 MuJoCo + TorchScript + ROS 2 三进程仿真控制链路**。
`ros2_gateway`、`motiond` 和 `mujoco_backendd` 通过固定容量本机 IPC 独立运行，并已覆盖
正常运动、策略切换、reset 和单进程退出后的安全退路。它仍不是 `rl_sar + real_robot`
的完整替代品，主要缺口转向 blackW、故障注入/回放和实机执行链路。

按不同目标估算完成度：

| 目标 | 当前完成度 | 说明 |
| --- | ---: | --- |
| black MuJoCo RL 行走 | 约 90% | flat/obstacle 推理与切换完成，仍需更长期稳定性和扰动验证 |
| black 的 ROS 2 仿真替代 | 约 95% | headless/GUI 三进程链路、launch 和进程监督完成，仍需更长期稳定性验证 |
| black 实机部署 | 约 25% | 缺硬件 RobotIO、IMU、通信、校准、安全监督和部署程序 |
| blackW、机械臂及其他行为 | 低于 20% | 当前刻意只实现 12 关节 black 路径 |

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
- black 机器人、控制器和 RL 参数的启动期 YAML 加载；
- `Passive → GetUp → Stand → RL Running → GetDown`；
- 与 black himloco 配置一致的观测顺序、6 帧历史、动作缩放、KP/KD；
- 真实 flat/obstacle TorchScript 策略、运行时切换和请求终态管理；
- MuJoCo 无头测试和单进程官方 Simulate 调试界面；
- 固定容量共享内存 wire schema、latest slot、SPSC 请求队列和远程 `RobotIO`；
- schema、robot identity、ordered joint names、startup 和 session 校验；
- `/cmd_vel`、GetUp、GetDown、StartBehavior、SwitchPolicy、EnterPassive 和 ResetFault；
- MotionStatus、RobotIOStatus、StateDiagnostic 和 ModeResult 发布；
- backend reset 后 session 更新及跨 session 旧数据拒绝；
- gateway、motiond 或 backend 退出时的命令超时和活动请求失败语义；
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
| blackW 轮足控制 | 未实现 | 先冻结真实规格，再复用同一 core 和运行链路 |
| CSV、绘图和状态 UI | 仅状态话题和单进程 MuJoCo 面板 | 后续增加低频诊断、日志和回放 |

## 4. 与 real_robot 的差距

ROS 2 顶层接口和三进程本机运行架构已经具备，尚未实现的实机侧能力包括：

- 串口、CAN 或厂商 SDK 对应的硬件 `RobotIO`；
- 电机零点、方向、减速比、扭矩换算和最终执行侧限幅；
- IMU 驱动、姿态估计和传感器健康状态；
- 急停、通信看门狗、失联阻尼、启动握手和故障锁存；
- 实机进程监督、部署配置、日志与回放；
- 里程计和真实环境所需的额外感知接口；
- blackW、black_with_arm 的模型、配置、策略和执行链路；
- 跨机器通信协议。

## 5. 当前明确边界

阶段 8 交付 Linux 本机 black 的模块化三进程入口、headless/GUI 链路、ROS 2 launch 兼容和 command 侧人工输入。以下内容仍属于后续阶段：

- GUI 长时间运行、窗口关闭和多显示环境的稳定性压力验证；
- 真实手柄设备的 SDL/pygame 现场验证；
- blackW 规格、模型、轮关节执行和 RL；
- 仿真故障注入、持久日志和回放；
- 真实硬件后端。

现有 `apps/mujoco_sim/` 仍是单进程本地调试入口，不等同于三进程 ROS 2 GUI。

## 6. 推荐补齐顺序

1. 阶段 7：已完成三进程 MuJoCo GUI 模式、ROS 2 launch 和统一启动参数；
2. 阶段 8：已完成三进程脚本模块化、构建 target、headless E2E 和 command 输入接线；
3. 阶段 9：勘察并冻结 blackW 关节、执行器、传感器和策略规格；
4. 阶段 10：完成 blackW MuJoCo RobotIO、腿轮混合控制和基础动作；
5. 阶段 11：在保持 black 回归的前提下配置化 RL 数据路径并接入 blackW 策略；
6. 阶段 12：增加故障注入、低频日志和回放基线；
7. 在仿真边界稳定后实现目标硬件 `RobotIO`、校准和安全监督。

这个顺序优先保持一条可运行链路，不把 `rl_sar` 和 `real_robot` 的历史包袱重新搬入仓库。
