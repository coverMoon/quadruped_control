# 当前能力与验证边界

本文只描述当前仓库已经存在的功能和仍需验证的仿真边界，不记录历史阶段或实机开发路线。

## 1. 当前结论

仓库已经具备 black 与 blackW 共用的 MuJoCo、TorchScript、MotionRuntime、共享内存 IPC 和
ROS 2 三进程仿真链路。black 和 blackW 的基础动作、RL 策略及运行时策略切换均已接入；
blackW 还支持 Retry、Event chain 和三种固定姿态轮驱行为。

正式运行链路：

```text
keyboard / joystick / ROS 2
             │
             ▼
        ros2_gateway
             │  BaseCommand / ModeRequest
             ▼
           motiond
             │  RobotIO: StateFrame / CommandFrame
             ▼
      mujoco_backendd
             │
             ▼
           MuJoCo
```

## 2. 已实现能力

### 2.1 公共核心

- C++17、无第三方依赖的 `core/`；
- 最大 16 关节的固定容量 `StateFrame`、`CommandFrame` 和诊断结构；
- 显式 `ControlMode`、`JointRole`、位置限制语义和统一 SI 单位；
- robot name、关节名顺序、startup、session、序号、时间和数值校验；
- `RobotIO` 作为 MotionRuntime 唯一状态读取和命令提交边界；
- Passive、GetUp、Stand、Running、GetDown 五个稳定 MotionMode；
- 带 Accepted、Running、Completed、Rejected、Failed 终态的请求生命周期。

### 2.2 black

- 12 个腿关节的 MuJoCo plain/terrain 场景；
- Passive、GetUp、Stand、GetDown；
- flat、obstacle 两份 TorchScript 策略；
- RL 运行中策略切换和默认姿态平滑过渡；
- 公共 Retry；
- Event chain 配置接口存在，但当前事件列表为空，不声明轮驱能力。

### 2.3 blackW

- 12 个腿关节加 4 个轮关节的 16 关节模型；
- 腿位置阻抗与轮速度目标阻抗混合命令；
- Passive、GetUp、Stand、GetDown 和公共 Retry；
- flat、obstacle、stair 三份 57×6 历史观测、16 维动作策略；
- `event_chain` 的 10 个姿态/轮驱事件；
- `bridge_drive`、`low_bar_drive`、`car_drive` 三种固定姿态差速轮行为；
- 行为完成、超时、打断和返回当前 RL 策略的终态语义。

### 2.4 运行与调试

- `ros2_gateway`、`motiond`、`mujoco_backendd` 三进程运行；
- MuJoCo backend 的 GUI/headless 模式；
- 键盘、规范化手柄和 ROS 2 `/cmd_vel` 输入；
- 策略 `command_limits` 对 MotionRuntime、键盘、导航和手柄统一限幅；
- GUI reset、Load Key、暂停、相机聚焦和鼠标扰动力；
- latest slot、SPSC 请求队列、heartbeat、session reset 和旧帧拒绝；
- 状态/命令诊断 CSV、故障注入和只读 ReplayRobotIO。

## 3. 当前验证入口

| 范围 | 命令 |
| --- | --- |
| RL、MuJoCo、公共测试 | `./scripts/build.sh --rl` |
| ROS 2 接口与 gateway | `./scripts/build.sh --target command` |
| 当前 profile 的 CTest | `./scripts/test/ctest.sh` |
| 三进程 headless 端到端 | `./scripts/test/ros2_headless.sh` |

测试覆盖核心校验、状态机、请求终态、策略张量与切换、black/blackW 基础动作、Retry、固定
姿态轮驱、Event chain、IPC 会话与锁竞争、MuJoCo 命令执行以及回放确定性。

## 4. 尚需仿真验收的内容

以下内容已有代码或配置基础，但还不能仅凭当前自动测试声明为完整场景能力：

- MuJoCo GUI 长时间运行、频繁窗口交互和多显示环境稳定性；
- 不同实体手柄的长期连接、断开重连和现场轴映射；
- blackW Event chain 在完整 0.30 m 墙体场景中的接触与整体通过性；
- Bridge、Low-bar、Car 对应专用障碍尺寸和接触场景的端到端验收；
- 长时间状态日志自动采集、性能统计和绘图工具；
- 三进程 backend 参数与 `configs/simulation/*.yaml` 的进一步统一。

这些项目是当前仿真质量边界，不影响已有单元和集成测试所覆盖的行为语义。

## 5. 参考工程边界

- `../rl_sar`：策略观测、动作、历史和行为语义参考，基线 `4abccaa`，Apache-2.0；
- `../real_robot`：MuJoCo 场景与历史交互行为参考，基线 `4662151`；
- `../URDF`：机器人几何和关节限制来源，基线
  `22c120bad450a81af2c4d6fcbf221262e2884928`。

参考工程不决定本仓库的模块边界。当前代码、配置、测试和本文档中的已实现语义优先。
