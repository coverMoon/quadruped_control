# blackW 模型、策略与行为

本文记录 blackW 在当前仓库中的有效技术契约。配置文件和代码是运行时事实来源；本文用于
解释关节顺序、策略张量、腿轮混合命令以及各行为的共同语义，不再记录已经完成的实施阶段。

## 1. 事实来源

- `rl_sar@4abccaa60d4c2242466f82c8ac8ff83f4cefe9b3`：策略观测、历史、动作顺序和
  轮足行为参考，Apache-2.0；
- `real_robot@4662151ab5c72e7fa28d4af2c66b5d6b0d7018d0`：MuJoCo 对象和历史运行参数参考；
- `URDF@22c120bad450a81af2c4d6fcbf221262e2884928`：关节位置、速度和力矩限制参考；
- `configs/`、`assets/robots/blackW/` 和当前测试：本仓库实际执行契约。

冲突字段按职责选取：策略语义以 `rl_sar` 为参考，MuJoCo 名称以当前 MJCF 为准，命令限制
以 `configs/robots/blackW.yaml` 为准。参考工程中的数组下标不能直接替代本仓库的显式名称映射。

## 2. RobotModel 与关节顺序

blackW 有 16 个逻辑关节，每条腿依次为 hip、thigh、calf、wheel：

```text
FL_hip_joint, FL_thigh_joint, FL_calf_joint, FL_wheel_joint,
FR_hip_joint, FR_thigh_joint, FR_calf_joint, FR_wheel_joint,
RL_hip_joint, RL_thigh_joint, RL_calf_joint, RL_wheel_joint,
RR_hip_joint, RR_thigh_joint, RR_calf_joint, RR_wheel_joint
```

这个顺序由 RobotModel、ControllerConfig、行为配置和策略 YAML 共同显式声明。MJCF 的物理
书写顺序不是逻辑顺序，后端按关节名和 actuator 传动建立映射。

轮关节固定使用：

```yaml
role: wheel
position_limited: false
```

不能根据关节名称、MJCF joint 类型或很大的位置范围推断轮子。腿关节有位置限制，轮关节
连续旋转。当前软件命令包络位于 `configs/robots/blackW.yaml`；其中 KP/KD 是本控制系统允许
接收的范围，不表示电机物理极限。

## 3. MuJoCo 映射

模型入口：

```text
assets/robots/blackW/mujoco/scene.xml
assets/robots/blackW/mujoco/scene_terrain.xml
```

两者包含 `black_description.xml`。模型具有 16 个单自由度关节和 16 个直接关节传动 motor，
物理步长为 `2 ms`。motor 没有独立名称，`MujocoRobotIO` 通过 `joint` 传动反查执行器，不能
使用 actuator 数组下标推断逻辑顺序。

IMU 固定在 site `imu`：

| 传感器 | MuJoCo 类型 | 内部语义 |
| --- | --- | --- |
| `imu_quat` | `framequat` | 四元数 `w,x,y,z` |
| `imu_gyro` | `gyro` | 角速度，rad/s |
| `imu_acc` | `accelerometer` | 线加速度，m/s² |

MJCF 为仿真兼容保留了较宽的腿关节 range；运行时仍按 RobotModel 限制校验命令。

## 4. 基础控制

| 项目 | 当前值 |
| --- | ---: |
| MotionRuntime 周期 | 5 ms / 200 Hz |
| CommandFrame 有效期 | 10 ms |
| MuJoCo 物理步长 | 2 ms / 500 Hz |
| RL decimation | 4 |
| RL 推理周期 | 20 ms / 50 Hz |
| GetUp | 200 个控制周期，随后 1 周期确认 |
| GetDown | 500 个控制周期 |

腿和轮都显式使用 `JointImpedance`：

- 腿：位置目标、`target_velocity=0`，使用非零 KP/KD；
- 轮：保持当前轮角、`target_velocity=目标轮速`、`KP=0`，使用 KD；
- 控制模式由 `ControlMode` 明确给出，不能根据 KP 是否为零推断。

## 5. RL 策略契约

blackW 当前注册 `flat`、`obstacle`、`stair` 三份 TorchScript 策略，切换顺序由
`configs/policies/blackW/policy_switch.yaml` 决定。

三份策略使用相同张量契约：

- 单帧观测：57 维；
- 历史帧：`[0,1,2,3,4,5]`；
- 推理输入：`57 × 6 = 342` 维；
- 动作：16 维，与 RobotModel 逻辑顺序一致；
- 观测顺序：`commands(3) + angular_velocity(3) + projected_gravity(3) +
  joint_position_error(16) + joint_velocity(16) + previous_action(16)`；
- 轮位置误差槽位保留但置零；
- 腿动作是默认姿态的位置残差，缩放为 `0.25`；
- 轮动作是目标角速度，FL/FR/RL/RR 缩放为 `+10/-10/+10/-10`；
- 腿使用 `KP=50`、`KD=1.2`，轮使用 `KP=0`、`KD=1.0`。

策略 YAML 的 `command_limits` 同时约束 MotionRuntime、键盘、导航和手柄。手柄归一化轴
`[-1,1]` 会按当前策略的限制缩放到完整速度量程，策略切换后同步更新。

策略切换时，如果目标默认姿态与当前姿态差异较大，MotionRuntime 先执行固定周期的姿态
过渡，再清空目标策略的观测历史和旧动作。过渡或首次推理失败会终止活动请求并进入 Passive。

## 6. 行为分类

行为通过 `ModeRequest.behavior_name` 选择，运行时统一归入 `MotionMode::Running`：

| 行为 | black | blackW | 含义 |
| --- | --- | --- | --- |
| `rl_locomotion` | 支持 | 支持 | RL 策略行走 |
| `retry` | 支持 | 支持 | 公共恢复姿态 |
| `event_chain` | 空配置 | 支持 | 顺序执行姿态和轮驱事件 |
| `bridge_drive` | 不支持 | 支持 | 桥面固定姿态差速轮驱动 |
| `low_bar_drive` | 不支持 | 支持 | 低矮通道固定姿态差速轮驱动 |
| `car_drive` | 不支持 | 支持 | 低姿态车辆式差速轮驱动 |

具体状态由以下字段表达，不为单个行为增加新的 `MotionMode`：

```text
MotionStatus.behavior_name
MotionStatus.behavior_phase
MotionStatus.policy_name
```

`policy_switch.yaml` 只包含 RL 策略，不放入 Retry、Event chain 或固定姿态轮驱行为。

## 7. Retry

Retry 从当前姿态平滑插值到配置的恢复姿态并持续保持：

```text
当前行为 → prepare → locked
```

执行规则：

- 清除 RL 输出、速度命令和待处理的策略切换；
- 腿关节插值到 `retry_default_joint_positions`；
- 轮关节保持当前轮角、目标速度归零、`KP=0`；
- 不自动退出；
- GetUp 和 EnterPassive 可以打断；
- 状态或命令提交失败时产生明确的 Failed 结果并进入 Passive。

black 与 blackW 使用相同实现和 YAML 字段，差异来自显式关节顺序及 `JointRole`。

## 8. Event chain

Event chain 配置支持三种事件：

| 类型 | 必需内容 |
| --- | --- |
| `pose` | `transition_cycles`、`dof_pos`，可选 `hold_cycles` |
| `drive` | `wheel_group`、距离、速度、超时，可选保持周期 |
| `pose_drive` | 同时具有姿态过渡和轮驱字段 |

姿态插值使用 smoothstep。轮驱事件根据显式 wheel group 和方向符号计算轮速，用参与轮组的
平均编码器位移判断距离，超时则 Failed。当前事件名直接作为 `behavior_phase`；退出阶段为
`policy_transition`，完成后返回当前 RL 策略。

black 的 `event_chain.yaml` 使用空事件列表，因此不会伪造轮组能力。blackW 配置包含 10 个
直接对照 `rl_sar` 的事件，轮半径为 `0.103 m`，轮方向为 `[+,-,+,-]`。参考姿态中超过
RobotModel 限位的目标已经收敛到允许范围。

事件调度、编码器距离、超时和返回 RL 已由测试覆盖；完整 0.30 m 墙体接触链仍需要在专用
MuJoCo 场景中做整体通过性验证。

## 9. 固定姿态差速轮行为

Car、Bridge 和 Low-bar 共用一个实现，配置分别位于：

```text
configs/behaviors/blackW/car_drive.yaml
configs/behaviors/blackW/bridge_drive.yaml
configs/behaviors/blackW/low_bar_drive.yaml
```

共同生命周期：

```text
当前姿态 → 150 周期进入目标姿态 → 差速轮驱动
差速轮驱动 → 轮速归零 → 150 周期返回当前 RL 默认姿态
```

轮速换算使用 `10 × x ± 5 × yaw`，并按 `max_x=2.0`、`max_yaw=3.0` 限幅。左右轮和方向
均由配置显式给出，不依赖关节下标。

## 10. 输入映射

| 键盘 | 手柄 | 行为 |
| --- | --- | --- |
| `0` | `A` | GetUp |
| `1` | `RB + DPadUp` | RL locomotion / 返回 RL |
| `2` | `RB + DPadRight` | Bridge drive |
| `3` | `RB + DPadDown` | Low-bar drive |
| `4` | `RB + DPadLeft` | Car drive |
| `5` | — | Retry（仅单进程调试入口） |
| `6` | `LB + DPadUp` | Event chain |
| `9` | `B` | GetDown |
| `P` | `LB + X` | EnterPassive |

正式三进程 command 与单进程调试入口存在少量按键差异，以根目录 README 的运行说明为准。

## 11. 测试关注点

- 12/16 关节名称、顺序、角色和位置限制校验；
- 腿轮混合 CommandFrame 的位置、速度、KP 和 KD；
- 三份策略的 342 维输入、16 维输出和切换历史清零；
- black/blackW 共用 Retry 生命周期；
- black 对轮驱 Event chain 的明确拒绝；
- Event chain 距离、正负方向、超时、中断和返回 RL；
- 三种固定姿态轮驱行为的进入、限幅、差速和退出；
- black 的 RL、基础动作和三进程链路保持回归通过。
