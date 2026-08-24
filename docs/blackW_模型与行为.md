# blackW 模型与行为

blackW 是 16 关节轮足机器人，在公共 MotionRuntime 上增加轮关节、轮足 RL 策略、Event chain 和固定姿态轮驱行为。

## 1. 关节顺序

每条腿依次为 hip、thigh、calf、wheel：

```text
FL_hip_joint, FL_thigh_joint, FL_calf_joint, FL_wheel_joint,
FR_hip_joint, FR_thigh_joint, FR_calf_joint, FR_wheel_joint,
RL_hip_joint, RL_thigh_joint, RL_calf_joint, RL_wheel_joint,
RR_hip_joint, RR_thigh_joint, RR_calf_joint, RR_wheel_joint
```

逻辑顺序由 RobotModel 和各配置文件中的 `joint_names` 统一描述。MuJoCo backend 按关节名和 actuator transmission 建立映射。

轮关节配置：

```yaml
role: wheel
position_limited: false
```

腿关节带位置限制，轮关节连续旋转。软件命令范围位于 `configs/robots/blackW.yaml`。

## 2. MuJoCo 模型

入口：

```text
assets/robots/blackW/mujoco/scene.xml
assets/robots/blackW/mujoco/scene_terrain.xml
```

模型包含 16 个单自由度关节和对应执行器，physics timestep 为 2 ms。

IMU：

| 传感器 | MuJoCo 类型 | 内部格式 |
| --- | --- | --- |
| `imu_quat` | `framequat` | `w,x,y,z` |
| `imu_gyro` | `gyro` | rad/s |
| `imu_acc` | `accelerometer` | m/s² |

## 3. 基础控制

| 项目 | 值 |
| --- | ---: |
| MotionRuntime | 200 Hz |
| CommandFrame 有效期 | 10 ms |
| MuJoCo physics | 500 Hz |
| RL decimation | 4 |
| RL policy | 50 Hz |
| GetUp | 200 cycles |
| GetDown | 500 cycles |

腿和轮都使用 `JointImpedance`：

- 腿：位置目标 + KP/KD；
- 轮：当前轮角 + 目标轮速，`KP=0`，使用 KD。

`ControlMode` 表示控制模式，KP/KD 只表示该模式下的增益。

## 4. RL 策略

blackW 提供：

- `flat`
- `obstacle`
- `stair`

切换顺序由 `configs/policies/blackW/policy_switch.yaml` 定义。

三份策略共享相同张量结构：

| 项目 | 规格 |
| --- | --- |
| 单帧观测 | 57 |
| 历史帧 | 6 |
| 输入 | 342 |
| 动作 | 16 |

观测：

```text
commands(3)
+ angular_velocity(3)
+ projected_gravity(3)
+ joint_position_error(16)
+ joint_velocity(16)
+ previous_action(16)
```

动作顺序与 RobotModel 一致：

- 腿：默认姿态位置残差，scale `0.25`；
- 轮：目标角速度，FL/FR/RL/RR scale 为 `+10/-10/+10/-10`；
- 腿：`KP=50, KD=1.2`；
- 轮：`KP=0, KD=1.0`。

策略 YAML 中的 `command_limits` 同时用于 MotionRuntime、键盘、导航和手柄输入。

## 5. 行为

| 行为 | black | blackW |
| --- | --- | --- |
| `rl_locomotion` | ✓ | ✓ |
| `retry` | ✓ | ✓ |
| `event_chain` | 空配置 | ✓ |
| `bridge_drive` | — | ✓ |
| `low_bar_drive` | — | ✓ |
| `car_drive` | — | ✓ |

这些行为都运行在 `MotionMode::Running` 下，通过：

```text
behavior_name
behavior_phase
policy_name
```

描述具体状态。

## 6. Retry

Retry 从当前姿态插值到恢复姿态：

```text
prepare → locked
```

进入 Retry 后：

- RL 输出和速度目标清零；
- 腿移动到 `retry_default_joint_positions`；
- 轮速归零；
- 保持恢复姿态，直到收到新的模式请求。

black 和 blackW 共用同一实现。

## 7. Event chain

Event chain 支持：

| 类型 | 内容 |
| --- | --- |
| `pose` | 姿态过渡与保持 |
| `drive` | 指定轮组按距离驱动 |
| `pose_drive` | 姿态与轮驱组合 |

blackW 配置包含 10 个事件，轮半径为 `0.103 m`，轮方向为 `[+,-,+,-]`。

轮驱事件根据参与轮组的编码器位移计算行驶距离。事件完成后通过 `policy_transition` 返回当前 RL 策略。

完整墙体接触与整体通过性仍建议在专用场景中继续验证。

## 8. 固定姿态轮驱

配置：

```text
configs/behaviors/blackW/car_drive.yaml
configs/behaviors/blackW/bridge_drive.yaml
configs/behaviors/blackW/low_bar_drive.yaml
```

三种行为共享流程：

```text
当前姿态
  → 进入目标姿态
  → 差速轮驱
  → 轮速归零
  → 返回 RL 默认姿态
```

轮速使用：

```text
10 × x ± 5 × yaw
```

并按 `max_x=2.0`、`max_yaw=3.0` 限幅。左右轮分组和方向由配置给出。

## 9. 参考来源

- `rl_sar`：策略观测、动作和轮足行为；
- `real_robot`：MuJoCo 场景与历史参数；
- `URDF`：几何和关节限制。

本仓库运行时参数以 `configs/`、模型文件和代码为准。
