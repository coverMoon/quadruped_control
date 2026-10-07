# 机器人模型与行为

black 和 blackW 共用 MotionRuntime、RobotIO 边界和行为实现。机器人差异由 RobotModel、控制器、策略和行为配置表达，不按关节数量推断模型。

## 1. 能力概览

| 项目 | black | blackW |
| --- | --- | --- |
| 结构 | 12 关节四足 | 16 关节轮足 |
| RL 策略 | flat、obstacle | flat、obstacle、stair |
| Retry | ✓ | ✓ |
| Event chain | 空事件列表 | 10 段事件链 |
| 固定姿态轮驱 | — | Bridge、Low-bar、Car |
| MuJoCo physics | 500 Hz | 500 Hz |
| MotionRuntime | 200 Hz | 200 Hz |
| TorchScript policy | 50 Hz | 50 Hz |

公共基础动作包括 Passive、GetUp、Stand 和 GetDown。RL locomotion、Retry、Event chain 和固定姿态轮驱都使用 `MotionMode::Running`，具体功能由 `behavior_name` 区分。

## 2. 公共模型约定

逻辑腿顺序固定为 FL、FR、RL、RR。RobotModel 明确记录机器人名称、有序关节名、`JointRole`、位置限制和命令限值；启动时会与策略、行为配置和 MuJoCo 模型交叉校验。

关节命令始终显式携带 `ControlMode`。KP/KD 只描述选定模式下的参数，不能用于推断控制模式。内部统一使用 rad、rad/s、N·m 和单调纳秒时间戳，四元数顺序为 `w,x,y,z`。

两个模型的 MuJoCo scene 均提供：

- `imu_quat`：`framequat`，内部顺序 `w,x,y,z`；
- `imu_gyro`：角速度，单位 rad/s；
- `imu_acc`：线加速度，单位 m/s²。

## 3. black

black 每条腿包含 hip、thigh、calf：

```text
FL_hip_joint, FL_thigh_joint, FL_calf_joint,
FR_hip_joint, FR_thigh_joint, FR_calf_joint,
RL_hip_joint, RL_thigh_joint, RL_calf_joint,
RR_hip_joint, RR_thigh_joint, RR_calf_joint
```

MuJoCo 入口：

```text
assets/robots/black/mujoco/scene.xml
assets/robots/black/mujoco/scene_terrain.xml
assets/robots/black/mujoco/scene_nwbt.xml
```

black 提供 flat 和 obstacle 两个策略。单帧观测为 45 维，使用 6 帧历史形成 270 维输入，输出 12 个腿关节位置残差。策略顺序和默认策略由 `configs/policies/black/policy_switch.yaml` 定义。

black 的 Event chain 配置保留公共行为入口，但事件列表为空，因此不会执行 blackW 的轮驱通过动作。

## 4. blackW

blackW 每条腿增加一个 wheel：

```text
FL_hip_joint, FL_thigh_joint, FL_calf_joint, FL_wheel_joint,
FR_hip_joint, FR_thigh_joint, FR_calf_joint, FR_wheel_joint,
RL_hip_joint, RL_thigh_joint, RL_calf_joint, RL_wheel_joint,
RR_hip_joint, RR_thigh_joint, RR_calf_joint, RR_wheel_joint
```

MuJoCo 入口：

```text
assets/robots/blackW/mujoco/scene.xml
assets/robots/blackW/mujoco/scene_terrain.xml
assets/robots/blackW/mujoco/scene_nwbt.xml
```

轮关节使用 `JointRole::Wheel`，不带位置限制。腿和轮都显式使用 `JointImpedance`：

- 腿关节：位置目标加 KP/KD；
- 轮关节：当前轮角、目标轮速、`KP=0` 和 KD。

blackW 提供 flat、obstacle 和 stair 三个策略。单帧观测为 57 维，使用 6 帧历史形成 342 维输入，输出 16 个动作：

- 腿动作是默认姿态的位置残差，scale 为 `0.25`；
- 轮动作是目标角速度，FL/FR/RL/RR scale 为 `+10/-10/+10/-10`。

策略顺序由 `configs/policies/blackW/policy_switch.yaml` 定义。策略 YAML 中的 `command_limits` 同时限制 MotionRuntime、键盘、导航和手柄输入。

### 女娲补天地图（nwbt）

`nwbt` 取“女娲补天”的拼音首字母。两个机器人的 `scene_nwbt.xml` 共用
`assets/maps/nwbt/map.xml` 和其中的 29 个场地 OBJ 网格。资源来自
`rc27/robocon_mujoco/dog_robocon.xml` 的 ROBOCON 场地，保留红蓝半场、高台、
斜坡、阶梯、围栏、柱体与小方块的视觉和碰撞设置。

场地相对来源坐标整体平移 `(4, -2, 0)` m，原红方安全出生点 `(-4, +2)`
对应本仓库世界原点。black 和 blackW 沿用各自模型的出生高度、零位、站姿 keyframe、
执行器和 IMU，以及 2 ms 物理步长；地图不引入来源示例的机器人或 ONNX 策略。
场地接触采用来源的三维接触与摩擦参数。
红蓝半场的地面碰撞体向下延伸至 `z=-1` m，顶面保持 `z=0`，
覆盖机器人复位零姿态中低于地面的足端，使其自然落地后被地面托起。

使用 `./scripts/run/backend.sh black nwbt` 或 `./scripts/run/backend.sh blackW nwbt` 启动，
可追加 `--mode headless`。motion 和 command 的启动方式见根目录 [README](../README.md#run-simulation)。

## 5. 策略运行与切换

RL 数据路径为：

```text
StateFrame
  → observation
  → 6 帧 history
  → TorchScript policy
  → action conversion
  → CommandFrame
```

运行 RL 时可用键盘 `0` 或手柄 `A` 请求 GetUp。MotionRuntime 会先停止策略并清理其
历史状态，再从当前实测关节姿态用固定增益插值回默认站姿，完成后进入 Stand。black
接近默认站姿时会跳过预起立收腿阶段；blackW 按其单段起立配置回到站姿。该路径与
`rl_sar` 的 `RL_Locomotion → GetUp` 语义一致，不会瞬时切换关节目标。

MotionRuntime 每 5 ms 生成一次 CommandFrame，策略每 4 个控制周期推理一次，其余周期复用最近动作。策略在启动阶段完成加载和预热，单次推理 deadline 为 20 ms。

策略列表由 `configs/policies/<robot>/policy_switch.yaml` 定义。切换策略时会重置目标策略的历史和旧动作；默认姿态差异较大时，先按 `posture_transition_cycles` 完成姿态过渡。

## 6. Retry

black 和 blackW 共用 Retry 实现，并分别从 `configs/behaviors/<robot>/retry.yaml` 加载目标姿态和增益：

```text
prepare → locked
```

进入 Retry 后会清零 RL 输出和速度目标，将腿移动到 `retry_default_joint_positions`，并将 blackW 轮速归零。到达 `locked` 后请求结果为 Completed，但运行状态仍保持 `Running / retry / locked`。

后续使用 GetUp 恢复 Stand，或使用 EnterPassive 放松。正式三进程可通过键盘 `5`、手柄 `LB+B` 或 ROS 2 `StartBehavior("retry")` 启动 Retry。

## 7. Event chain

Event chain 支持三种事件：

| 类型 | 内容 |
| --- | --- |
| `pose` | 姿态过渡与保持 |
| `drive` | 指定轮组按距离驱动 |
| `pose_drive` | 姿态与轮驱组合 |

blackW 配置包含 10 个事件，轮半径为 `0.103 m`，轮方向为 `[+,-,+,-]`。轮驱事件根据参与轮组的编码器位移计算行驶距离，结束后通过 `policy_transition` 返回当前 RL 策略。

black 的配置使用相同 schema，但当前事件列表为空。

## 8. blackW 固定姿态轮驱

配置位于：

```text
configs/behaviors/blackW/bridge_drive.yaml
configs/behaviors/blackW/low_bar_drive.yaml
configs/behaviors/blackW/car_drive.yaml
```

三种行为共享流程：

```text
当前姿态
  → 进入目标姿态
  → 差速轮驱
  → 轮速归零
  → 返回 RL 默认姿态
```

轮速使用 `10 × x ± 5 × yaw`，并按配置中的 `max_x` 和 `max_yaw` 限幅。左右轮分组和方向均由行为配置明确给出。

## 9. 验证范围

单元测试、MuJoCo 集成测试和 black/blackW 三进程 headless 测试覆盖模型加载、关节映射、基础动作、RL、策略切换、Retry 和主要故障退路。以下内容仍适合继续做场景或长时间验证：

- blackW Event chain 的完整墙体接触与通过性；
- Bridge、Low-bar、Car 对应的专用障碍场景；
- 不同实体手柄的连接、重连和轴映射；
- MuJoCo GUI 长时间运行及多显示器环境。

策略和模型来源记录在 `assets/policies/<robot>/SOURCE.md` 与 `assets/robots/<robot>/mujoco/SOURCE.md`，实际运行参数以本仓库配置和代码为准。
