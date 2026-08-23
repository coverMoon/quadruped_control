# blackW 规格记录

## 1. 结论与来源基线

阶段 9 已冻结 blackW 的 16 关节逻辑模型、MuJoCo 名称映射和三份 RL 策略契约。事实来源：

- `rl_sar@4abccaa60d4c2242466f82c8ac8ff83f4cefe9b3`：策略顺序、观测、历史、动作、
  控制周期、decimation、默认姿态、增益和轮动作换算；许可证为 Apache-2.0；
- `real_robot@4662151ab5c72e7fa28d4af2c66b5d6b0d7018d0`：当前 blackW MJCF、MuJoCo
  runner、实机轮命令路径和 IMU；
- `URDF@22c120bad450a81af2c4d6fcbf221262e2884928`：关节机械限位、速度和力矩限制；
- `URDF@fe699df481e4d71d3f9b474f2c77c5033e97b765`：当前导入网格的历史 MuJoCo
  资产版本。

同一字段冲突时采用明确分工：关节逻辑顺序和策略语义以 `rl_sar` 为准，MuJoCo 对象名称
和传动以当前 MJCF 为准，机械位置/速度/力矩限制以当前 URDF 为准。没有可靠来源的实机
参数不写成默认值。

## 2. RobotModel

机器人名称固定为 `blackW`，共有 16 个逻辑关节：

| 逻辑下标 | 关节 | 角色 | 有位置限制 |
|---:|---|---|---|
| 0–3 | FL hip、thigh、calf、wheel | Leg、Leg、Leg、Wheel | 是、是、是、否 |
| 4–7 | FR hip、thigh、calf、wheel | Leg、Leg、Leg、Wheel | 是、是、是、否 |
| 8–11 | RL hip、thigh、calf、wheel | Leg、Leg、Leg、Wheel | 是、是、是、否 |
| 12–15 | RR hip、thigh、calf、wheel | Leg、Leg、Leg、Wheel | 是、是、是、否 |

完整稳定名称为：

```text
FL_hip_joint, FL_thigh_joint, FL_calf_joint, FL_wheel_joint,
FR_hip_joint, FR_thigh_joint, FR_calf_joint, FR_wheel_joint,
RL_hip_joint, RL_thigh_joint, RL_calf_joint, RL_wheel_joint,
RR_hip_joint, RR_thigh_joint, RR_calf_joint, RR_wheel_joint
```

MJCF 的物理书写顺序是 FL、FR、RR、RL，不能用它替代上述逻辑顺序。现有 MuJoCo 后端
按关节名称和执行器传动建映射，因此无需复制参考 runner 的整数重排数组。

腿关节限制来自 `URDF/blackW/blackW_description.urdf`：hip 为 `[-0.5, 0.5] rad`、
速度上限 `30.1 rad/s`、力矩上限 `23.7 N·m`；左右 thigh/calf 使用镜像位置范围，
thigh 速度/力矩上限为 `30.1 rad/s` 和 `23.7 N·m`，calf 为 `12.04 rad/s` 和
`59.25 N·m`。轮关节连续旋转，速度上限 `50 rad/s`，力矩上限 `20 N·m`。

`RobotModel.max_kp/max_kd` 是本控制系统的命令接收包络，不声明为电机硬件极限。当前
包络覆盖参考配置实际使用的腿部 `KP ≤ 120`、`KD ≤ 6` 和轮部 `KP = 0`、`KD ≤ 6`。

## 3. MuJoCo 映射

模型入口为 `assets/robots/blackW/mujoco/scene.xml`，地形入口为同目录
`scene_terrain.xml`，二者包含 `black_description.xml`。模型有 16 个单自由度 hinge
joint 和 16 个直接关节传动 motor，物理步长为 `0.001 s`。

motor 没有独立的 `name` 属性。执行器映射因此冻结为“通过 `joint` 传动反查”，每个上述
逻辑关节恰好有一个 `gear=1` 的 motor；不得按 actuator 数组下标推断逻辑顺序。

IMU 固定在 site `imu`：

| 名称 | MuJoCo 类型 | 维度 | 内部语义 |
|---|---|---:|---|
| `imu_quat` | `framequat` | 4 | 四元数 `w,x,y,z` |
| `imu_gyro` | `gyro` | 3 | 角速度 `rad/s` |
| `imu_acc` | `accelerometer` | 3 | 线加速度 `m/s²` |

MJCF 为兼容历史仿真把腿关节 range 写成 `[-10, 10]`。运行时仍必须使用
`configs/robots/blackW.yaml` 中来自 URDF 的真实限制校验命令，不能把 MJCF 宽范围视为
机械能力。

## 4. 基础控制规格

- MotionRuntime 周期：`5 ms`（200 Hz）；
- MuJoCo 物理步长：`1 ms`；
- RL decimation：`4`，即策略推理周期 `20 ms`（50 Hz）；
- 起立：参考实现为 200 个控制周期的单段插值；
- 趴下：500 个控制周期；
- 腿：`JointImpedance`，基础姿态使用 `KP=80`、`KD=3`；
- 轮：`JointImpedance`，`KP=0`，目标位置保持当前轮角，目标速度承载轮命令，基础
  `KD=0.5`；不通过增益推断控制模式。

当前公共 ControllerConfig 是两段起立接口。blackW 配置把 200 周期单段插值放在第一段，
第二段以 1 周期确认同一姿态；这是现有接口的明确兼容映射，不代表 blackW 有第二个起立姿态。

## 5. RL 策略契约

阶段 9 冻结 `flat`、`obstacle`、`stair` 三份 TorchScript 模型。三者契约相同：

- 单帧观测 57 维；历史帧 `[0,1,2,3,4,5]`，推理输入为 `57 × 6 = 342`；
- 单帧顺序：`commands(3) + angular_velocity(3) + projected_gravity(3) +
  joint_position_error(16) + joint_velocity(16) + previous_action(16)`；
- 轮位置误差槽位保留但置零；
- 输出 16 维，顺序与 RobotModel 逻辑顺序一致；
- 腿输出是相对默认姿态的位置残差，缩放为 `0.25`；
- 轮输出是目标角速度，FL/FR/RL/RR 缩放分别为 `+10/-10/+10/-10`；
- 腿使用 `KP=50`、`KD=1.2`，轮使用 `KP=0`、`KD=1.0`。

已对仓库内三份模型做实际 TorchScript 调用：`[1,342]` 均输出 `[1,16]`，`[1,57]`
和 `[1,399]` 均被模型拒绝。阶段 11 已将 `RlController` 改为固定容量、启动期配置的
公共实现，现有 motiond 和单进程 MuJoCo 入口均可加载这些配置与模型。

## 6. 共同行为配置边界

`configs/behaviors/black/retry.yaml` 与 `blackW/retry.yaml` 使用相同字段：机器人名、
显式关节名顺序、`prepare_cycles`、Retry 姿态、KP 和 KD。blackW 轮槽固定 `KP=0`、
`KD=0.5`；公共 Retry 已按 `JointRole` 让轮速归零并保持当前轮角。

两份 `event_chain.yaml` 使用相同顶层接口和显式关节名顺序。black 继续以 `events: []`
表示未启用；blackW 已在轮足行为阶段 12 直接写入 `rl_sar` 的 10 个事件、`0.103 m`
轮半径和 `[+,-,+,-]` 方向符号。参考目标中的前腿大腿 `±3.14 rad` 超过本仓库显式
URDF 限位，因此执行配置收敛到 `±3.0 rad`，其余事件顺序、周期、距离和速度保持一致。

`car_drive.yaml`、`bridge_drive.yaml`、`low_bar_drive.yaml` 直接记录参考实现的 150 周期进入/
退出、`max_x=2.0`、`max_yaw=3.0`、`10*x ± 5*yaw` 差速换算和三组目标姿态。左右轮使用
显式 `[left,right,left,right]` 配置。Low-bar 前小腿的参考目标 `±2.52 rad` 超过 URDF
限位，执行配置收敛到 `±2.5 rad`。

## 7. 阻塞项

以下内容没有足够可靠的来源，进入对应实现前必须现场或由机械/电控资料确认：

1. 16 个逻辑关节到四路串口及真实 motor ID 的最终部署表；参考实机代码按每腿端口和
   局部 ID 组织，不能直接外推成全局 ID；
2. 四个轮在当前实机装配下的最终正方向。策略和仿真符号为 `[+,-,+,-]`，实机执行侧又
   对轮速度统一取负，必须悬空逐轮验证后才能冻结硬件符号；
3. 轮电机/轮端的真实机械减速比。当前实机代码对轮命令没有应用腿部的 `6.33` 或
   `6.33×2.5` 换算，这只能证明软件当前按 1:1 处理，不能证明机械减速比为 1；
4. `0.103 m` 轮半径及 Event chain 编码器位移换算在真实轮胎受载、打滑条件下的误差；
5. 当前 URDF/MJCF 零位与每台实机标定零位的对应关系；
6. 电机硬件允许的绝对 KP/KD 上限；当前 RobotModel 数值只是软件命令包络；
7. Car、Bridge、Low-bar 姿态对当前机械版本是否仍安全；
8. Event chain 在当前机械版本和 0.30 m 墙体场景中的接触成功条件，以及实机失败恢复。

这些阻塞项不影响 MuJoCo 中的事件调度、编码器位移和超时语义，但会阻止实机轮驱和完整
越墙动作被声明为已验收。
