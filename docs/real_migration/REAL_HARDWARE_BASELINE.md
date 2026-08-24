# REAL_HARDWARE_BASELINE

> 参考仓库：`N-W-wolf/real_robot_black-W`  
> 适用平台：`black`、`blackW`  
> 文档定位：记录旧实机代码中已经存在的硬件连接、通信参数、执行器映射、换算关系、IMU、校准、安全与时序行为。  
> 本文用于迁移核对，不负责定义新架构。无法仅凭当前代码完全确认的内容统一标记为 **待确认**。

---

## 1. 基线范围

旧仓库包含两套基本独立的 ROS 2 workspace：

```text
black/
blackW/
```

两套工程中的底层实机代码高度相似，均包含：

```text
real_robot/real_runner
├── serialPort
├── IOPort
├── unitreeMotor
├── imu/vqf
├── utils/serial_packages
├── utils/set_zero
└── utils/secure_protect
```

其中 Unitree Motor SDK、串口实现、VQF 等代码在 black 与 blackW 中基本重复。

实际型号差异主要集中在：

- actuator 数量；
- `serial_packages.hpp`；
- `real_runner.cpp`；
- calibration 文件；
- blackW wheel 相关代码和测试工具。

---

# 2. 机器人执行器布局

## 2.1 black

black 使用 12 个关节执行器。

逻辑顺序：

```text
FL: hip thigh calf
FR: hip thigh calf
RL: hip thigh calf
RR: hip thigh calf
```

旧代码按连续索引处理：

| Leg | hip | thigh | calf |
|---|---:|---:|---:|
| leg 0 | 0 | 1 | 2 |
| leg 1 | 3 | 4 | 5 |
| leg 2 | 6 | 7 | 8 |
| leg 3 | 9 | 10 | 11 |

`SerialPack` 中：

```cpp
legNum   = 4;
jointNum = 3;
```

每条腿独占一条串口。

---

## 2.2 blackW

blackW 使用 16 个执行器：

```text
FL: hip thigh calf wheel
FR: hip thigh calf wheel
RL: hip thigh calf wheel
RR: hip thigh calf wheel
```

ROS / 上层逻辑索引：

| Leg | hip | thigh | calf | wheel |
|---|---:|---:|---:|---:|
| leg 0 | 0 | 1 | 2 | 3 |
| leg 1 | 4 | 5 | 6 | 7 |
| leg 2 | 8 | 9 | 10 | 11 |
| leg 3 | 12 | 13 | 14 | 15 |

旧代码使用：

```cpp
rosIndex(leg, joint) = leg * 4 + joint
```

但实际 motor ID 使用：

```cpp
motorId(leg, joint) = leg * 3 + joint
```

因此 blackW 当前各腿串口上的电机 ID 为：

| Bus | hip | thigh | calf | wheel |
|---|---:|---:|---:|---:|
| `/dev/leg_0` | 0 | 1 | 2 | 3 |
| `/dev/leg_1` | 3 | 4 | 5 | 6 |
| `/dev/leg_2` | 6 | 7 | 8 | 9 |
| `/dev/leg_3` | 9 | 10 | 11 | 12 |

由于四条腿位于不同串口总线上，跨 bus 出现相同 motor ID 不构成当前实现中的冲突。

> **待确认：** 上表应在实机迁移前与实际电机配置再次核对，尤其是 blackW wheel 的 ID。

---

# 3. 电机串口

## 3.1 设备节点

black 和 blackW 均默认使用：

```text
/dev/leg_0
/dev/leg_1
/dev/leg_2
/dev/leg_3
```

一条腿一条总线。

---

## 3.2 SerialPort 默认参数

`SerialPort` 默认构造参数：

```text
baudrate       = 4,000,000
recv length    = 16 bytes
timeout        = 20,000 us
data bits      = 8
parity         = none
stop bits      = 1
flow control   = none
blocking       = no
```

black 的 `SerialPack` 直接使用默认构造：

```cpp
SerialPort(portName)
```

因此 nominal 参数为：

```text
4 Mbps
20 ms timeout
```

blackW 显式构造：

```cpp
SerialPort(portName, 16, 4000000, 5000)
```

因此 blackW 当前使用：

```text
4 Mbps
5 ms timeout
```

这是 black / blackW 旧实现中的一个实际差异。

---

# 4. Unitree Motor SDK

当前 SDK 中只声明：

```cpp
MotorType::GO_M8010_6
```

支持 ARM64 与 x86-64 动态库。

SDK 对命令字段的定义：

| Field | SDK 语义 | 单位 |
|---|---|---|
| `Pos` | 电机转子位置 | rad |
| `W` | 电机转子速度 | rad/s |
| `T` | 电机转子转矩 | N·m |
| `K_P` | 电机侧位置刚度系数 | SDK 定义 |
| `K_W` | 电机侧速度系数 | SDK 定义 |

状态字段：

| Field | SDK 语义 |
|---|---|
| `Pos` | 电机转子位置 |
| `W` | 电机转子速度 |
| `T` | 电机转子转矩 |
| `Temp` | 温度 |
| `MError` | 错误码 |
| `correct` | 当前返回数据是否有效 |

SDK 中 `MError` 注释：

```text
0 正常
1 过热
2 过流
3 过压
4 编码器故障
```

---

# 5. 腿部传动关系

旧 `SerialPack` 使用：

```cpp
gear_ratio = {
    6.33,
    6.33,
    6.33 * 2.5
};
```

即：

| Joint | ratio |
|---|---:|
| hip | 6.33 |
| thigh | 6.33 |
| calf | 15.825 |

对应刚度缩放：

```cpp
gear_ratio_squared = {
    6.33²,
    6.33²,
    (6.33 × 2.5)²
};
```

---

## 5.1 逻辑关节命令 → 电机命令

对 hip / thigh：

```text
motor.Pos = joint.q  × ratio
motor.W   = joint.dq × ratio
motor.T   = joint.tau / ratio

motor.K_P = joint.kp / ratio²
motor.K_W = joint.kd / ratio²
```

calf 首先改变符号：

```text
q   = -q
dq  = -dq
tau = -tau
```

随后再执行同样的 ratio 换算。

因此 calf 的实际命令关系为：

```text
motor.Pos = -joint.q   × 15.825
motor.W   = -joint.dq  × 15.825
motor.T   = -joint.tau / 15.825
```

零位修正存在时：

```text
target_motor_pos -= motor_offset
```

---

## 5.2 电机状态 → 逻辑关节状态

hip / thigh：

```text
joint.q   = corrected_motor_pos / ratio
joint.dq  = motor.W / ratio
joint.tau = motor.T × ratio
```

calf：

```text
joint.q   = -corrected_motor_pos / 15.825
joint.dq  = -motor.W / 15.825
joint.tau = -motor.T × 15.825
```

其中：

```text
corrected_motor_pos = raw_motor_pos + motor_offset
```

---

## 5.3 关于 6.33 与 2.5

当前代码能够确认：

- SDK 的 `Pos/W/T` 是电机转子侧量；
- `6.33` 被用于电机侧与关节侧转换；
- calf 在此基础上额外乘 `2.5`；
- calf 方向取反。

> **待确认：**
>
> 1. `6.33` 是否完全对应 GO-M8010-6 内部减速器机械传动比；
> 2. `2.5` 所对应的 calf 外部机构具体结构；
> 3. torque 与 stiffness 的这套理想传动换算是否需要考虑效率或其他历史补偿。
>
> 新实现迁移初期应先保持现有数学关系。

---

# 6. blackW Wheel

blackW 每条腿增加第 4 个 actuator。

wheel 在旧代码中采用速度控制为主：

```text
K_P = 0
T   = 0
Pos = 0
W   = -logical_wheel_dq
K_W = logical_wheel_kd
```

轮子状态：

```text
logical wheel q       = 0
logical wheel dq      = -motor.W
logical wheel tau_est = -motor.T
```

因此当前代码并没有为 wheel 累积或输出连续转角。

wheel 不使用腿部：

```text
6.33
6.33 × 2.5
```

传动换算。

旧测试说明中对“正转”的定义：

```text
从机器人内侧看：逆时针
从机器人外侧看：顺时针
```

> **待确认：**
>
> wheel 的负号应结合新模型中的 wheel joint axis 与 MuJoCo 正方向再次核对。

---

# 7. 电机零位与启动 offset

旧实机使用 `motor_zero` 处理电机转子多圈位置与机械零位之间的关系。

black 与 blackW 都只对 12 个腿部关节进行该校准：

```cpp
leg_num   = 4
joint_num = 3
motor_num = 12
```

blackW 的四个 wheel 不进入这套 calibration。

---

## 7.1 calibration 文件格式

路径：

```text
./src/real_robot/real_runner/motor_calibration.conf
```

文件包含两行：

```text
line 1: straight_position[12]
line 2: creep_position[12]
```

当前格式没有 joint name，完全依赖固定数组顺序。

---

## 7.2 启动时多圈 offset

初始化阶段：

1. 向本腿电机发送零增益、零力矩命令；
2. 读取当前 `MotorData.Pos`；
3. 与保存的 `creep_position` 比较；
4. 计算相差的完整 `2π` 圈数；
5. 生成当前启动对应的 rotor offset。

代码关系：

```text
diff   = current_motor_pos - creep_position
rounds = round(diff / 2π)
offset = -rounds × 2π
```

之后：

```text
command target position -= offset
state raw position       += offset
```

这使当前电机多圈编码值重新落到 calibration 所定义的机械位置附近。

---

## 7.3 calf calibration 特殊修正

`get_offset_()` 中还存在 calf 的固定补偿：

```text
offset_calf = 46.66° × 6.33 × 2.5
```

转换为 rad 后参与 rotor-side offset。

全局 motor index：

```text
2, 8  -> subtract offset_calf
5, 11 -> add offset_calf
```

也就是四个 calf 的补偿方向并不全部相同。

> **待确认：**
>
> 该 `46.66°` 的机械来源以及四腿正负号与实际连杆安装的对应关系。

---

# 8. calibration 工具行为

旧 `set_zero` 工具启动四路：

```text
/dev/leg_0
/dev/leg_1
/dev/leg_2
/dev/leg_3
```

并持续发送：

```text
mode = 1
K_P  = 0
K_W  = 0
Pos  = 0
W    = 0
T    = 0
```

主要交互：

```text
s : 记录 straight position
c : 记录 creep position
w : 写入 calibration file
m : 输出当前内存数据
q : 退出
```

循环中存在：

```text
sleep 2 ms
```

因此工具循环 nominal 为 500 Hz，但仍受实际串口 transaction 时间影响。

---

# 9. 当前 calibration 数据

## 9.1 black

straight:

```text
7.38268 8.8294 29.147629
-1.0458 -1.71699 -27.57437
-1.822951 10.11404 32.6769
4.47119 -0.94973 -29.775975
```

creep:

```text
4.69687 1.87472 0.652519
1.84001 5.33769 1.92074
0.603239 3.53468 3.02502
1.845 5.52963 0.471125
```

---

## 9.2 blackW

straight:

```text
0.89938 9.10196 36.581921
5.26167 -2.01807 -29.83154
4.520558 10.14655 34.26747
4.57626 -1.0016 -29.682472
```

creep:

```text
1.10543 3.80217 5.75091
4.98794 3.34351 1.09948
4.29707 5.49953 3.33277
4.42938 3.44552 0.752228
```

这些数据属于具体实机 calibration，不应直接视为机器人型号的通用机械常量。

---

# 10. 电机通信线程

`SerialPack` 为四条腿各创建一个独立线程。

每个线程：

```text
leg 0 thread → /dev/leg_0
leg 1 thread → /dev/leg_1
leg 2 thread → /dev/leg_2
leg 3 thread → /dev/leg_3
```

目标周期：

```cpp
motor_loop_period = 2 ms
```

nominal frequency：

```text
500 Hz
```

典型循环：

```text
copy latest command
        │
        ▼
hardware safety check
        │
        ▼
SerialPort::sendRecv()
        │
        ▼
update latest MotorData
        │
        ▼
sleep_until(next 2 ms boundary)
```

command buffer 与 state buffer 分别使用 mutex。

需要注意：

```text
2 ms loop period ≠ 已经证明总线稳定运行在严格 500 Hz
```

因为 `sendRecv()` 本身包含实际串口 transaction 和 timeout。

---

# 11. RealRunner 主交换周期

`RealRunner` 中：

```cpp
dt_ = 0.005;
```

对应：

```text
200 Hz
```

旧源码附近存在“1000 Hz”注释，但与实际 `0.005 s` 不一致，应以代码值为准。

因此旧系统实际上形成：

```text
4 × motor I/O thread
      nominal 500 Hz
            │
            ▼
    shared latest state
            │
            ▼
RealRunner exchange loop
          200 Hz
            │
            ▼
        ROS topics
```

---

# 12. 初始化阶段电机行为

每条 motor bus 在 offset 尚未初始化时：

```text
K_P = 0
K_W = 0
T   = 0
Pos = 0
W   = 0
```

只利用发送/接收 transaction 获取当前电机状态。

读取成功后：

```text
MotorData
    │
    ▼
motor_zero.get_motor_offset()
    │
    ▼
is_offset_initialized = true
```

初始化失败则等待后重试。

blackW 还会额外验证 wheel：

- wheel response 是否存在；
- `correct` 是否为 true；
- 返回 `motor_id` 是否与期望一致。

---

# 13. 电机位置跳变保护

正常控制阶段，旧 `SerialPack` 会比较：

```text
current motor Pos
target motor Pos
```

若：

```text
abs(current - target) > 8π
```

同时：

```text
not_first_command == true
K_P != 0
```

则将全局 safety state 置为 unsafe，并提前退出当前 command staging。

该保护发生在 motor-side position 空间。

> `not_first_command` 的完整生命周期需要在迁移前再次核对，确认其具体何时被置为 true。

---

# 14. 失衡安全保护

旧代码使用全局 `SafetyStateManager`。

阈值：

```text
roll  > 30°
pitch > 30°
```

即：

```cpp
π / 6
```

超出任一阈值时，将 safety state 标记为 unsafe。

---

## 14.1 unsafe 时腿部指令

在 command staging 中：

```text
mode = 1
K_P  = 0
K_W  = 6.0 / ratio²
T    = 0
Pos  = 0
W    = 0
```

即电机侧阻尼控制。

blackW 的 wheel：

```text
K_W = 6.0
```

---

## 14.2 motor thread 内的第二层检查

Motor I/O thread 在真正发送前还会再次检查：

```cpp
utils::safeok()
```

unsafe 时再次覆盖 local command 为阻尼模式。

这说明旧实现已经具备：

```text
上层 command staging safety
+
发送线程 safety override
```

两层检查。

需要注意，black 发送线程中使用统一：

```text
K_W = 6 / 6.33²
```

而 command staging 对 calf 使用：

```text
6 / (6.33 × 2.5)²
```

两处对 calf 的阻尼换算并不完全一致。

> **待确认：** 这是有意设计还是历史遗留不一致。

---

# 15. SafetyStateManager 当前状态

`SafetyStateManager`：

```text
is_safe             : atomic bool
protection_enabled  : atomic bool
```

默认：

```text
is_safe = true
protection_enabled = true
```

但当前 `RealRunner` 中定义：

```cpp
ENABLE_LATCHED_SAFETY_PROTECTION = false;
```

构造时会调用：

```cpp
setProtectionEnabled(false)
```

按照 `safeok()` 的实现，protection disabled 时直接返回 safe。

因此：

> **当前代码默认实际上关闭了这套 latched safety protection。**

这是迁移前必须明确确认的行为，不能只根据存在 `secure_protect` 代码就认为旧实机当前一定启用了姿态保护。

---

# 16. IMU 硬件接口

默认参数：

```text
device   = /dev/IMU_Link
baudrate = 460800
```

ROS 参数允许覆盖：

```text
imu_port
imu_baudrate
imu_vqf_enabled
imu_vqf_tau_acc
imu_vqf_dt
```

默认：

```text
imu_vqf_enabled = true
imu_vqf_tau_acc = 3.0
imu_vqf_dt      = 0.002 s
```

即 VQF nominal sample period：

```text
2 ms = 500 Hz
```

---

# 17. IMU 串口配置

当前 IMU 串口使用：

```text
8 data bits
no parity
1 stop bit
raw mode
non-blocking read
```

代码还显式启用了：

```text
CRTSCTS
```

即硬件流控（如果平台定义该宏）。

`VMIN = 0`，`VTIME = 1`。

打开失败时：

```text
wait 500 ms
retry
```

读取发生错误后：

```text
close
wait 200 ms
reopen
```

---

# 18. AB5465 数据帧

当前 `RealRunner` 直接实现一套 AB5465 parser。

固定帧长度：

```text
66 bytes
```

帧头：

```text
AB 54 65 00
```

CRC：

```text
CRC16-CCITT-FALSE
init = 0xFFFF
poly = 0x1021
```

帧尾最后两个 byte 为 little-endian CRC。

主要字段 offset：

| Data | Offset | Type |
|---|---:|---|
| roll | 11 | float32 LE |
| pitch | 15 | float32 LE |
| yaw | 19 | float32 LE |
| gyro x | 23 | float32 LE |
| gyro y | 27 | float32 LE |
| gyro z | 31 | float32 LE |
| accel x | 35 | float32 LE |
| accel y | 39 | float32 LE |
| accel z | 43 | float32 LE |

串口输入采用缓存流式解析：

```text
append bytes
     │
search header
     │
wait until 66 bytes
     │
CRC check
     │
decode sample
```

接收缓存超过 4096 bytes 时会裁剪旧数据。

---

# 19. IMU 坐标与单位转换

原始 Euler angle 被解释为 degree。

构造 raw orientation 时：

```text
roll  = +sample.roll
pitch = -sample.pitch
yaw   = -sample.yaw
```

随后 degree → rad 后构造 quaternion。

陀螺仪：

```text
x = +gyro_x × π/180
y = -gyro_y × π/180
z = -gyro_z × π/180
```

即原始 gyro 被认为是：

```text
degree / second
```

输出转换为：

```text
rad / second
```

加速度同样存在：

```text
x 保持正号
y 取反
z 取反
```

的坐标变换。

> **待确认：**
>
> 1. 原始 acceleration 是否已经是 m/s²；
> 2. 这些符号变换准确对应怎样的 IMU 安装姿态；
> 3. body frame 的 x/y/z 定义；
> 4. 新架构与 MuJoCo 中 body IMU frame 是否完全一致。

---

# 20. VQF

当前默认启用 VQF。

配置：

```text
tauAcc = 3.0
gyrTs  = 0.002 s
accTs  = 0.002 s
```

因此实现假定 gyro / accel nominal 为 500 Hz。

旧链路同时保留了 IMU 原始 Euler angle，可以构造 raw quaternion；VQF 则通过 gyro + accel 生成后续姿态估计。

迁移初期应记录并对比：

```text
raw sensor Euler
raw converted quaternion
VQF quaternion
gyro
acceleration
```

以确认实际发布的 orientation 来源与旧系统一致。

---

# 21. IMU watchdog

当前代码包含 IMU：

```text
last receive time
alive state
stable-frame count
timeout strike count
timeout monitoring enable
```

逻辑上先等待数据流达到稳定状态，再启用 timeout 监控，避免启动阶段立即触发故障。

IMU 数据恢复时会清除 timeout strike。

> **待确认：**
>
> 当前 timeout 阈值、连续 strike 数量以及 timeout 最终是否真正触发 motor safety，需要在迁移前继续核对 `real_runner.cpp` 后半部分。

---

# 22. 旧 ROS 2 实机接口

RealRunner：

订阅：

```text
/_lowCmd/command
```

发布：

```text
/_lowState/joint
/_lowState/imu
```

`midware` 再负责转发到更上层的 controller topic。

旧 `LowLevelCmd` 本身只是：

```text
robot_msgs::msg::RobotCommand
```

的轻量 wrapper，并按照 motor count resize command array。

因此这些 ROS topic 是旧软件组织方式，不属于硬件本身的通信协议。

---

# 23. black 与 blackW 差异摘要

| 项目 | black | blackW |
|---|---|---|
| logical actuators | 12 | 16 |
| actuators per leg | 3 | 4 |
| wheel | 无 | 有 |
| leg calibration count | 12 | 12 |
| wheel calibration | 无 | 不使用腿部 calibration |
| motor serial baud | 4 Mbps | 4 Mbps |
| SerialPort timeout | 默认 20 ms | 显式 5 ms |
| leg reduction | 6.33 / 6.33 / 15.825 | 相同 |
| calf sign | negative | negative |
| wheel command | — | velocity, negative sign |
| wheel q state | — | 固定 0 |
| wheel dq state | — | `-motor.W` |
| wheel torque state | — | `-motor.T` |
| motor thread target | 2 ms | 2 ms |
| main exchange | 5 ms | 5 ms |
| IMU | AB5465 + VQF | 同类链路 |

---

# 24. 当前旧实机数据流

```text
             /_lowCmd/command
                    │
                    ▼
                RealRunner
                    │
              LowLevelCmd
                    │
                    ▼
                SerialPack
          ┌─────┬─────┬─────┬─────┐
          ▼     ▼     ▼     ▼
       leg_0  leg_1  leg_2  leg_3
          │     │     │     │
          └─────┴─────┴─────┴─────┘
                    │
                MotorData
                    │
                    ▼
              LowLevelState
                    │
                    ▼
           /_lowState/joint


/dev/IMU_Link
      │
      ▼
serial stream
      │
      ▼
AB5465 parser
      │
      ├── Euler
      ├── Gyro
      └── Acc
            │
            ▼
           VQF
            │
            ▼
      /_lowState/imu
```

---

# 25. 迁移时必须保持的旧实机语义

在新 backend 第一次实机运行前，应逐项验证：

### Motor

```text
leg ordering
joint ordering
motor IDs
serial bus mapping
4 Mbps serial
zero initialization
rotor multi-turn compensation
6.33 reduction
calf ×2.5 reduction
calf direction
q conversion
dq conversion
tau conversion
kp conversion
kd conversion
wheel sign
wheel velocity semantics
```

### IMU

```text
/dev/IMU_Link
460800 baud
AB5465 66-byte frame
CRC16
Euler sign conversion
gyro sign/unit conversion
acc sign/unit conversion
VQF parameters
quaternion convention
body-frame convention
```

### Safety

```text
30° roll/pitch threshold
8π motor-position mismatch
damping command
startup zero-gain behavior
motor communication failure behavior
IMU timeout behavior
current protection_enabled state
shutdown behavior
```

### Timing

```text
motor I/O target: 2 ms
RealRunner exchange: 5 ms
VQF nominal dt: 2 ms
actual bus period
actual IMU frame rate
actual jitter
```

---

# 26. 待确认项

以下信息不应在迁移代码中依靠猜测：

1. black / blackW 当前实机上每个 bus 的实际 motor ID；
2. `6.33` 在机械与 SDK 语义中的精确定义；
3. calf `2.5` 外部传动的机械来源；
4. calf `46.66°` calibration 修正的机械来源；
5. 四个 calf offset 正负号与实际腿安装关系；
6. wheel joint axis 与旧代码负号之间的对应关系；
7. acceleration 原始单位；
8. IMU 的物理安装方向；
9. VQF 最终 orientation 与 raw Euler orientation 的实际使用路径；
10. IMU timeout 阈值与故障动作；
11. motor `correct=false` / `MError!=0` 在当前 runtime 中的最终处理；
12. `not_first_command` 的设置时机；
13. 发送线程与上层 staging 中 calf damping ratio 不一致的原因；
14. black 当前 20 ms SerialPort timeout 是否为有意配置；
15. blackW 5 ms timeout 是否经过稳定实机验证。

这些内容应在实现新的 `RealRobotIO` 前，通过旧代码继续核查或通过静态实机测试确认。

---

# 27. 基线结论

旧实机系统的核心硬件行为可以概括为：

```text
4 independent motor serial buses
            +
GO-M8010-6 rotor-side SDK
            +
per-joint transmission/sign mapping
            +
startup multi-turn calibration
            +
AB5465 IMU
            +
VQF attitude estimation
            +
shared safety state
```

black 与 blackW 的腿部控制链路基本一致。

blackW 的主要新增硬件语义是：

```text
每腿增加一个 wheel actuator
+
wheel velocity control
+
wheel 独立方向定义
```

迁移时真正需要保留的是这些已经形成实机行为的硬件语义，而旧 ROS topic、workspace 复制方式和节点组织仅作为历史实现参考。
