# REAL_HARDWARE_BASELINE

> 核查源码：本机 `../real_robot`，origin 为 `git@github.com:coverMoon/real_robot.git`
> 源码版本：`4662151ab5c72e7fa28d4af2c66b5d6b0d7018d0`（核查时工作区干净）
> 适用平台：`black`、`blackW`  
> 文档定位：记录旧实机代码中已经存在的硬件连接、通信参数、执行器映射、换算关系、IMU、校准、安全与时序行为。  
> 下述旧行为来自源码静态核查，不代表已经验证当前硬件。新后端约定见
> [实机后端迁移架构参考](REAL_ROBOT_BACKEND_MIGRATION_REFERENCE.md)。

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

本地 `unitreeMotor/unitreeMotor.h` 中只声明：

```cpp
MotorType::GO_M8010_6
```

仓库带 ARM64 与 x86-64 动态库，两种机器人对应架构的库 SHA-256 相同：

| 架构 | SHA-256 |
|---|---|
| ARM64 | `c2a23b6adf68f90f1fc459c7bb0708d4f39d6330f4b80f19c130baf03f45d93f` |
| x86-64 | `7b53d6ffc779860e753fa8a90d35af55cd30a0964226419e17c82025b726559f` |

本地只有 `SerialPort` 声明，未找到其 `.cpp` 实现；动态符号确认 `sendRecv`
及反馈解码位于 SDK 库。不能从函数名或返回 bool 推断整组响应全部有效、超时总预算、
自动重连或电机断联停机行为。这些需要供应商源码/资料或台架验证。

SDK 头文件声明 `mode=0` 为刹车、`1` 为 FOC、`2` 为电机标定。旧运行路径使用
`mode=1`，零力矩查询不等于 SDK `mode=0`。以上为头文件语义，尚未验证固件行为。

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

初始化完成后的零位修正如下，`motor_offset` 是第 7.2 节的完整 `O`：

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

- SDK 头文件将 `Pos/W/T` 标为电机转子侧量；
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

## 7.2 完整零位公式

依据两套 `src/utils/set_zero.cpp::get_motor_offset()` 和
`include/real_runner/utils/set_zero.h::get_offset_()`，对腿关节索引
`i = leg * 3 + joint`：

```text
P0 = 本次启动读取的转子位置（rad）
C  = creep_position[i]（保存的转子位置，rad）
S  = straight_position[i]（保存的转子位置，rad）
R  = -round((P0 - C) / (2π)) × 2π
A  = 46.66 × π/180 × 6.33 × 2.5
B  = -A（i=2,8），+A（i=5,11），0（其他关节）
O  = R - S + B
```

`round` 为 C++ `std::round`，半整数向远离零方向取整。
`R` 是本次启动整圈补偿；`O` 才是用于命令和反馈的完整 offset。
不能省略 `-S`，也不能把本次启动的 `R` 保存为型号固定零位。

设方向 `d=+1`（hip/thigh）、`d=-1`（calf），传动比 `r` 见第 5 节：

```text
motor.Pos = d × r × joint.q - O
motor.W   = d × r × joint.dq
motor.T   = d × joint.tau / r
motor.K_P = joint.kp / r²
motor.K_W = joint.kd / r²

joint.q   = d × (motor.Pos + O) / r
joint.dq  = d × motor.W / r
joint.tau = d × motor.T × r
```

这些公式描述旧代码数值关系；机械效率和 SDK 增益的固件解释尚未实测。
blackW 的校准索引仍是 12 个腿关节，不能使用含轮子的 `leg*4+joint` 直接索引校准数组。

## 7.3 校准成立条件

整圈选择依赖上电姿态与保存的 creep 参考关系，源码没有验证机械姿态是否满足这一前提。
任意姿态上电是否会选错圈需要实机核对。`46.66°` 的机械来源及正负号也需核对安装。

旧构造函数未因校准读取失败而停止启动：缺失文件保留零初始化值，格式错误可能留下
部分读入值。新后端应在校准完整性检查失败时拒绝主动控制，不能迁移该默认退路。

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

工具每轮顺序执行四路事务后再 sleep 2 ms，实际周期是事务与处理耗时加 2 ms，
不能按严格 500 Hz 描述。`record_position()` 在事务失败后仍会继续复制数据，采集
straight/creep 时未统一检查每电机有效性；新工具应只接受完整有效样本。

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

`not_first_command` 在构造时为 false；收到第一条 ROS 命令后，
`_commandCallback()` 将它设为 true，运行期间未见复位。因此首条外部命令也可能触发检查，
并不是“第一条命令执行成功后才检查”。blackW 先校验数组长度，black 没有相应长度检查。
两者均未在该回调校验 NaN/Inf 或命令有效期。

触发位置跳变分支时，函数直接 return，此前可能已写入部分命令缓冲；保护默认关闭时
`setIsSafe(false)` 不会建立 unsafe 状态。这不是可复用的完整帧安全提交实现。

---

# 14. 失衡安全保护

实际判定在 `real_runner.cpp::_processImuSafety()`，比较的是绝对值：

| 型号 | roll / pitch 阈值 |
|---|---|
| black | `abs(angle) > π/6`，30° |
| blackW | `abs(angle) > π/3`，60° |

两套 `utils/secure_protect.hpp` 都另有 30° 常量，但上述调用使用的是
`real_runner.cpp` 的全局常量。不能据头文件认定 blackW 使用 30°。

若启用保护，首次 unsafe 后 `_processImuSafety()` 会提前返回，姿态恢复不会自动解除；
默认关闭保护时则不会形成这个锁存效果。

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

> **当前 Black 补充确认：**实体 AB5465 安装与配置延续已验证的 legacy 路径；
> `diag(1,-1,-1)` 是 Black v1 的 sensor→body 变换，不再把安装方向列为 blocker。
> 原始 acceleration 物理单位、时间戳质量、启动收敛、振动和 bias 仍待 bring-up。
> 本节其余描述仅反映旧源码；blackW 实体安装不由这项 Black 确认覆盖。

---

# 20. VQF 与最终姿态来源

两套 `_handleImuSample()` 先按第 19 节的 Euler 符号构造 raw quaternion。
默认 `imu_vqf_enabled=true`；当 gyro/accel 各分量有限且加速度模长大于 `1e-6` 时，
转换后的 gyro、accel 输入 VQF，`getQuat6D()` 的 `w,x,y,z` 覆盖 raw quaternion。
最终结果同时写入 `/_lowState/imu` 和聚合 joint state 内的 IMU，姿态保护也使用该结果。

```text
tauAcc = 3.0
gyrTs = accTs = 0.002 s
```

每个成功解析的帧更新一次 VQF，不按接收时间差调整 dt；串口批量到达时仍逐帧更新。
禁用 VQF 或该帧不满足输入检查时保留 raw quaternion，仍发布消息并喂 IMU watchdog。
CRC 正确不等于数值有效；旧代码没有对最终整份 IMU 做统一有限值和单位四元数检查。

接收错误后重开串口，未见清空 parser 残留或重新初始化 VQF 的处理。
实际采样率、加速度原始单位、安装坐标和重连后的估计恢复仍需验证。

# 21. IMU watchdog、命令失联与退出

两套源码中的 IMU 参数相同：

| 参数 | 实际值与含义 |
|---|---|
| 稳定间隔 | 相邻处理时间间隔 `0 < gap <= 20 ms` |
| 稳定计数 | 累积 250 个连续合格间隔后启用监控 |
| 超时 | 距最后处理帧 `> 0.5 s` |
| 确认 | exchange loop 连续 3 次超时检查 |
| 恢复 | 新帧清空 strike，设置 alive；不重新关闭已启用的监控 |

确认超时后调用 `setIsSafe(false)`；默认保护关闭时该调用仍保持 safe，日志中
“触发阻尼”不能证明实际已输出阻尼。喂狗时间使用 ROS 节点 `this->now()`，不是传感器
采样时间，也不是新后端所需的独立单调时间。

| 情况 | 旧运行路径实际处理 |
|---|---|
| 未收到或未稳定的 IMU | 提示等待，不因此阻止电机命令 |
| ROS 命令停止更新 | 持续复用 `_lowCmd`，无命令年龄/heartbeat 检查 |
| 电机初始化 `sendRecv=false` | 零增益查询，等待约 10 ms 后重试 |
| 正常电机通信 `sendRecv=false` | 记录错误；触发 safety 的语句被注释，仍拷贝 localState |
| 正常电机 `correct=false` / ID 不符 | SerialPack 未统一逐电机拒绝；SDK bool 与部分反馈语义待验证 |
| `MError!=0` | blackW 仅额外记录 leg 0 第三个电机错误变化，未形成全电机故障覆盖 |
| 正常退出 | RealRunner 停止并 join IMU/exchange；SerialPack 停止并 join 总线线程 |

应用析构和循环退出路径未显式发送最终零增益或阻尼帧；SDK 析构和电机固件的断联行为
尚不能由当前可见源码确认。新后端需要明确发送侧超时、逐电机故障检查及退出动作，
不能依赖旧 ROS 输入或进程析构完成安全停机。

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
| 姿态阈值（保护默认关闭） | 30° | 60° |
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
black 30° / blackW 60° roll/pitch threshold（旧值，非新默认值）
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

# 26. 源码证据与剩余确认项

源码根为 `../real_robot/<black或blackW>/src/real_robot/real_runner/`。
版本见文首，以下符号用于复核；它们不是新后端接口：

| 核查内容 | 源文件与符号 |
|---|---|
| 总线、ID、换算、线程、失联 | `include/real_runner/utils/serial_packages.hpp`：构造、`sendRecv`、`_sendRecvMotorGroup`、析构 |
| 标定读写与整圈计算 | `src/utils/set_zero.cpp`：`load_calibration_file`、`get_motor_offset`、`record_position` |
| 完整 offset | `include/real_runner/utils/set_zero.h`：`get_offset_` |
| SDK 单位、模式及声明限值 | `include/real_runner/unitreeMotor/unitreeMotor.h` |
| 串口默认参数与超时存储 | `include/real_runner/serialPort/SerialPort.h`、`IOPort/IOPort.h` |
| IMU、watchdog、命令及退出 | `src/real_runner.cpp`：`_handleImuSample`、`_processImuSafety`、`_exchangeLoop`、`_commandCallback`、析构 |
| 保护开关 | `include/real_runner/utils/secure_protect.hpp`、`src/utils/secure_protect.cpp` |

以下仍需硬件/SDK资料或台架验证，不能仅靠旧程序常量确认：

1. 四路设备节点、每个 bus 的实际 motor ID，尤其 blackW wheel；
2. SDK 固件单位与增益定义、传动比和机械方向；wheel 未缩放是否代表输出轴量；
3. calf `2.5`、`46.66°` 的机械依据与每腿安装关系；
4. 当前机器校准文件、允许的上电姿态、整圈选择及掉电后的编码器行为；
5. wheel 连续角、跨圈、方向和新模型轴向；旧固定零轮角不能支持现有 Event chain 行程计算；
6. IMU 原始加速度单位、实际帧率、时间戳质量及重连后的 VQF 恢复；
7. SDK 整组事务中逐电机的成功条件、响应校验、实际超时和重连行为；
8. 电机固件断联停机、通信彻底中断/进程被杀后的行为，以及硬件停机通道；
9. 总线实际周期与抖动，旧 20 ms / 5 ms timeout 是否适合当前设备；
10. 新后端采用的姿态阈值和阻尼参数。旧 black calf 两层阻尼换算不一致，不能同时复现。

第 7、13、20、21 节已经确认完整零位、首命令检查时机、最终姿态来源和 watchdog
参数；不再把这些列为等待源码核查的事项。

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
