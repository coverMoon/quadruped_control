# 运行时与 IPC

正式仿真将外部接口、运动控制和物理后端拆成三个进程：

```text
ROS 2 / keyboard / joystick
            │
            ▼
       ros2_gateway
            │
            ▼
          motiond
            │
            ▼
     mujoco_backendd
            │
            ▼
          MuJoCo
```

black 和 blackW 共享这套拓扑，通过启动参数选择机器人、控制器、策略和行为配置。

## 1. 进程职责

### ros2_gateway

- `/cmd_vel`、action 和 service；
- 键盘与规范化手柄输入；
- MotionStatus、RobotIOStatus、StateDiagnostic、ModeResult 发布；
- ROS 2 与 IPC 的数据转换。

### motiond

- 加载 RobotModel、ControllerConfig、行为和策略配置；
- 周期运行 MotionRuntime；
- 读取 StateFrame；
- 提交 CommandFrame；
- 消费 BaseCommand 和 ModeRequest；
- 发布 MotionStatus 和 ModeResult。

### mujoco_backendd

- 创建共享内存；
- 加载 MuJoCo scene 与 `MujocoRobotIO`；
- 运行物理线程；
- 执行 CommandFrame；
- 发布 StateFrame、RobotIOStatus 和 heartbeat；
- 处理 reset 与 pause 请求。

## 2. 共享内存数据面

IPC 使用 POSIX `shm_open` / `mmap`。

| 数据 | 通道 | 生产者 | 消费者 |
| --- | --- | --- | --- |
| StateFrame | latest slot | backend | motion / gateway |
| CommandFrame | latest slot | motion | backend |
| BaseCommand | latest slot | gateway | motion |
| MotionStatus | latest slot | motion | gateway |
| RobotIOStatus | latest slot | backend | gateway |
| heartbeat | latest slot | 各进程 | 对端 |
| ModeRequest | SPSC queue | gateway | motion |
| ModeResult | SPSC queue | motion | gateway |
| backend control | SPSC queue | control | backend |

高频状态与命令使用 latest-value 语义，一次性请求和结果使用 FIFO 队列。

latest slot 只保留最近发布值，消费者可以跳过中间版本；SPSC queue 保留尚未消费的一次性消息顺序。两者都使用固定容量 wire 类型，高频路径不传递 `std::string`、虚函数对象或外部框架类型。

## 3. futex 通知面

三个缓存行对齐的进程共享 futex event 只负责提示“对应数据面可能发生变化”：

| event | 唤醒对象 | 对应数据 |
| --- | --- | --- |
| motion_event | motiond | StateFrame、BaseCommand、ModeRequest |
| backend_event | backend | CommandFrame、reset / pause 请求 |
| gateway_event | gateway | ModeResult |

生产者通过 `publish_latest_and_notify()` 或 `queue_push_and_notify()` 固定执行“先发布、后通知”。queue 已满时写入失败，不递增 event。event sequence 使用 release/acquire 顺序，保证等待者观察到序列变化后可以读取已经发布的数据。

通知允许合并，event sequence 不是数据版本或队列长度。消费者使用以下模式避免在读取数据和准备休眠之间丢失唤醒：

```text
snapshot = event.sequence
读取 latest slot / 清空当前 queue
if event.sequence == snapshot:
    FUTEX_WAIT(event.sequence, snapshot, timeout)
```

通知发生在记录快照前、读取数据期间或进入 futex 前，都不会导致消费者在已有新数据时持续休眠。

等待结果：

| 结果 | 含义 |
| --- | --- |
| `Changed` | event 已变化或等待者被通知唤醒 |
| `TimedOut` | 相对单调时钟 timeout 到期 |
| `Interrupted` | 信号中断等待，用于正常停机检查 |
| `Failed` | futex 出现意外系统错误，进程结束当前运行链路 |

timeout 和信号中断是正常状态，不抛异常，也不在高频路径输出重复日志。

### motiond

StateFrame、BaseCommand 或 ModeRequest 发布后通知 `motion_event`。motiond 被状态更新唤醒，但仍根据 StateFrame 时间戳和 5 ms `control_period_ns` 判断 `control_due`，因此 500 Hz 或 1 kHz 状态发布不会改变 200 Hz MotionRuntime 周期。没有事件时最多等待一个控制周期，以继续 heartbeat、断线和 fault 检查。

### mujoco_backendd

CommandFrame 或 backend control request 发布后通知 `backend_event`。物理线程可以在 deadline 前读取控制请求并缓存最新命令，但不得因此提前调用 `RobotIO::submit()` 或 `step()`；到达固定 deadline 后会再次读取最新版本，再提交命令、执行 MuJoCo 步进并发布 StateFrame。

错过 deadline 时丢弃墙钟欠账，不连续快速补步。管理级 reset 清除 pending command，并把已观察 command version 推进到 reset 时的当前版本；普通姿态 reset 保持 session 和待执行命令语义。

### ros2_gateway

ModeResult 入队后通知 `gateway_event`。result thread 醒来后清空当前结果队列，并以 50 ms timeout 检查退出。gateway 析构时主动通知该 event，保证线程及时结束。低频状态 timer 和临时 IPC control 工具的结果轮询不参与高频控制路径。

### 生命周期

共享内存 owner 关闭时先将 `ready` 置为 0，再唤醒三个 event，最后解除映射。等待者醒来后检查 owner、session 和 heartbeat 状态，并进入既有断线退路。

已验证的三进程共享内存基线为 wire schema v2、公共 frame schema v1。
工作树中未提交的 v3/v2 Unit 0 尝试尚未通过三进程 headless 测试，不能作为已验收版本。
正式升级时必须停止所有旧进程、同步重建三个二进制，
再按正常顺序启动；共享内存尺寸或 schema 不匹配时进程会明确拒绝连接。

## 4. Session

backend 启动时建立控制 session。

普通姿态 reset：

- 加载 MuJoCo keyframe；
- 保持当前 session；
- 保持当前运动行为。

管理级 reset：

```bash
ros2 run quadruped_gateway quadruped_ipc_control \
    /quadruped_control_black reset
```

管理级 reset 创建新的 session。motiond 检测到 session 变化后重置旧请求、速度输入和控制调度状态。

## 5. 周期与超时

| 项目 | 默认值 |
| --- | ---: |
| MuJoCo physics | 500 Hz |
| MotionRuntime | 200 Hz |
| CommandFrame 有效期 | 10 ms |
| BaseCommand 有效期 | 200 ms |
| backend / motion heartbeat | 500 ms |
| gateway 状态发布 | 20 Hz |
| action / service 等待结果 | 10 s |

BaseCommand 与 CommandFrame 的超时分别作用于机体速度目标和关节命令。
CommandFrame 同时记录帧有效期和关节目标的硬过期时间。RL 默认每 20 ms 更新目标，
中间控制周期复用目标时不刷新目标生成时间，目标硬有效期为 40 ms。
执行截止时间取帧有效期与目标硬过期时间的较早值；普通姿态命令每周期生成新目标。
设计依据见 [Black real backend v1 计划](real_migration/BLACK_REAL_BACKEND_V1_PLAN.md)。
MuJoCo 的状态与命令校验使用仿真时间；heartbeat 使用 host monotonic 存活计时。
RemoteRobotIO 从后端最新状态读取控制时钟，命令时间戳和有效期沿用该时间域。
共享状态槽忙时保守复用最近采样时间，不用 host monotonic 外推仿真时间。

event timeout 只用于推进 MotionRuntime 周期、更新 heartbeat 和检查停机状态，
本身不表示 IPC 断开。连接状态仍由 owner ready 状态、session 与 heartbeat 判定。

latest slot 的瞬时锁竞争会复用同一 backend session 内最近的有效状态；真正的连接状态仍由 heartbeat 判断。

## 6. 验证采样

2026-08-24 在 black、headless、500 Hz physics、200 Hz MotionRuntime 和
`real_time_factor=1.0` 下，以独立只读进程观察 wire version，连续采样 300 个命令：

| 路径 | mean | p50 | p99 |
| --- | ---: | ---: | ---: |
| StateFrame 可见到对应 CommandFrame 可见 | 49.6 μs | 49.2 μs | 132.1 μs |
| CommandFrame 可见到 effective state 可见 | 1.95 ms | 1.95 ms | 2.08 ms |

第二项包含等待下一个 2 ms 物理 deadline、`submit()`、MuJoCo 步进和状态发布。
观察器自身的调度与读槽时间也计入结果，因此这些数据用于本机回归比较，不是实时上界。
同次 `ps` 快照中，backend 为 62.0% CPU / 731516 KiB RSS，motiond 为
14.5% CPU / 143760 KiB RSS；数值包含 MuJoCo 和 Torch 模型资源。

## 7. 进程退出与故障

| 情况 | 结果 |
| --- | --- |
| gateway 停止 | BaseCommand 超时，速度来源回到 None |
| motiond 停止 | CommandFrame 过期，backend 不再沿用旧主动命令 |
| backend 停止 | heartbeat 超时，RemoteRobotIO 进入 Disconnected |
| session 改变 | 旧 session 的命令、请求和结果失效 |
| wire / frame 校验失败 | 丢弃对应数据 |

MotionRuntime 的详细错误处理见 [系统架构](architecture.md)。

## 8. 启动

三个终端依次运行：

```bash
./scripts/run/backend.sh black terrain
./scripts/run/motion.sh black
./scripts/run/command.sh black keyboard
```

手柄：

```bash
./scripts/run/command.sh black joystick
```

blackW：

```bash
./scripts/run/backend.sh blackW terrain
./scripts/run/motion.sh blackW
./scripts/run/command.sh blackW keyboard
```

backend 支持 `plain|terrain|dog26|nwbt|dog27`、`gui|headless` 等运行参数。详细命令见根目录 [README](../README.md)。

## 9. 测试

```bash
./scripts/build.sh
./scripts/test/ctest.sh
./scripts/test/ros2_headless.sh
```

`black_simulation.launch.py` 主要用于 CI 与自动化测试，人工运行使用 `scripts/run/` 下的三个入口。
