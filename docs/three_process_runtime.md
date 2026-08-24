# 三进程运行架构

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

## 2. IPC

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

wire 数据结构使用固定容量字段，不跨进程复制 `std::string`、虚函数对象等 C++ 运行时对象。core 与 wire 的转换位于 `adapters/ipc/`。

## 3. Session

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

## 4. 周期与超时

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

latest slot 的瞬时锁竞争会复用同一 backend session 内最近的有效状态；真正的连接状态仍由 heartbeat 判断。

## 5. 进程退出与故障

| 情况 | 结果 |
| --- | --- |
| gateway 停止 | BaseCommand 超时，速度来源回到 None |
| motiond 停止 | CommandFrame 过期，backend 停止沿用旧主动命令 |
| backend 停止 | heartbeat 超时，RemoteRobotIO 进入 Disconnected |
| session 改变 | 旧 session 的命令、请求和结果失效 |
| wire / frame 校验失败 | 丢弃对应数据 |

MotionRuntime 的详细错误处理见 [系统架构](quadruped_control_architecture.md)。

## 6. 启动

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

backend 支持 `plain|terrain`、`gui|headless` 等运行参数。详细命令见根目录 [README](../README.md)。

## 7. 测试

```bash
./scripts/build.sh
./scripts/test/ctest.sh
./scripts/test/ros2_headless.sh
```

`black_simulation.launch.py` 主要用于 CI 与自动化测试，人工运行使用 `scripts/run/` 下的三个入口。
