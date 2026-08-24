# ROS 2 三进程仿真运行架构

## 1. 范围

当前正式仿真把控制闭环拆为三个独立进程，并支持 backend headless/GUI 两种运行模式：

```text
ROS 2 topic/action/service
            │
            ▼
       ros2_gateway
            │  BaseCommand / ModeRequest / status
            ▼
          motiond
            │  RobotIO: StateFrame / CommandFrame
            ▼
     mujoco_backendd
            │
            ▼
     MuJoCo GUI/headless
```

black 和 blackW 使用同一进程拓扑，通过启动参数选择各自模型、控制器、策略和行为配置。
GUI 使用独立显示 model/data 副本；物理状态、reset 和鼠标扰动力由 backend 统一应用到
authoritative mjData。普通姿态 reset 保持当前 session，管理级 reset 才建立新 session。

## 2. 进程职责

### 2.1 `ros2_gateway`

- 订阅 `/cmd_vel`，生成最新 `BaseCommand`；
- 提供 GetUp、GetDown、StartBehavior、SwitchPolicy action；
- 提供 EnterPassive、ResetFault service；
- 发布 MotionStatus、RobotIOStatus、StateDiagnostic 和 ModeResult；
- ROS callback 只转换和写入 IPC，不调用 `MotionRuntime::update()`；
- 不依赖 MuJoCo，不生成或提交 `CommandFrame`。

### 2.2 `motiond`

- 加载所选机器人的 RobotModel、ControllerConfig、行为配置，以及
  `configs/policies/<robot>/policy_switch.yaml` 中列出的策略；
- 创建并周期调用 `MotionRuntime`；
- 通过 `RemoteRobotIO` 读取 StateFrame、提交 CommandFrame；
- 消费 BaseCommand 和 ModeRequest，发布 MotionStatus 和 ModeResult；
- 不依赖 ROS 2 或 MuJoCo API。

### 2.3 `mujoco_backendd`

- 创建共享内存并成为唯一 owner；
- 加载所选机器人的 MuJoCo scene 和 `MujocoRobotIO`；
- 按 MJCF physics timestep 在独立物理线程中步进；
- 消费最新 CommandFrame，发布 StateFrame、RobotIOStatus 和 heartbeat；
- 处理保持 session 的姿态 reset，以及显式递增 session 的管理级 reset；
- 不依赖 ROS 2 或 Torch。

## 3. IPC wire schema

IPC 使用 POSIX `shm_open` 和 `mmap`。共享内存只保存固定容量、显式定义的 wire 结构，
不会把 `std::string`、虚函数对象或其他 C++ core 对象直接跨进程复制。wire 转换入口统一位于
`adapters/ipc/`。

| 数据 | 通道 | 生产者 | 消费者 | 语义 |
| --- | --- | --- | --- | --- |
| StateFrame | latest slot | backend | motion、gateway | 新值覆盖旧值 |
| CommandFrame | latest slot | motion | backend | 新值覆盖旧值 |
| BaseCommand | latest slot | gateway | motion | 新值覆盖旧值 |
| MotionStatus | latest slot | motion | gateway | 低频状态快照 |
| RobotIOStatus | latest slot | backend | gateway | 后端状态快照 |
| 三类 heartbeat | latest slot | 各自进程 | 对端 | 进程在线和会话检查 |
| ModeRequest | 固定容量 SPSC | gateway | motion | FIFO，请求不可覆盖 |
| ModeResult | 固定容量 SPSC | motion | gateway | FIFO，结果不可覆盖 |
| reset request/result | 固定容量 SPSC | ipc_control/backend | backend/ipc_control | 请求—应答 |

latest slot 使用共享 spin lock 保护普通 payload，避免用非法的无锁 seqlock 读取非原子对象。
队列保持单生产者、单消费者：gateway 内部用 mutex 串行化 action/service 请求写入；
`quadruped_ipc_control` 是 reset 控制队列的唯一生产者。

启动时还会校验：

- wire magic 和 wire schema version；
- core frame schema version；
- robot name；
- joint count 和 ordered joint names。

每个运行数据都携带适用的 `schema_version`、`startup_id`、`session_id`、序列或请求编号。
任一 startup 或 session 不匹配时，旧状态、旧命令和旧请求都不能进入当前控制会话。

## 4. 周期和超时

- `mujoco_backendd`：按 MuJoCo scene 的 physics timestep 步进；
- `motiond`：按 `ControllerConfig.control_period_ns` 调度，black/blackW 当前均为 5 ms；
- CommandFrame：按 controller 配置失效，black/blackW 当前均为 10 ms；
- BaseCommand：gateway 默认有效 200 ms，可用 `cmd_vel_timeout_ns` 参数覆盖；
- backend、motion heartbeat：500 ms 后判定断开；
- ROS action/service：gateway 最长等待 motion 结果 10 s；
- gateway 状态发布周期：50 ms。

BaseCommand 和 CommandFrame 的有效期相互独立。ROS 速度命令不能复用 10 ms 的关节命令
有效期。

latest slot 的短暂锁竞争不表示进程断开。读取端采用有界 CAS 重试；`RemoteRobotIO` 在同一
backend startup/session 内保留最后一次有效 heartbeat 和 StateFrame，只在槽正被发布者占用
时复用。heartbeat 超过 500 ms 或明确发布 offline 后仍会按真实断线处理。MuJoCo GUI 的显示
同步同样使用非阻塞取锁；渲染线程繁忙时只丢弃一帧画面同步，不得阻塞物理步进和 heartbeat。
物理线程错过墙钟截止时间时从当前时刻重新排程，不连续补跑已经积压的物理步。

## 5. reset 和会话

backend 首次启动建立 `session_id == 1`。用户通过键盘 `R` 或手柄 `RB + Y` 复位时，
backend 在当前 session 内加载 `default_pose`，保持单调仿真时间以及 motiond 当前行为。
MuJoCo GUI 的 Reset 加载模型零位，Load Key 加载当前选中的 XML keyframe；两者同样不建立
新 session。

管理和测试需要完整重建会话时执行：

```bash
ros2 run quadruped_gateway quadruped_ipc_control \
    /quadruped_control_black reset
```

后端成功执行管理级 reset 后 session 加一，并记录 reset 时的 CommandFrame slot version。
reset 前留下的旧命令不会在新 session 再提交。motiond 发现 startup/session 改变后清除旧
BaseCommand、活动请求跟踪和控制调度基准；gateway 只接收当前 motion startup 和 backend
session 的结果。

## 6. 故障退路

| 故障 | 行为 |
| --- | --- |
| gateway 停止 | 旧 BaseCommand 在默认 200 ms 后过期，active source 回到 None |
| motiond 停止 | backend 不再收到新 CommandFrame，10 ms 后旧命令失效并进入执行侧安全输出 |
| backend 停止 | RemoteRobotIO 在 500 ms 后返回 Disconnected，活动请求进入 Failed，不再生成有效主动命令 |
| session 改变 | 旧命令、旧 BaseCommand、旧请求和旧结果全部拒绝 |
| wire/enum/数值非法 | 转换或 core 校验失败，不使用该帧 |

## 7. 构建、测试和模块化运行入口

主工程统一使用 `scripts/build.sh`，构建目录为 `.build/default`、`.build/mujoco` 和
`.build/rl`：

```bash
./scripts/build.sh
./scripts/build.sh --target backend --backend mujoco
./scripts/build.sh --target motion
./scripts/build.sh --target command
./scripts/test/ctest.sh
./scripts/test/ros2_headless.sh
```

`core` 使用 default profile，`backend` 使用 MuJoCo profile，`motion` 使用 MuJoCo + LibTorch
RL profile，`command` 使用 `.build/ros2/` 下的固定 colcon 工作区，安装环境为
`.build/ros2/install/setup.bash`。默认 `./scripts/build.sh` 先完成 RL CMake 构建和 CTest，
再执行 ROS 2 colcon；`--target command` 只执行 ROS 2 构建，`./scripts/setup/ros2.sh` 可单独
构建并安装 ROS 2 包环境。旧的 `--mujoco`、`--rl` 参数仍兼容。

人工运行按职责拆成三个独立终端，后端创建共享内存和 session：

```bash
./scripts/run/backend.sh black plain
./scripts/run/motion.sh black
./scripts/run/command.sh black keyboard
# 使用手柄时显式选择，避免键盘和手柄同时抢占输入
./scripts/run/command.sh joystick
# 也可以显式写机器人名称：
./scripts/run/command.sh black joystick
```

backend 不依赖 ROS 2，支持 `plain|terrain`、`gui|headless`、共享内存、MJCF、robot config
和实时倍率覆盖；motion 不读取终端、不启动 ROS 2 或 GUI，只负责 MotionRuntime 和
`policy_switch.yaml` 中列出的策略；command 负责 `/cmd_vel`、action/service、键盘和规范化 `/joy`。command 的
主接口支持 `[robot] [keyboard|joystick]` 或直接使用 `[keyboard|joystick]`，默认只启动键盘；如果工程 ROS 2 安装环境或 gateway
产物缺失，脚本会自动调用 `scripts/setup/ros2.sh` 完成构建安装，系统基础环境仍需存在于
`/opt/ros/humble/setup.bash`（或由 `ROS_SETUP` 指定）。joystick 模式才会启动
`controller_input`。`--shm` 只用于测试或多实例，ROS 2 特殊参数通过 `--ros-args` 传递。

策略循环读取 `configs/policies/<robot>/policy_switch.yaml`：`policy_config_cycle` 中的每一项
对应同目录下的 `<name>.yaml`，同时决定启动加载白名单和 toggle/Y 的循环顺序；未列出的策略
不会进入运行时循环。`posture_transition_cycles` 传给 MotionRuntime，用于策略默认姿态不同
时的插值过渡。

`configs/input/gamepads.yaml` 只保存物理手柄到固定 8 轴/12 按钮 `Joy` 数组的映射，不保存
动作或 MotionMode 语义。正式入口统一位于 `scripts/run/`、`scripts/test/` 和 `scripts/debug/`；
`black_simulation.launch.py` 继续用于 CI/自动化，不作为人工三进程主入口。
