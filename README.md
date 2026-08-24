# quadruped_control

`quadruped_control` 是一个面向四足与轮足机器人的运动控制工程，包含 MuJoCo 仿真、TorchScript 强化学习策略、ROS 2 指令接口和运行时运动控制。

目前支持两套机器人模型：

- `black`：12 关节四足机器人；
- `blackW`：16 关节轮足机器人。

工程采用统一的控制核心和配置体系，在相同运行框架下完成基础动作、强化学习运动控制、策略切换和轮足行为。

## Features

- MuJoCo 3.9.0 GUI / headless 仿真；
- Passive、起立、站立、趴下等基础动作；
- LibTorch 加载 TorchScript 强化学习策略；
- black 的 flat / obstacle 策略；
- blackW 的 flat / obstacle / stair 策略；
- 运行时策略切换；
- blackW Retry、Event chain 和固定姿态轮驱行为；
- 键盘、手柄和 ROS 2 `/cmd_vel` 控制；
- `ros2_gateway`、`motiond`、`mujoco_backendd` 三进程运行；
- 单元测试、MuJoCo 集成测试和 ROS 2 headless 端到端测试。

## Architecture

正式仿真使用三个独立进程：

```text
keyboard / joystick / ROS 2
             │
             ▼
        ros2_gateway
             │  BaseCommand / ModeRequest
             ▼
           motiond
             │  StateFrame / CommandFrame
             ▼
     mujoco_backendd
             │
             ▼
           MuJoCo
```

`ros2_gateway` 负责外部指令与 ROS 2 接口，`motiond` 运行运动状态机和强化学习策略，`mujoco_backendd` 负责 MuJoCo 物理仿真与机器人状态读写。三个进程之间通过本机 IPC 通信。

更完整的模块关系和运行语义见 [系统架构](docs/quadruped_control_architecture.md) 和 [三进程运行架构](docs/three_process_runtime.md)。

## Repository Structure

```text
quadruped_control/
├── core/                 公共控制核心与数据结构
├── motion/               运动状态机、RL 控制器和 MotionRuntime
├── backends/             MuJoCo、Replay 等 RobotIO 后端
├── policy/               LibTorch / TorchScript 策略适配
├── adapters/             IPC、ROS 2 与输入适配
├── config_loader/        YAML 配置加载
├── configs/              机器人、控制器、策略、行为和仿真配置
├── assets/               MuJoCo 模型、网格和策略文件
├── apps/                 运行程序与调试工具
├── scripts/              构建、运行、测试和环境脚本
├── tests/                单元与集成测试
├── docs/                 架构、接口和运行文档
└── cmake/                CMake 依赖查找模块
```

## Getting Started

### Environment

推荐环境：

- Ubuntu 22.04 x86_64；
- ROS 2 Humble；
- GCC / Clang with C++17；
- CMake 3.16+。

安装常用系统依赖：

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    curl \
    unzip \
    libyaml-cpp-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libpng-dev \
    python3-colcon-common-extensions \
    python3-pygame \
    python3-yaml
```

ROS 2 默认从 `/opt/ros/humble/setup.bash` 加载。如果安装在其他位置，可以设置：

```bash
export ROS_SETUP=/path/to/ros/setup.bash
```

### Dependencies

仓库使用固定版本的 MuJoCo 和 LibTorch：

```bash
./scripts/setup/mujoco.sh
./scripts/setup/libtorch.sh
```

依赖安装到仓库内的 `.deps/`，无需 Conda 或 Python 版 PyTorch。

### Build

默认构建完整 RL 与 ROS 2 工程：

```bash
./scripts/build.sh
```

也可以按模块构建：

```bash
./scripts/build.sh --target core
./scripts/build.sh --target backend
./scripts/build.sh --target motion
./scripts/build.sh --target command
```

常用选项：

```bash
./scripts/build.sh --no-test
./scripts/build.sh --clean
```

构建结果保存在 `.build/` 下。

## Run Simulation

正式仿真建议使用三个终端依次启动 backend、motion 和 command。

### Terminal 1: backend

```bash
./scripts/run/backend.sh black terrain
```

平地场景：

```bash
./scripts/run/backend.sh black plain
```

headless 模式：

```bash
./scripts/run/backend.sh black terrain --mode headless
```

### Terminal 2: motion

```bash
./scripts/run/motion.sh black
```

也可以指定初始策略：

```bash
./scripts/run/motion.sh black flat
./scripts/run/motion.sh black obstacle
```

### Terminal 3: command

键盘控制：

```bash
./scripts/run/command.sh black keyboard
```

手柄控制：

```bash
./scripts/run/command.sh black joystick
```

`black` 是默认机器人，因此也可以直接运行：

```bash
./scripts/run/command.sh
./scripts/run/command.sh joystick
```

运行 blackW 时，将三个终端中的机器人名称改为 `blackW`：

```bash
./scripts/run/backend.sh blackW terrain
./scripts/run/motion.sh blackW
./scripts/run/command.sh blackW keyboard
```

## Control

### Keyboard

键盘输入由 `command.sh` 所在终端读取，无需切换到 MuJoCo 窗口。

| 按键 | 功能 |
| --- | --- |
| `0` | 起立并进入 Stand |
| `1` | 启动 RL locomotion |
| `2` / `3` | 切换策略或 blackW 行为 |
| `4` | blackW Car drive |
| `6` | Event chain |
| `9` | 趴下并进入 Passive |
| `P` | 进入 Passive |
| `R` | 加载 MuJoCo `default_pose` |
| `Enter` | GUI 暂停 / 继续 |
| `N` | 切换手动输入与 ROS 2 `/cmd_vel` |
| `X` | 退出 command |

RL 速度控制：

| 按键 | 功能 |
| --- | --- |
| `W` / `S` | 增加 / 减少前向速度 |
| `A` / `D` | 增加 / 减少横向速度 |
| `Q` / `E` | 增加 / 减少转向角速度 |
| `Space` | 速度归零 |

### Gamepad

常用手柄操作：

| 输入 | 功能 |
| --- | --- |
| `A` | 起立 |
| `B` | 趴下 |
| `RB + DPadUp` | RL locomotion |
| `Y` | 切换策略 |
| `LB + DPadUp` | Event chain |
| `LB + X` | Passive |
| `RB + Y` | MuJoCo reset |
| `X` | 切换手动输入与 `/cmd_vel` |
| 左摇杆 | `vx` / `vy` |
| 右摇杆左右 | `yaw` |

blackW 还提供 Bridge、Low-bar 和 Car 三种固定姿态轮驱组合键：

| 输入             | 功能    |
| ---------------- | ------- |
| `RB + DPadRight` | Bridge  |
| `RB + DPadDown`  | Low-bar |
| `RB + DPadLeft`  | Car     |

手柄型号和轴映射配置位于 `configs/input/gamepads.yaml`。

## Configuration

主要配置文件位于 `configs/`：

```text
configs/
├── robots/               机器人关节与限制
├── controllers/          控制周期、基础姿态与增益
├── policies/             RL 策略和策略切换
├── behaviors/            Retry、Event chain 与轮驱行为
├── simulation/           MuJoCo 运行参数
└── input/                手柄映射
```

每个机器人的策略位于：

```text
configs/policies/<robot>/
```

模型文件位于：

```text
assets/policies/<robot>/<policy>/
```

`policy_switch.yaml` 定义启动加载的策略和运行时切换顺序。新增策略通常只需要添加 TorchScript 模型、对应 YAML，并将策略加入切换列表。

## ROS 2

运行 ROS 2 CLI 前加载环境：

```bash
source /opt/ros/humble/setup.bash
source .build/ros2/install/setup.bash
```

发送速度指令示例：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

完整 topic、action、service 和 QoS 定义见 [ROS 2 接口契约](docs/ros2_interface_contract.md)。

## Development

运行 CTest：

```bash
./scripts/test/ctest.sh
```

运行三进程 ROS 2 headless 端到端测试：

```bash
./scripts/test/ros2_headless.sh
```

单进程 MuJoCo 调试入口：

```bash
./scripts/debug/mujoco_sim.sh
```

## Documentation

- [文档索引](docs/README.md)
- [当前能力与验证状态](docs/current_status.md)
- [系统架构](docs/quadruped_control_architecture.md)
- [RL 与 MuJoCo 运行说明](docs/rl_mujoco_runtime.md)
- [ROS 2 三进程运行架构](docs/ros2_three_process_runtime.md)
- [ROS 2 接口契约](docs/ros2_interface_contract.md)
- [blackW 模型、策略与行为](docs/blackW_模型与行为.md)
- [故障、诊断与回放](docs/fault_logging_replay.md)
