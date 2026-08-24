# quadruped_control

`quadruped_control` 是一个面向四足与轮足机器人的运动控制工程。当前仓库已经集成
**black、blackW、MuJoCo、TorchScript 强化学习策略和 ROS 2 指令入口**，可以在 Linux
以三个独立进程运行完整仿真控制链路。

本 README 以仓库怎样安装、构建和运行为主。系统设计与接口约束请参阅
[`docs/`](docs/README.md)。

## 快速导航

- [当前支持范围](#1-当前支持范围)
- [仓库目录](#2-仓库目录)
- [环境要求](#3-环境要求)
- [构建与测试](#4-构建与测试)
- [启动仿真](#5-启动仿真)
- [键盘控制](#6-键盘控制)
- [手柄控制](#7-手柄控制)
- [策略配置与切换](#8-策略配置与切换)
- [其他配置文件](#9-其他配置文件)
- [ROS 2 使用补充](#10-ros-2-使用补充)
- [常见问题](#11-常见问题)
- [只读日志回放](#12-只读日志回放)
- [当前验证边界](#13-当前验证边界)
- [进一步阅读](#14-进一步阅读)

## 1. 当前支持范围

当前已支持：

- black 12 关节四足机器人和 blackW 16 关节轮足机器人；
- MuJoCo 3.9.0 GUI 与 headless 仿真；
- 起立、站立、趴下、Passive、reset 和暂停；
- LibTorch 加载真实 TorchScript 策略；
- black 的 flat/obstacle 与 blackW 的 flat/obstacle/stair 策略；
- 配置驱动的运行时策略切换、Retry、Event chain 和 blackW 固定姿态轮驱；
- 键盘、手柄和 ROS 2 `/cmd_vel` 速度指令；
- `ros2_gateway`、`motiond`、`mujoco_backendd` 三进程本机运行；
- 核心单元测试、MuJoCo 集成测试和 ROS 2 headless 端到端测试。

正式仿真运行链路如下：

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

当前范围聚焦仿真、测试和回放；具体已实现能力和场景验证边界见
[当前能力与验证边界](docs/current_status.md)。

## 2. 仓库目录

日常使用时优先关注以下目录：

```text
quadruped_control/
├── core/                 无 ROS 2、Torch、MuJoCo 依赖的公共控制核心
├── configs/              机器人、控制器、策略、手柄和仿真配置
├── motion/               基础运动状态机、RL 控制器和 MotionRuntime
├── backends/             MuJoCo 等机器人状态与关节命令后端
├── assets/               MuJoCo 模型、网格和 TorchScript 策略文件
├── apps/                 motiond、mujoco_backendd 和调试程序
├── scripts/              依赖安装、构建、运行、测试和调试脚本
├── adapters/             本机 IPC、ROS 2 接口与输入适配
├── config_loader/        YAML 配置加载和启动期校验
├── policy/               LibTorch/TorchScript 策略适配器
├── tests/                单元、集成和仿真测试
├── docs/                 当前能力、架构、接口和运行原理文档
└── cmake/                CMake 依赖查找模块
```

`scripts/` 已按用途整理：

```text
scripts/
├── build.sh              统一构建入口
├── setup/                MuJoCo、LibTorch 和 ROS 2 工程环境准备
├── run/                  三进程正式运行入口
├── test/                 CTest 和 ROS 2 端到端测试
└── debug/                单进程 MuJoCo 等本地调试入口
```

脚本会将本地依赖和构建结果放在：

```text
.deps/                    固定版本的 MuJoCo、LibTorch
.build/default/           不启用 MuJoCo/Torch 的 CMake profile
.build/mujoco/            启用 MuJoCo 的 CMake profile
.build/rl/                启用 MuJoCo + LibTorch 的 CMake profile
.build/ros2/              ROS 2 colcon 工作区
.build/ros2/install/       本仓库 ROS 2 包的安装环境
```

这些目录均为本地生成内容，不应提交到 Git。

## 3. 环境要求

### 3.1 推荐平台

当前依赖安装脚本面向：

- Linux x86_64；
- Ubuntu 22.04；
- ROS 2 Humble；
- 支持 C++17 的 GCC 或 Clang；
- CMake 3.16 或更高版本。

其中 ROS 2 基础环境默认位于：

```text
/opt/ros/humble/setup.bash
```

如果 ROS 2 安装在其他位置，可以在构建或运行前设置：

```bash
export ROS_SETUP=/path/to/ros/setup.bash
```

### 3.2 系统依赖

Ubuntu 22.04 可先安装常用构建、GUI 和手柄依赖：

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

还需要提前安装 ROS 2 Humble。ROS 2 安装完成后，可以用下面的命令确认基础环境：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
colcon --help
```

如使用精简 ROS 2 安装，请确保包含本仓库用到的 `ament_cmake`、`rclcpp`、
`rclcpp_action`、`geometry_msgs`、`sensor_msgs`、`action_msgs`、ROSIDL、`launch` 和`launch_ros`。

### 3.3 仓库本地依赖

工程固定使用：

- MuJoCo `3.9.0`；
- LibTorch `2.0.1 CPU`，C++11 ABI。

安装脚本会下载并校验固定版本，然后安装到仓库的 `.deps/`，不会放到系统根目录，也不依赖
Conda 或 Python 版 PyTorch：

```bash
./scripts/setup/mujoco.sh
./scripts/setup/libtorch.sh
```

首次执行需要网络；后续构建直接复用 `.deps/` 中的文件。

## 4. 构建与测试

### 4.1 统一构建入口

```bash
./scripts/build.sh [--target all|core|backend|motion|command] [选项]
```

常用命令：

| 命令 | 用途 |
|---|---|
| `./scripts/build.sh` | 默认全量构建：RL CMake/CTest，再构建 ROS 2 |
| `./scripts/build.sh --target core` | 构建不启用 MuJoCo/Torch 的基础 profile |
| `./scripts/build.sh --target backend` | 构建 MuJoCo profile |
| `./scripts/build.sh --target motion` | 构建 MuJoCo + LibTorch 的 RL profile |
| `./scripts/build.sh --target command` | 只构建并安装 ROS 2 接口和网关 |
| `./scripts/build.sh --no-test` | 构建所选 profile，但跳过 CTest |
| `./scripts/build.sh --clean` | 清理所选 profile 后重新配置和构建 |

### 4.2 单独准备 ROS 2 工程环境

```bash
./scripts/setup/ros2.sh
```

清理并重建：

```bash
./scripts/setup/ros2.sh --clean
```

`command.sh` 在发现 `.build/ros2/install/setup.bash` 或 gateway 缺失时，也会自动调用此脚本。
系统级 ROS 2 Humble 仍需提前安装。

### 4.3 测试

运行 RL profile 的 CTest：

```bash
./scripts/test/ctest.sh
```

也可以明确选择 profile：

```bash
./scripts/test/ctest.sh default
./scripts/test/ctest.sh mujoco
./scripts/test/ctest.sh rl
```

运行三进程 ROS 2 headless 端到端测试：

```bash
./scripts/test/ros2_headless.sh
```

该测试要求 RL profile 和 ROS 2 工程环境已经构建完成。

## 5. 启动仿真

正式人工运行使用三个终端，分别启动 backend、motion 和 command。建议按下面的顺序启动。

### 5.1 终端 1：启动 MuJoCo backend

平地 GUI：

```bash
./scripts/run/backend.sh black plain
```

地形 GUI：

```bash
./scripts/run/backend.sh black terrain
```

headless：

```bash
./scripts/run/backend.sh black terrain --mode headless
```

`backend.sh` 的位置参数为：

```text
backend.sh [robot] [scene]
```

- `robot` 默认 `black`；
- `scene` 支持 `plain` 和 `terrain`，默认是 `terrain`；
- `--mode` 支持 `gui` 和 `headless`，默认是 `gui`；
- 默认共享内存名称是 `/quadruped_control_<robot>`。

其他可选参数：

```bash
./scripts/run/backend.sh black terrain \
    --real-time-factor 1.0 \
    --visual-sync-hz 60 \
    --vsync true
```

### 5.2 终端 2：启动 MotionRuntime 和策略

```bash
./scripts/run/motion.sh black
```

未指定初始策略时，使用 `configs/policies/black/policy_switch.yaml` 中第一个可用策略。
也可以指定循环内的初始策略：

```bash
./scripts/run/motion.sh black flat
./scripts/run/motion.sh black obstacle
```

`motion.sh` 会自动选择当前机器人的 robot、controller、policy switch 和策略配置，不需要在命令行
重复传入各个 YAML 或模型路径。

### 5.3 终端 3：启动键盘或手柄指令入口

默认机器人和默认键盘：

```bash
./scripts/run/command.sh
```

显式写法：

```bash
./scripts/run/command.sh black keyboard
```

使用手柄：

```bash
./scripts/run/command.sh joystick
```

或者显式指定机器人：

```bash
./scripts/run/command.sh black joystick
```

`command.sh` 的默认机器人是 `black`，默认输入方式是 `keyboard`。单参数 `keyboard`、
`joystick` 或兼容别名 `gamepad` 会被识别为输入方式，不需要先写机器人名称。

blackW 使用同一组入口，只需让三个终端选择相同机器人：

```bash
./scripts/run/backend.sh blackW terrain
./scripts/run/motion.sh blackW
./scripts/run/command.sh blackW keyboard
```

### 5.4 推荐操作顺序

仿真窗口和三个进程均启动后：

1. 使用 `0` 或手柄 `A` 起立；
2. 等待状态进入 `Stand`；
3. 使用 `1` 或手柄 `RB + DPadUp` 进入 `RL locomotion`；
4. 使用键盘速度键或摇杆控制速度；
5. 使用 `2`/`3` 或手柄 `Y` 切换策略；

键盘模式可按 `X` 退出 command，也可以在各终端使用 `Ctrl+C`。

## 6. 键盘控制

键盘由运行 `command.sh` 的终端读取，不需要在 MuJoCo 窗口中按键，也不需要按 Enter 确认。

### 6.1 模式和运行控制

| 按键 | 功能 |
|---|---|
| `0` | 执行 GetUp，完成后进入 `Stand` |
| `1` | 从 `Stand` 启动 RL，或从 Event chain 平滑返回 RL |
| `2` / `3` | black 切换策略；blackW 分别进入 Bridge/Low-bar drive |
| `4` | blackW 进入 Car drive |
| `6` | 启动已配置的 Event chain |
| `9` | 执行 GetDown，完成后回到 `Passive` |
| `P` | 立即请求进入 `Passive` |
| `R` | 加载 MuJoCo `default_pose`，保持当前 session 和运动行为 |
| `Enter` | GUI 模式下暂停/继续物理仿真 |
| `N` | 在终端手动指令和 ROS 2 `/cmd_vel` 导航指令之间切换 |
| `H` | 在日志中重新输出按键帮助 |
| `X` | 退出 command 进程 |

`Enter` 的暂停请求只适用于 GUI backend；headless 模式没有可暂停的 GUI 交互状态。

### 6.2 RL 速度控制

| 按键 | 指令变化 |
|---|---|
| `W` / `S` | `vx` 增加/减少 `0.1 m/s` |
| `A` / `D` | `vy` 增加/减少 `0.1 m/s` |
| `Q` / `E` | `yaw` 增加/减少 `0.1 rad/s` |
| `Space` | `vx`、`vy`、`yaw` 全部归零，并切回手动输入 |

速度键只在 `Running` 且行为为 `rl_locomotion` 时生效。在 Passive、GetUp、Stand 和 GetDown
中不会提前缓存速度。离开 RL 模式后，速度目标会自动清零。

进入 RL 模式后，command 终端会原地刷新当前策略和速度，例如：

```text
RL Controller policy=flat x:0.00 y:0.00 yaw:0.00
```

MotionRuntime 会按当前策略 YAML 的 `command_limits` 做最终限幅，并通过 MotionStatus 把同一
上限传给 command gateway。键盘、导航输入均按该值限幅；手柄的 `[-1, 1]` 归一化轴会自动
缩放到当前策略的完整 `[-limit, limit]` 量程，策略切换后同步更新。

### 6.3 单进程调试程序的按键差异

单进程 MuJoCo 调试入口使用：

```bash
./scripts/debug/mujoco_sim.sh
```

其基础模式不加载 Torch：

```bash
./scripts/debug/mujoco_sim.sh --basic
```

调试程序中的运动和速度键与上表相同，但有以下差异：

- `5`：进入 Retry；
- `K`：暂停/继续；
- `H`：重新显示帮助；
- `X` 或 `Esc`：退出；
- 不使用 `N` 切换 ROS 2 `/cmd_vel`，因为该入口不是三进程 ROS 2 主路径。

## 7. 手柄控制

### 7.1 启动和连接状态

使用手柄时必须选择 `joystick` 模式：

```bash
./scripts/run/command.sh joystick
```

command 终端第一行显示连接状态。未检测到手柄时以黄色显示：

```text
[Controller] Not connected
```

识别成功后会显示选中的 profile，例如：

```text
[Controller] Connected: beitong_btp_kp20d
```

当前支持的配置 profile：

- `xbox`；
- `gaishi_xiaoji`；
- `beitong_btp_kp20d`；
- `shamohu`。

物理设备名与 profile 的匹配、轴方向、按钮编号和 D-pad 映射位于：

```text
configs/input/gamepads.yaml
```

未知手柄不会套用一个可能危险的默认映射，需要先在该文件中增加并验证 profile。

### 7.2 手柄键位

键位语义与旧 `rl_sar` 的 black 基础操作对齐：

| 手柄输入 | 功能 |
|---|---|
| `A` | 执行 GetUp，完成后进入 `Stand` |
| `B` | 执行 GetDown |
| `RB + DPadUp` | 进入 `RL locomotion` |
| `RB + DPadRight` | blackW 进入 Bridge drive |
| `RB + DPadDown` | blackW 进入 Low-bar drive |
| `RB + DPadLeft` | blackW 进入 Car drive |
| `LB + DPadUp` | 启动已配置的 Event chain |
| `LB + X` | 进入 `Passive` |
| `RB + Y` | 加载 MuJoCo `default_pose`，保持当前运动行为 |
| `RB + X` | GUI 模式下暂停/继续 |
| `Y` | 切换到策略循环中的下一项 |
| `X` | 在手柄手动指令和 ROS 2 `/cmd_vel` 导航指令之间切换 |
| 左摇杆上下 | 控制 `vx` |
| 左摇杆左右 | 控制 `vy` |
| 右摇杆左右 | 控制 `yaw` |

组合键以按下边沿触发。使用 `LB + X`、`RB + X` 或 `RB + Y` 时，单独的 `X`、`Y` 动作不会
同时触发。

black 保持 `2`/`3` 策略切换且不响应 blackW 专用 D-pad 组合；blackW 启动脚本会加载三份
固定姿态轮驱配置并启用对应键位。`LB + DPadUp` 通过同一公共请求启动 Event chain。

### 7.3 手柄无响应时检查

先确认 Python 依赖：

```bash
/usr/bin/python3 -c 'import pygame, yaml; print(pygame.version.ver)'
```

再检查：

1. 是否使用了 `./scripts/run/command.sh joystick`，而不是默认 keyboard；
2. 操作系统和 pygame/SDL 是否能看到设备；
3. 终端是否显示 `Connected: <profile>`；
4. 设备名称是否能匹配 `configs/input/gamepads.yaml` 中某个 `match`；
5. 北通手柄是否匹配 `beitong_btp_kp20d`；
6. 实际轴和按钮编号是否与对应 profile 一致。

手柄输入进程的默认日志位于：

```text
/tmp/quadruped_control_ros2_logs/controller_input.log
```

## 8. 策略配置与切换

每个机器人的策略目录位于：

```text
configs/policies/<robot>/
```

black 当前包含：

```text
configs/policies/black/
├── flat.yaml
├── obstacle.yaml
└── policy_switch.yaml
```

blackW 目录结构相同，并包含 `flat.yaml`、`obstacle.yaml`、`stair.yaml` 和
`policy_switch.yaml`。行为配置不放在策略循环中，统一位于 `configs/behaviors/<robot>/`。

### 8.1 `policy_switch.yaml`

当前配置示例：

```yaml
black:
  posture_transition_cycles: 150
  policy_config_cycle:
    - flat
    - obstacle
```

规则如下：

- `policy_config_cycle` 是策略加载白名单，也是运行时循环顺序；
- `flat` 对应同目录下的 `flat.yaml`；
- 仅把新的 `foo.yaml` 放进目录，不会自动加入策略循环；
- 必须将 `foo` 显式加入 `policy_config_cycle`；
- 默认初始策略是列表中的第一项；
- `motion.sh black foo` 只能选择已经列入循环并成功加载的策略；
- 键盘 `2`/`3`、手柄 `Y` 和单进程调试程序 `2`/`3` 都按该列表循环；
- `posture_transition_cycles` 控制策略切换时默认关节姿态的插值过渡周期；
- 重复、非法或找不到 YAML 的项目会在启动阶段给出警告并跳过；如果最终没有可用策略，motion 不会继续启动；
- 当前运行时最多注册 8 个策略，超过容量会启动失败。

### 8.2 单个策略 YAML

例如 `configs/policies/black/flat.yaml` 主要定义：

- 策略名称和适用机器人；
- TorchScript 模型路径；
- 观测缩放和裁剪；
- 动作缩放和裁剪；
- 默认关节姿态；
- 策略使用的 KP/KD；
- `vx`、`vy`、`yaw` 的 `command_limits`。

模型文件本身位于：

```text
assets/policies/<robot>/<policy>/
```

新增策略时，建议按下面的顺序操作：

1. 将 TorchScript 模型放入 `assets/policies/<robot>/<policy>/`；
2. 在 `configs/policies/<robot>/<policy>.yaml` 写入模型路径和运行参数；
3. 将策略名加入 `policy_switch.yaml`；
4. 重启 `motion.sh`，让 motiond 重新加载策略循环；
5. 验证默认姿态、速度限幅、观测顺序和切换过渡。

只新增模型和 YAML 时不需要重新编译 C++；修改运行时代码后才需要重新执行对应 build target。

## 9. 其他配置文件

### 9.1 机器人配置

```text
configs/robots/<robot>.yaml
```

定义机器人固有信息，包括：

- 机器人名称；
- 统一关节名称和严格顺序；
- 每个关节的 `role`；
- 是否具有位置限制；
- 位置、速度、力矩和增益上限。

`role` 表示关节在控制系统中的**功能角色**，例如 `leg`、`wheel`，不是 URDF/MJCF 中的关节
类型，也不能根据关节名称或运动范围自动推断。它用于让上层控制逻辑理解关节用途；仿真几何
和物理关节类型仍由 URDF/MJCF 定义。

### 9.2 控制器配置

```text
configs/controllers/<robot>.yaml
```

定义基础运动控制参数，包括：

- 控制周期和命令有效期；
- 起立、站立和趴下的插值周期；
- `pre_getup_position` 和 `stand_position`；
- 基础动作使用的固定 KP/KD；
- 与机器人配置一致的关节顺序。

### 9.3 仿真配置

```text
configs/simulation/<robot>_mujoco.yaml
```

当前 black 和 blackW 各有一份 MuJoCo 配置，均定义：

- `real_time_factor`：仿真时间相对墙钟时间的倍率；
- `visual_sync_hz`：物理状态提交给 GUI 的频率；
- `vsync`：是否启用垂直同步。

这些 YAML 由单进程 `mujoco_sim` 调试入口读取。三进程 `backend.sh` 使用同名命令行参数和
等价默认值；临时覆盖仿真参数时，在启动 backend 时传入对应参数。

### 9.4 手柄配置

```text
configs/input/gamepads.yaml
```

该文件只负责把不同品牌手柄转换为统一的 8 轴、12 按钮 `sensor_msgs/msg/Joy` 布局，不定义
GetUp、RL、Passive 等机器人动作语义。动作语义由 ROS 2 gateway 统一处理。

## 10. ROS 2 使用补充

手动使用 ROS 2 CLI 前，需要加载系统环境和本仓库安装环境：

```bash
source /opt/ros/humble/setup.bash
source .build/ros2/install/setup.bash
```

查看运行状态：

```bash
ros2 topic echo /motion/status
ros2 topic echo /robot_io/status
ros2 topic echo /state/diagnostic
ros2 topic echo /motion/result
```

发送导航速度：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

在 command 终端按 `N`，或在手柄上按单独的 `X`，切换到 navigation 输入后，`/cmd_vel` 才会
成为当前速度来源。停止发布后，导航命令会按 gateway 的超时规则失效。

完整 action、service、topic 和 QoS 契约见
[`docs/ros2_interface_contract.md`](docs/ros2_interface_contract.md)。

## 11. 常见问题

### 构建提示找不到 MuJoCo 或 LibTorch

先运行：

```bash
./scripts/setup/mujoco.sh
./scripts/setup/libtorch.sh
```

不要通过 Conda 的 PyTorch 或系统中另一个 MuJoCo 目录替代仓库固定依赖。

### 找不到 `/opt/ros/humble/setup.bash`

说明系统 ROS 2 Humble 尚未安装，或者安装路径不同。路径不同可设置：

```bash
export ROS_SETUP=/actual/path/setup.bash
./scripts/build.sh --target command
```

### command 启动时自动开始 colcon 构建

这是预期行为。`command.sh` 检测到本仓库 ROS 2 安装环境缺失或不完整时，会调用：

```bash
./scripts/setup/ros2.sh
```

安装位置仍然是固定的 `.build/ros2/install/`。

### motion 或 command 无法连接共享内存

确认：

1. backend 已经先启动；
2. 三个终端使用相同机器人名称；
3. 三个进程使用相同 `--shm`；
4. 没有另一个旧 backend 占用同名实例。

### 已进入 RL，但机器人速度没有变化

确认：

- 状态已经是 `Running`，而不是 GetUp 或 Stand；
- 当前行为是 `rl_locomotion`；
- command 状态行中的速度值确实发生变化；
- 当前输入来源是 manual，或者 navigation 模式下 `/cmd_vel` 正在持续发布；
- 当前策略的 `command_limits` 没有将该方向限制为零。

### GUI 无法打开

确认当前存在图形会话，并已安装 GLFW、OpenGL 和 PNG 开发包。服务器或 CI 环境请使用：

```bash
./scripts/run/backend.sh black terrain --mode headless
```

### 策略切换后没有出现新策略

仅放置 YAML 文件不够。确认策略名已加入：

```text
configs/policies/<robot>/policy_switch.yaml
```

并确认对应 YAML、模型路径和机器人名称均有效。

## 12. 只读日志回放

默认构建会生成 `.build/default/apps/replay/quadruped_replay`。它读取带机器人名称和显式
关节顺序的 StateFrame CSV，只读驱动 MotionRuntime，并输出实际状态、目标命令和帧统计
对比；生成命令不会连接任何执行设备。格式和命令见
[故障注入、诊断日志与回放](docs/fault_logging_replay.md)。

## 13. 当前验证边界

以下内容需要继续做仿真和工具链验证：

- blackW Car/Bridge/Low-bar 的专用障碍场景验收和完整墙体接触验收；
- 不同实体手柄的长期 SDL/pygame 现场验证；
- MuJoCo GUI 长时间运行和多显示环境验证；
- 面向长时间运行的自动日志采集、绘图和性能统计；
- 将三进程 backend 的仿真参数进一步统一到 simulation YAML；
- 更多机器人、场景和策略的回归测试。

## 14. 进一步阅读

- [文档索引](docs/README.md)
- [当前能力与验证边界](docs/current_status.md)
- [RL 与 MuJoCo 运行说明](docs/rl_mujoco_runtime.md)
- [ROS 2 三进程运行架构](docs/ros2_three_process_runtime.md)
- [ROS 2 接口契约](docs/ros2_interface_contract.md)
- [系统架构](docs/quadruped_control_architecture.md)
- [blackW 模型、策略与行为](docs/blackW_模型与行为.md)
- [故障注入、诊断日志与回放](docs/fault_logging_replay.md)
