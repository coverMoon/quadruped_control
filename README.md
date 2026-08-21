# quadruped_control

四足和轮足机器人的运动控制工程。

## 当前实现

仓库目前包含独立的 C++17 控制核心，以及可选的 MuJoCo、Torch 和 ROS 2 运行链路：

- 固定最大 16 关节的公共数据结构；
- `StateFrame`、`CommandFrame`、`BaseCommand` 和模式请求；
- 机器人名称、关节顺序、功能角色和限制定义；
- `RobotIO` 统一接口；
- 模型、帧、时间、控制模式和数值合法性检查；
- black 的关节顺序参考配置；
- 核心接口单元测试；
- `backends/mujoco/` 中的 `MujocoRobotIO`，支持模型加载、reset、命令执行和显式单物理步进；
- `config_loader/` 中的启动期 YAML 配置加载，统一提供 black 的 `RobotModel` 和 `ControllerConfig`；
- `motion/` 中不依赖外部框架的 `MotionRuntime`，实现
  `Passive → GetUp → Stand → GetDown → Passive` 基础运动闭环；
- `motion/` 中固定 black 数据布局的 `RlController`，直接实现与 `rl_sar` 一致的
  45 维观测、6 帧历史和动作换算；
- `policy/torch/` 中的轻量 `TorchPolicy`，只负责单个 TorchScript 模型的加载和 forward；
- flat/obstacle 策略运行时切换及请求终态管理；
- `adapters/ipc/` 中的固定容量本机共享内存协议和远程 `RobotIO`；
- `apps/runtime_daemons/` 中相互独立的 `motiond` 和 `mujoco_backendd`；
- `adapters/ros2/quadruped_gateway/` 中的 ROS 2 topic、action 和 service 网关；
- `tests/mujoco/` 中覆盖 RobotIO、MotionRuntime 和 MuJoCo 的无界面闭环测试；
- 三进程 ROS 2 headless 端到端和进程故障退路测试；
- `apps/mujoco_headless/` 中的最小无界面运行入口；
- `apps/mujoco_sim/` 中嵌入 MuJoCo 官方 Simulate 完整界面的交互仿真程序，支持起立、
  RL 行走、趴下、被动、重置和暂停。

核心库 `core/` 不依赖 ROS 2、Torch、MuJoCo 或电机 SDK。`motion/` 只依赖 `core/`，
YAML 配置加载只存在于启动期模块 `config_loader/`。MuJoCo C++ API 类型
只存在于 MuJoCo 后端 `backends/mujoco/` 及其直接使用方 `apps/mujoco_headless/`、
`apps/mujoco_sim/` 和 `tests/mujoco/` 中；依赖查找、安装脚本和模型资产分别放在
`cmake/`、`scripts/` 和 `assets/` 中。

## 当前阶段

阶段 1 至阶段 6 已完成。black 已具备基础运动、真实 TorchScript RL、flat/obstacle
运行时切换、冻结的 ROS 2 接口契约，以及以下三进程无界面仿真链路：

```text
ros2_gateway
    ↓ BaseCommand / ModeRequest
motiond
    ↓ RobotIO StateFrame / CommandFrame
mujoco_backendd
```

进程之间使用固定容量本机共享内存，包含 schema、startup、session、序列、heartbeat 和
过期语义。ROS 2 网关已接入 `/cmd_vel`、运动 action/service 和状态话题；headless 验收覆盖
正常动作、策略切换、reset，以及 gateway、motiond、backend 分别退出时的安全退路。

当前阶段仍不包含 ROS 2 launch、三进程 MuJoCo GUI、blackW、真实硬件或跨机器协议。

M1 固定使用 MuJoCo 3.9.0。依赖安装在仓库本地的 `.deps/` 目录，不依赖
Python 或 Conda 环境。

编译时运行：

```bash
./scripts/build.sh
```

脚本默认执行 Debug 构建和全部测试。需要删除旧构建结果后重新编译时运行：

```bash
./scripts/build.sh --clean
```

只编译、不运行测试：

```bash
./scripts/build.sh --no-test
```

构建需要 MuJoCo 的 M1 模块：

```bash
./scripts/build.sh --mujoco
```

构建 MuJoCo 和 LibTorch 策略模块：

```bash
./scripts/build.sh --rl
```

该参数会启用 MuJoCo 3.9.0 依赖，构建 `quadruped_mujoco` 后端、全部 MuJoCo 测试、
  `apps/mujoco_headless/` 无界面程序和 `apps/mujoco_sim/` 官方完整界面程序。
首次使用 MuJoCo 模块前运行：

```bash
./scripts/setup_mujoco.sh
```

构建 ROS 2 Humble 接口和 gateway：

```bash
./scripts/build_ros2.sh
```

该脚本在 `/tmp/quadruped_control_ros2_ws_stage6/` 下创建独立 colcon 工作区，不在仓库内
生成 ROS 2 build、install 或 log。完成主工程 `--rl` 构建和 ROS 2 构建后，可运行阶段 6
无界面端到端测试：

```bash
./scripts/test_ros2_headless.sh
```

系统依赖（Ubuntu/Debian 包名）：

- 默认构建需要 yaml-cpp 开发包：`sudo apt install libyaml-cpp-dev`；
- `--mujoco` 构建的带界面程序还需要 GLFW 3.3、OpenGL 和 libpng：
  `sudo apt install libglfw3-dev libgl1-mesa-dev libpng-dev`；
- MuJoCo 3.9.0 本身由 `setup_mujoco.sh` 安装到仓库本地 `.deps/`，不依赖 Python 或 Conda。

运行无界面仿真：

```bash
./scripts/run_mujoco_headless.sh --duration 2.0
```

运行带 GLFW 界面的 RL 交互仿真：

```bash
./scripts/run_mujoco_sim.sh
```

机器人按键在启动程序的终端中输入，MuJoCo 窗口只保留官方快捷键。按 `0` 起立，完成后按
`1` 启动 RL；`W/S`、`A/D`、`Q/E` 每次把对应速度调整 `0.1`，`Space` 将三轴速度归零。
`9` 趴下，`P` 进入被动，`R` 重置，`K` 暂停，`X` 退出。只运行不加载 LibTorch 的基础
仿真可使用 `./scripts/run_mujoco_sim.sh --basic`。
速度键只在 `rl_locomotion` 的 Running 模式生效；离开该模式会自动清零速度目标。

当前实现状态、RL 时序、ROS 2 接口和三进程运行架构分别见：

- [`docs/current_status.md`](docs/current_status.md)；
- [`docs/rl_mujoco_runtime.md`](docs/rl_mujoco_runtime.md)；
- [`docs/ros2_interface_contract.md`](docs/ros2_interface_contract.md)；
- [`docs/ros2_three_process_runtime.md`](docs/ros2_three_process_runtime.md)。

上述脚本均可从任意当前目录启动。

也可以手动执行：

```bash
cmake -S . -B build/default -DCMAKE_BUILD_TYPE=Debug
cmake --build build/default
ctest --test-dir build/default --output-on-failure
```

## 仓库目录

```text
quadruped_control/
├── apps/
│   ├── mujoco_headless/              最小无界面 MuJoCo 仿真入口
│   ├── mujoco_sim/                   带 GLFW 界面的交互式 MuJoCo 仿真
│   └── runtime_daemons/              独立 motiond 和 MuJoCo backend 进程
├── adapters/
│   ├── ipc/                          固定容量本机共享内存和 RemoteRobotIO
│   └── ros2/                         ROS 2 接口包与顶层 gateway
├── assets/
│   └── robots/black/mujoco/          固定版本的仿真模型和网格
├── backends/
│   └── mujoco/                       MuJoCo 模型加载与 RobotIO 实现
├── cmake/                            CMake 依赖查找模块
├── config_loader/                    启动期 YAML 配置加载代码
├── configs/
│   ├── controllers/                  控制器参数配置
│   ├── policies/                     精简 RL 策略参数
│   └── robots/                       机器人结构配置
├── core/
│   ├── include/quadruped/core/       核心库公共头文件
│   └── src/                          核心库实现
├── docs/
│   └── diagrams/                     架构图源文件和图片
├── motion/                           MotionRuntime、RlController 与基础运动状态机
├── policy/torch/                     单个 TorchScript 策略的推理适配器
├── scripts/                          编译、运行和开发辅助脚本
└── tests/                            自动测试
```

运行构建后还会生成：

```text
build/default/                         默认配置的缓存、目标文件和测试程序
build/mujoco/                          MuJoCo 配置的缓存、目标文件和测试程序
compile_commands.json                  指向编译数据库的符号链接
.deps/                                 脚本安装的固定版本第三方依赖
```

这些都是本地生成内容，已经被 `.gitignore` 排除，不应提交。

## 文件存放规则

- 仓库根目录只放工程级说明和构建入口，不在根目录堆放模块源码或零散脚本。
- `assets/robots/` 放固定版本的仿真模型和网格，并附来源说明；不存放构建产物。
- `cmake/` 放自定义依赖查找模块，不放业务源码。
- `core/include/quadruped/core/` 只放其他模块可以使用的公共头文件。公共头文件不能引入 ROS 2、Torch、MuJoCo 或电机 SDK。
- `core/src/` 放核心库的实现，不把只在一个 `.cpp` 中使用的辅助函数暴露到公共头文件。
- `config_loader/` 是读取和校验 YAML 的 C++ 代码模块，不存放具体机器人或控制器参数。
- `configs/robots/` 放机器人固有信息，例如关节名称、顺序、功能角色和机械限制。控制器参数和策略参数分别放入 `configs/controllers/` 与 `configs/policies/`，不能混入机器人配置。
- `docs/` 放设计文档，文档使用的架构图源文件和图片统一放在 `docs/diagrams/`。
- `scripts/` 放编译、运行、维护和开发辅助脚本。脚本应能从任意工作目录启动，不能假定调用者当前位于仓库根目录。
- `tests/` 按被测模块组织测试。M0 测试不依赖网络和第三方测试框架。
- 构建结果、日志、临时文件和编辑器生成文件不能放入源码目录，也不能提交到 Git。

`adapters/ipc/` 和 `adapters/ros2/` 可以依赖 `core`，但 `core` 不能反向依赖它们。

## 机器人匹配与标定边界

单进程应用加载一份 `RobotModel` 并同时交给 MotionRuntime 和后端。三进程运行时则在
共享内存建立控制会话前核对 wire schema、机器人名称和有序关节名称，再使用 startup 和
session 编号拒绝重启前的旧帧。机器人身份和标定编号不进入高频状态帧或命令帧。

电机零点、方向、减速比和传感器标定归最终执行侧管理。MotionRuntime 只使用统一关节
顺序下的归一化 SI 数据，不加载也不选择具体实机的标定记录。
