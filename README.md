# quadruped_control

四足和轮足机器人的运动控制工程。

## 当前实现

仓库目前包含一个独立的 C++17 核心库，以及可选的 MuJoCo 仿真后端：

- 固定最大 16 关节的公共数据结构；
- `StateFrame`、`CommandFrame`、`BaseCommand` 和模式请求；
- 机器人名称、关节顺序、功能角色和限制定义；
- `RobotIO` 统一接口；
- 模型、帧、时间、控制模式和数值合法性检查；
- black 的关节顺序参考配置；
- 核心接口单元测试；
- `backends/mujoco/` 中的 `MujocoRobotIO`，支持模型加载、reset、命令执行和显式单物理步进；
- `tests/mujoco/` 中的 MuJoCo 自动测试，覆盖状态生成、命令校验、三种控制模式、安全路径和可重复性；
- `apps/mujoco_headless/` 中的最小无界面运行入口。

核心库 `core/` 不依赖 ROS 2、Torch、MuJoCo 或电机 SDK。MuJoCo C++ API 类型
只存在于 MuJoCo 后端 `backends/mujoco/` 及其直接使用方 `apps/mujoco_headless/`
和 `tests/mujoco/` 中；依赖查找、安装脚本和模型资产分别放在 `cmake/`、
`scripts/` 和 `assets/` 中。

## 当前阶段

当前 M1 的 MuJoCo 开环仿真已完成，准备进入 M2。公共数据结构、`RobotModel`、
`RobotIO`、基础校验、`MujocoRobotIO`、无界面运行入口和可重复性测试已经全部完成并通过测试；
MuJoCo 版本和 black 平地基准模型也已经确定。

M0 只确定模块之间传递什么数据以及怎样检查数据。M1 增加了无界面的
MuJoCo 物理仿真，仍不包含运动状态机、策略推理和 ROS 2 适配器。

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

该参数会启用 MuJoCo 3.9.0 依赖，构建 `quadruped_mujoco` 后端、全部 MuJoCo 测试
和 `apps/mujoco_headless/` 无界面程序。首次使用 MuJoCo 模块前运行：

```bash
./scripts/setup_mujoco.sh
```

运行无界面仿真：

```bash
./scripts/run_mujoco_headless.sh --duration 2.0
```

该脚本可从任意当前目录启动。

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
│   └── mujoco_headless/              最小无界面 MuJoCo 仿真入口
├── assets/
│   └── robots/black/mujoco/          固定版本的仿真模型和网格
├── backends/
│   └── mujoco/                       MuJoCo 模型加载与 RobotIO 实现
├── cmake/                            CMake 依赖查找模块
├── configs/
│   └── robots/                       机器人结构配置
├── core/
│   ├── include/quadruped/core/       核心库公共头文件
│   └── src/                          核心库实现
├── docs/
│   └── diagrams/                     架构图源文件和图片
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
- `configs/robots/` 放机器人固有信息，例如关节名称、顺序、功能角色和机械限制。控制器参数和策略参数以后分别放入 `configs/controllers/` 与 `configs/policies/`，不能混入机器人配置。
- `docs/` 放设计文档，文档使用的架构图源文件和图片统一放在 `docs/diagrams/`。
- `scripts/` 放编译、运行、维护和开发辅助脚本。脚本应能从任意工作目录启动，不能假定调用者当前位于仓库根目录。
- `tests/` 按被测模块组织测试。M0 测试不依赖网络和第三方测试框架。
- 构建结果、日志、临时文件和编辑器生成文件不能放入源码目录，也不能提交到 Git。

后续开始相应功能时，再按下面的位置创建目录：

```text
motion/                                 MotionRuntime、状态机和策略运行
adapters/ros2/                          ROS 2 消息转换和外围接口
```

这些模块可以依赖 `core`，但 `core` 不能反向依赖它们。没有开始实现的模块暂时不创建空目录。

## ID 分配

`model_id` 和实机使用的 `calibration_id` 是项目明确分配的非零 64 位编号，不从名称、
序列号或其他字段推导。编号一旦用于配置或通信协议就保持稳定；新增机器人型号和标定
记录时，应在对应配置中明确填写尚未使用的编号。

`model_id` 用于拒绝发给其他机器人结构版本的帧。`calibration_id: 0` 只用于仿真或
不需要实机标定的情况，实机标定编号由具体实机配置明确分配。
