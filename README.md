# quadruped_control

四足和轮足机器人的运动控制工程。

## 当前实现

仓库目前包含一个独立的 C++17 核心库：

- 固定最大 16 关节的公共数据结构；
- `StateFrame`、`CommandFrame`、`BaseCommand` 和模式请求；
- 机器人名称、关节顺序、功能角色和限制定义；
- `RobotIO` 统一接口；
- 模型、帧、时间、控制模式和数值合法性检查；
- black 的关节顺序参考配置；
- 核心接口单元测试。

核心库不依赖 ROS 2、Torch、MuJoCo 或电机 SDK。

## 当前阶段

当前 M0 已完成，正在进行 M1 的 MuJoCo 仿真准备。公共数据结构、
`RobotModel`、`RobotIO` 和基础校验已经完成并通过测试。

M0 只确定模块之间传递什么数据以及怎样检查数据。M1 开始增加无界面的
MuJoCo 物理仿真，仍不包含运动状态机、策略推理和 ROS 2 适配器。

M1 固定使用 MuJoCo 3.9.0。依赖安装在仓库本地的 `.deps/` 目录，不依赖
Python 或 Conda 环境。首次使用时运行：

```bash
./scripts/setup_mujoco.sh
```

## 构建和测试

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

也可以手动执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

## 仓库目录

```text
quadruped_control/
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
build/                                 CMake 缓存、目标文件和测试程序
compile_commands.json                  指向编译数据库的符号链接
.deps/                                 脚本安装的固定版本第三方依赖
```

这些都是本地生成内容，已经被 `.gitignore` 排除，不应提交。

## 文件存放规则

- 仓库根目录只放工程级说明和构建入口，不在根目录堆放模块源码或零散脚本。
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
backends/mujoco/                        MujocoRobotIO 和物理仿真
adapters/ros2/                          ROS 2 消息转换和外围接口
apps/mujoco_sim/                        完整仿真程序的组装与启动入口
```

这些模块可以依赖 `core`，但 `core` 不能反向依赖它们。没有开始实现的模块暂时不创建空目录。

## ID 生成

模型 ID 由稳定的机器人名称和模型版本生成：

```bash
./scripts/generate_id.py model black 1
```

实机标定 ID 由机器人名称、整机序列号和标定版本生成：

```bash
./scripts/generate_id.py calibration black BLACK-001 1
```

脚本使用 SHA-256 的前 64 位，输出可直接填写到 YAML 的十六进制值。相同输入始终产生相同 ID。`calibration_id: 0` 只用于仿真或不需要实机标定的情况。
