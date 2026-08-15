# M1 MuJoCo 开环开发方案

本文用于把 M1 分给不同的开发者或 agent。它只覆盖 MuJoCo 开环仿真，不提前实现
MotionRuntime、ROS 2、RL 推理、实机通信或图形界面。

执行者开始工作前必须完整阅读仓库根目录的 `AGENTS.md`、`README.md` 和本文。每次只完成
一个任务，测试通过并提交后再交给下一个执行者，避免多人同时修改同一组文件。

## 1. M1 的完成标准

M1 完成时应满足：

1. black 平地模型可以从本仓库加载、重置并在无界面模式下运行；
2. MuJoCo 只存在于 `backends/mujoco/` 及其测试和程序入口中，`core/` 不依赖它；
3. 程序按名称检查 12 个关节、执行器和传感器，不依靠 XML 中的排列碰运气；
4. `Disabled`、`Damping` 和 `JointImpedance` 三种命令行为明确且经过测试；
5. 仿真状态可以转换成合法的 `StateFrame`，命令可以通过 `RobotIO` 提交；
6. 固定初始状态、命令和步数可以重复得到相同结果；
7. `./scripts/build.sh` 仍能在没有 MuJoCo 的情况下构建核心代码；
8. `./scripts/build.sh --mujoco` 能构建并运行全部 MuJoCo 测试。

以下内容不属于 M1：起立和趴下状态机、RL 策略、ROS 2、渲染、噪声、通信延迟、
齿隙、复杂地形、blackW 和实机控制。

## 2. 已确定的基准模型

M1 固定使用资产仓库中的 black MuJoCo 模型：

```text
源仓库：XJTURoboCon_quadruped_assets
源提交：22c120bad450a81af2c4d6fcbf221262e2884928
主体模型：mujoco/black/black_description.xml
平地场景：mujoco/black/scene.xml
网格目录：mujoco/black/assets/
```

文件校验值（本仓库 `assets/robots/black/mujoco/` 内的文件；`black_description.xml`
在导入时清理了行尾空白，因此与源提交的 `ea3eb8e1…` 不同，模型语义不变）：

```text
black_description.xml
SHA-256 688e67eb9f3b04b3cb6bf7304ca977859828ec7ee214c0223b7241545e3c1539

scene.xml
SHA-256 1e83bd6a0e1f2c9bcfbd4240ba3c0a541e7ff436859572caedc3d4bdc2242431
```

选择依据：

- 资产仓库是机器人模型的专用来源，模型版本可以通过 Git 提交追踪；
- 这两个文件与 `black_mujoco`、`real_robot/black` 中使用的文件内容相同；
- 平地场景不包含持续变化的赛道和复杂地形，适合建立最小可重复测试；
- 已使用仓库固定的 MuJoCo 3.9.0 成功加载并推进物理仿真。

第一版不使用 `scene_terrain.xml`。该文件在不同旧目录中已有分歧，而且复杂地形与 M1 的
控制接口验证无关。

基准模型当前具有以下结构：

```text
自由基座：1 个，qpos 使用 3 个位置和 4 个四元数分量
腿关节：12 个
速度自由度：18 个（自由基座 6 个 + 腿关节 12 个）
执行器：12 个力矩电机
物理步长：2 ms
```

RobotModel、执行器和关节传感器统一使用下列顺序：

```text
FL_hip_joint, FL_thigh_joint, FL_calf_joint,
FR_hip_joint, FR_thigh_joint, FR_calf_joint,
RL_hip_joint, RL_thigh_joint, RL_calf_joint,
RR_hip_joint, RR_thigh_joint, RR_calf_joint
```

注意：XML 中机体树的书写顺序不是上述顺序。实现必须通过名称取得关节地址、速度地址、
执行器编号和传感器编号，再构造显式映射。禁止直接假设 `qpos[7 + i]`、`qvel[6 + i]`
或 `ctrl[i]` 永远对应逻辑关节 `i`。

模型中的 `range="-10 10"` 是宽泛的仿真范围，不是已经确认的实机关节限位。M1 不修改
动力学参数，也不根据该范围补写真实控制限制；限制应在后续控制器配置中单独确定。

## 3. 推荐目录和依赖方向

完成 M1 后建议形成：

```text
assets/robots/black/mujoco/       固定的 XML 和模型网格
backends/mujoco/                  MuJoCo 加载、映射、步进和 RobotIO 实现
apps/mujoco_headless/             最小无界面运行入口
tests/mujoco/                     需要 MuJoCo 的自动测试
docs/                             来源记录和开发说明
```

依赖方向固定为：

```text
apps/mujoco_headless → backends/mujoco → core
tests/mujoco         → backends/mujoco → core
```

`core/` 不得包含 MuJoCo 头文件、类型、路径和条件编译宏。模型加载、重置和物理步进是
MuJoCo 实现特有的能力，不要为了它们扩大通用 `RobotIO` 接口。

## 4. 实现约定

### 4.1 数据流

第一版使用单线程、显式步进：

```text
reset
  ↓
生成初始 StateFrame
  ↓
submit(CommandFrame)
  ↓
计算并写入 12 个执行器力矩
  ↓
MujocoRobotIO::step() 调用一次 mj_step
  ↓
生成新的 StateFrame
```

`submit()` 只校验并保存最新命令，不在调用内部偷偷推进物理仿真。物理步进只能由明确的
`MujocoRobotIO::step()` 触发，每次调用执行一个物理步长，以便测试程序准确控制仿真时间。

### 4.2 三种必要控制模式

- `Disabled`：主动执行器力矩写为零；MuJoCo 模型本身的关节阻尼和摩擦仍然存在；
- `Damping`：每个关节计算 `tau = -kd * velocity`；不读取 KP 和目标位置；
- `JointImpedance`：计算
  `tau = kp * (target_position - position) + kd * (target_velocity - velocity)
  + feedforward_effort`。

计算结果同时受 `RobotModel` 的 `max_effort` 和 MuJoCo 执行器 `ctrlrange` 限制，
取两者交集作为有效范围。最终力矩还会经过有限值检查，非有限值会触发 Fault。

M1 尚未确定正式控制器限值。当前实现和测试临时使用：

- `RobotModel::joint_limits.max_effort = 40.0`；
- black 模型执行器 `ctrlrange="-33.5 33.5"`。

因此当前有效输出范围为 `[-33.5, 33.5]`。这些都不是已经确认的实机控制限制，
正式限制将在后续控制器配置中单独确定。

`Velocity` 和 `Torque` 可以在上述三种模式稳定后补充，不得因此延迟 M1 验收。

### 4.3 时间和序号

- `StateFrame.header.timestamp_ns` 使用仿真时间换算的单调纳秒，不使用系统墙上时间；
- reset 后开始一个新的仿真会话，并从明确的状态序号起点重新生成状态；
- 每次实际生成新状态时，状态序号递增；重复读取同一最新状态不能伪造新序号；
- 过期、模型 ID 不符、会话不符、序号倒退或包含 NaN/Inf 的命令必须被拒绝；
- `latest_command_sequence` 表示最近接受的命令，`effective_command_sequence` 表示本次步进
  实际使用的命令，两者不能混为一谈。

### 4.4 状态转换

第一版 `StateFrame` 至少填写：

- 12 个关节的位置、速度和当前执行器输出力矩；
- IMU 四元数，顺序转换并确认是 `w, x, y, z`；
- IMU 角速度和线加速度，单位分别为 rad/s 和 m/s²；
- 关节与 IMU 的 `online`、`valid` 和 `age_ns`；
- 当前会话、模型 ID、仿真时间、状态序号和命令序号；
- 仿真正常时使用明确的安全状态，不得保留 `Unknown`。

不要把 MuJoCo 的裸指针或数组视图暴露到 `StateFrame`。每帧数据都应完整复制到核心类型。

## 5. 分任务交接清单

下面每个任务应由一个 agent 独立完成。接手者先运行 `git status --short`，确认没有其他人
留下的未提交修改；发现修改时先阅读并保留，不得覆盖。

### 任务 M1-1：导入并校验模型资源

目标：让 `quadruped_control` 不依赖相邻目录也能加载固定模型。

工作内容：

1. 创建 `assets/robots/black/mujoco/`；
2. 从第 2 节固定的源提交复制主体 XML、平地场景和 XML 实际引用的网格；
3. 增加简短来源说明，记录源仓库、提交、原路径和两个 XML 的 SHA-256；
4. 不复制高度图和 `scene_terrain.xml`；
5. 用 MuJoCo 3.9.0 从新路径加载 `scene.xml` 并至少推进一步；
6. 将资源目录补入 README 的目录说明，但不展开到具体文件。

验收：加载无警告失败；模型具有 12 个执行器和 18 个速度自由度；两个 XML 的校验值与
本文一致；普通构建不需要 MuJoCo。

建议提交信息：`assets: add pinned black MuJoCo model`

### 任务 M1-2：建立 MuJoCo 模块和模型加载器

目标：建立只在 `--mujoco` 配置中构建的 `quadruped_mujoco` 库。

工作内容：

1. 创建 `backends/mujoco/` 的 CMake 目标，链接 `quadruped_core` 和 `mujoco::mujoco`；
2. 用 RAII 管理 `mjModel` 和 `mjData`，明确谁负责释放；
3. 提供加载结果和可读错误信息，文件错误不得导致空指针继续运行；
4. 读取模型维度和 timestep，但不要在加载器中静默覆盖 XML 参数；
5. 建立按名称查询的关节、执行器与 IMU 传感器映射；
6. 对缺失、重复、类型错误和关节数量不符提供启动失败测试。

验收：使用正式模型加载成功；把任一名称改错的测试模型会被明确拒绝；`core/` 的依赖和
公共头文件保持不变。

建议提交信息：`feat: add MuJoCo model loader and name mapping`

### 任务 M1-3：实现 reset 和 StateFrame

目标：从仿真产生第一份完整、合法的核心状态帧。

工作内容：

1. reset 时使用 XML 的 `default_pose` keyframe；找不到时拒绝启动，不自行猜默认姿态；
2. reset 后执行必要的 `mj_forward`，保证派生状态和传感器已刷新；
3. 根据名称映射读取关节状态、执行器力矩和 IMU；
4. 正确维护仿真会话、时间、状态序号和 RobotIOStatus；
5. 为 reset 前后状态、关节顺序、四元数顺序和数值合法性增加测试。

验收：生成的帧通过核心校验；12 个关节与 `configs/robots/black.yaml` 顺序完全一致；连续
reset 得到相同初始状态。

建议提交信息：`feat: produce StateFrame from MuJoCo state`

### 任务 M1-4：实现 CommandFrame 和物理步进

目标：通过 `RobotIO` 提交命令并可控地推进仿真。

工作内容：

1. 在 `submit()` 中复用或扩展核心校验，不复制一套相互矛盾的规则；
2. 保存最新有效命令，拒绝模型、会话、数量、时间、序号和数值错误；
3. 实现 `Disabled`、`Damping` 和 `JointImpedance` 力矩计算；
4. 对最终力矩进行有限值检查和执行器范围限制；
5. 没有有效命令或命令过期时使用 `Disabled`，不得继续保持旧主动命令；
6. 每个物理步只应用一次明确的有效命令并更新实际命令序号；
7. 增加公式、限幅、过期、重复序号和错误模型测试。

验收：测试可以观测到三种模式的预期执行器力矩；错误命令不会改变已记录的最新有效命令；
过期后主动输出归零。

建议提交信息：`feat: apply joint commands in MuJoCo backend`

### 任务 M1-5：可重复性和端到端测试

目标：证明开环仿真不是只能手工运行的演示。

工作内容：

1. 从同一 keyframe reset 两次，分别施加相同命令并执行固定步数；
2. 比较最终关节位置、速度、IMU 和仿真时间；
3. 测试 `Damping` 不会产生方向错误的主动加速；
4. 测试 `JointImpedance` 的正负方向与关节状态一致；
5. 连续运行足够多的无界面步数，检查所有输出保持有限；
6. 所有测试注册到 CTest，并只在 `QUADRUPED_ENABLE_MUJOCO=ON` 时构建。

同一机器、同一构建中的两次运行应尽量逐值一致；测试断言仍使用有依据的小容差，避免编译器
或平台的微小浮点差异造成无意义失败。容差必须在测试旁说明原因。

建议提交信息：`test: verify deterministic MuJoCo open loop`

### 任务 M1-6：最小无界面程序和运行脚本

目标：提供不依赖测试框架的手动检查入口。

工作内容：

1. 创建 `apps/mujoco_headless/`，只负责组装模型、RobotModel 和 MuJoCo 实现；
2. 支持 reset、执行指定仿真秒数并打印低频摘要；
3. 默认使用仓库内 black 平地场景，允许显式覆盖场景路径；
4. 创建 `scripts/run_mujoco_headless.sh`，从任意当前目录都能运行；
5. 找不到构建结果、依赖或模型时给出可执行的中文提示；
6. 不添加 ROS 2、渲染线程、键盘控制或策略代码。

验收：新环境依次执行 `setup_mujoco.sh`、`build.sh --mujoco` 和运行脚本即可完成无界面
仿真；程序退出码能区分成功和失败。

建议提交信息：`feat: add headless MuJoCo smoke application`

### 任务 M1-7：M1 总体验收

目标：只修复验收发现的问题，不顺手进入 M2。

执行：

```bash
./scripts/build.sh --clean
./scripts/build.sh --mujoco
git diff --check
git status --short
```

还要检查：

- 默认构建日志没有查找或链接 MuJoCo；
- MuJoCo 构建运行全部核心和仿真测试；
- 无界面程序可以从仓库外的当前目录启动；
- 没有提交 `.deps/`、`build/`、日志或编辑器文件；
- README 的“当前阶段”更新为 M1 已完成、准备进入 M2；
- 本文中与最终实现不一致的接口描述得到同步修正。

建议提交信息：`docs: record M1 MuJoCo open-loop completion`

## 6. 每个 agent 的通用执行要求

每个任务都按以下顺序完成：

1. 阅读 `AGENTS.md` 和本任务说明；
2. 查看当前 Git 状态和最近提交，保留已有修改；
3. 先检查相关核心接口和测试，再设计最小改动；
4. 只实现本任务范围，不创建尚未使用的抽象层；
5. 新增 C/C++、Shell、Python 和 CMake 文件时添加规定的中文文件头；
6. C/C++ 使用 Allman 大括号、明确命名和必要的中文注释；
7. 运行与改动风险相称的构建和测试；
8. 运行 `git diff --check` 并检查没有意外生成物；
9. 提交一个主题明确的 commit；
10. 交接时列出修改、测试结果、遗留限制和 commit 哈希。

禁止事项：

- 不从旧 Python 程序整段翻译线程、ROS 2 和噪声逻辑；
- 不硬编码 Python/Conda 的 MuJoCo 路径；
- 不用数组长度猜机器人型号；
- 不用 XML 书写顺序代替名称映射；
- 不修改模型惯量、关节方向、零点、摩擦和接触参数来让测试“看起来能过”；
- 不让 MuJoCo 类型进入 `core/`；
- 不在高频步进路径读文件、打印日志或动态扩容；
- 不跳过失败测试，也不通过放大容差掩盖控制方向错误。

## 7. 必须暂停并询问的情况

遇到以下情况不要自行猜测：

1. 固定源提交不存在，或者两个基准 XML 的 SHA-256 不一致；
2. 需要改变模型的关节方向、零点、惯量、传动比或执行器范围；
3. `black.yaml` 与模型名称无法建立一一对应；
4. 实现需要修改 `StateFrame`、`CommandFrame` 或通用 `RobotIO` 接口；
5. 同一控制公式与旧实机代码表现相反，无法判断哪一侧坐标约定正确；
6. 为完成当前任务必须引入本文范围外的第三方库或 ROS 2；
7. 工作区存在来源不明且会与当前任务重叠的未提交修改。

暂停时应提供具体文件、名称、测试输入、实际结果和候选处理方式，不能只报告“模型有问题”。
