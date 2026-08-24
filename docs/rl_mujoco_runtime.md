# RL 与 MuJoCo 运行说明

## 1. 构建和启动

```bash
./scripts/setup/mujoco.sh
./scripts/setup/libtorch.sh
./scripts/build.sh --rl
./scripts/debug/mujoco_sim.sh
```

界面使用 MuJoCo 3.9.0 官方 Simulate UI，具备 File、Option、Simulation、Physics、
Rendering、Joint、Control、Sensor 和 Profiler 等原生面板。为避免与官方快捷键冲突，机器人
控制键只在启动程序的终端中读取，MuJoCo 窗口不拦截这些按键：

| 按键 | 行为 |
|---|---|
| `0` | 起立到 Stand |
| `1` | 从 Stand 启动 `rl_locomotion` |
| `W/S` | `vx` 每次增加/减少 `0.1 m/s` |
| `A/D` | `vy` 每次增加/减少 `0.1 m/s` |
| `Q/E` | `wz` 每次增加/减少 `0.1 rad/s` |
| `2` 或 `3` | black 切换策略；blackW 进入 Bridge/Low-bar drive |
| `4` | blackW 进入 Car drive |
| `5` | 进入 Retry |
| `6` | 启动 Event chain |
| `9` | 趴下 |
| `P` | 进入 Passive |
| `R` | 加载 `default_pose`，保持当前会话和行为 |
| `Space` | 三轴速度立即归零 |
| `K` | 暂停/继续物理线程 |
| `H` | 重新显示帮助 |
| `X` 或 `Esc` | 退出 |

速度目标会在松开按键后保持，不会自动归零。长按按键时，终端的键盘自动重复会连续产生
字符，因此看起来会逐级“加速”，但每一级改变的是目标速度，不是直接控制物理加速度。
最终限幅来自当前策略 YAML 的 `command_limits`，单位依次为 m/s、m/s、rad/s；达到上限后
继续输入不会再增加。手柄归一化轴也会按这三个值缩放到完整量程。

速度键只在 `MotionMode::Running` 且行为名为 `rl_locomotion` 时生效。在 Passive、GetUp、
Stand 和 GetDown 中输入速度键只提示一次，不会提前缓存命令；离开 RL 模式会自动把三轴
目标清零。命令显示使用终端单行原地刷新，不为每次增量新增日志行。

## 2. 控制和策略频率

| 环节 | black | blackW |
| --- | ---: | ---: |
| MuJoCo physics | 2 ms / 500 Hz | 2 ms / 500 Hz |
| MotionRuntime | 5 ms / 200 Hz | 5 ms / 200 Hz |
| RL decimation | 4 | 4 |
| TorchScript policy | 20 ms / 50 Hz | 20 ms / 50 Hz |

策略输出只在每第 4 个控制周期更新，其余 3 个周期保持最近一次关节目标。因此“50 Hz 策略”
不等于“50 Hz 物理”，关节阻抗命令仍以 200 Hz 提交。

策略创建时通过正式输入路径预热三次，把 Torch JIT 首次执行和图优化成本留在启动阶段。
运行时以 `20 ms` 为单次推理期限；超过期限会结束活动请求并进入 Passive，不会自动降低
策略频率。具体耗时取决于处理器、Torch 版本和系统负载，应通过运行时诊断字段测量。

## 3. 模型选择

- 策略循环由 `configs/policies/<robot>/policy_switch.yaml` 的 `policy_config_cycle` 定义；
- 列表项对应同目录下的 `<name>.yaml`，只列入循环的策略才会在启动时加载和允许 toggle/Y 切换；
- 默认启动使用列表中的第一个策略，也可以通过 `motion.sh [robot] [policy]` 指定循环内的初始策略；
- `posture_transition_cycles` 控制策略默认姿态不同于当前姿态时的位置阻抗过渡周期；
- 策略文件来源记录在 `assets/policies/<robot>/SOURCE.md`。
