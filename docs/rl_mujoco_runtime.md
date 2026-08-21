# RL 与 MuJoCo 运行说明

## 1. 构建和启动

```bash
./scripts/setup_mujoco.sh
./scripts/setup_libtorch.sh
./scripts/build.sh --rl
./scripts/run_mujoco_sim.sh
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
| `9` | 趴下 |
| `P` | 进入 Passive |
| `R` | 重置仿真并建立新会话 |
| `Space` | 三轴速度立即归零 |
| `K` | 暂停/继续物理线程 |
| `H` | 重新显示帮助 |
| `X` 或 `Esc` | 退出 |

速度目标会在松开按键后保持，不会自动归零。长按按键时，终端的键盘自动重复会连续产生
字符，因此看起来会逐级“加速”，但每一级改变的是目标速度，不是直接控制物理加速度。
策略配置的最终限幅是 `[3.0, 1.0, 3.0]`，单位依次为 m/s、m/s、rad/s；达到上限后继续
输入不会再增加。该配置位于 `configs/policies/black/flat.yaml` 的 `command_limits`。

速度键只在 `MotionMode::Running` 且行为名为 `rl_locomotion` 时生效。在 Passive、GetUp、
Stand 和 GetDown 中输入速度键只提示一次，不会提前缓存命令；离开 RL 模式会自动把三轴
目标清零。命令显示使用终端单行原地刷新，不为每次增量新增日志行。

## 2. 控制和策略频率

```text
MuJoCo 物理：dt = 2 ms               500 Hz
MotionRuntime：control_period = 5 ms  200 Hz
RL decimation：4 个控制周期             ÷ 4
TorchScript 策略目标频率                50 Hz（20 ms）
```

策略输出只在每第 4 个控制周期更新，其余 3 个周期保持最近一次关节目标。因此“50 Hz 策略”
不等于“50 Hz 物理”，关节阻抗命令仍以 200 Hz 提交。

当前机器连续推理基准约为：

- flat：平均约 `0.07 ms`；
- obstacle：平均约 `0.09 ms`。

此前观察到的约 `35 ms` 来自 Torch JIT 的首次执行与图优化，不是稳定单次耗时。策略创建时
通过正式输入路径预热三次，把初始化成本留在启动阶段。运行时仍以 `20 ms` 为单次推理期限；
超时说明这一周期已经无法满足 50 Hz，而不是把策略频率自动变成约 28 Hz。

## 4. 模型选择

- 默认交互运行加载 `assets/policies/black/flat/Flat_Jul03_15-01-07.pt`；
- obstacle 模型保留在资产目录并有参考输出测试，但当前没有运行时切换入口；
- 策略文件来源记录在 `assets/policies/black/SOURCE.md`。
