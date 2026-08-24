# RL 与 MuJoCo

## 1. 启动

准备依赖并构建：

```bash
./scripts/setup/mujoco.sh
./scripts/setup/libtorch.sh
./scripts/build.sh --target motion
```

单进程调试：

```bash
./scripts/debug/mujoco_sim.sh
```

MuJoCo 使用官方 Simulate UI。机器人控制按键由启动程序的终端读取，避免与 MuJoCo 自身快捷键冲突。

## 2. 单进程调试按键

| 按键 | 功能 |
| --- | --- |
| `0` | 起立并进入 Stand |
| `1` | 启动 `rl_locomotion` |
| `W/S` | 调整 `vx` |
| `A/D` | 调整 `vy` |
| `Q/E` | 调整 `wz` |
| `2` / `3` | black 切换策略；blackW Bridge / Low-bar |
| `4` | blackW Car drive |
| `5` | Retry |
| `6` | Event chain |
| `9` | 趴下 |
| `P` | Passive |
| `R` | 加载 `default_pose` |
| `Space` | 速度归零 |
| `K` | 暂停 / 继续物理线程 |
| `H` | 显示帮助 |
| `X` / `Esc` | 退出 |

速度每次调整 `0.1 m/s` 或 `0.1 rad/s`，最终范围由策略 YAML 的 `command_limits` 限制。离开 `rl_locomotion` 后速度目标会清零。

正式三进程的键盘和手柄操作见根目录 [README](../README.md)。

## 3. 控制频率

| 环节 | black | blackW |
| --- | ---: | ---: |
| MuJoCo physics | 500 Hz | 500 Hz |
| MotionRuntime | 200 Hz | 200 Hz |
| RL decimation | 4 | 4 |
| TorchScript policy | 50 Hz | 50 Hz |

策略每 4 个 MotionRuntime 周期更新一次，其余周期复用最近的动作目标。CommandFrame 仍以 200 Hz 生成和提交。

TorchScript 策略在启动阶段完成预热。当前推理超时阈值为 20 ms，实际耗时可从运行时诊断中查看。

## 4. 策略选择

策略列表由：

```text
configs/policies/<robot>/policy_switch.yaml
```

定义。

`policy_config_cycle` 同时决定启动加载的策略和运行时切换顺序。默认使用列表第一项，也可以在启动 `motion.sh` 时指定初始策略：

```bash
./scripts/run/motion.sh black flat
./scripts/run/motion.sh black obstacle
```

单个策略的模型、观测、动作、增益和速度限制位于同目录的 `<policy>.yaml`。模型文件位于：

```text
assets/policies/<robot>/<policy>/
```

策略默认姿态不同时，MotionRuntime 按 `posture_transition_cycles` 完成姿态过渡后再切换。
