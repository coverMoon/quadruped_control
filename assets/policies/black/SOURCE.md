# black 策略模型来源

本目录保存 black 当前使用的 TorchScript 策略模型。

## 来源

### rl_sar（HIM-era 六帧 history 策略）

- 仓库：`rl_sar`；
- 提交：`4abccaa60d4c2242466f82c8ac8ff83f4cefe9b3`；
- 许可证：Apache-2.0。

| 本仓库路径 | 源仓库路径 | 用途 |
| --- | --- | --- |
| `flat/Flat_Jul03_15-01-07.pt` | `src/rl_sar/policy/black/himloco_flat/Flat_Jul03_15-01-07.pt` | 平地策略 |
| `obstacle/best.pt` | `src/rl_sar/policy/black/himloco_obstacle/best.pt` | 障碍策略 |

### alldog_mjlab（MjLab PPO，45 维单帧策略）

- 仓库：`alldog_mjlab`；
- 提交：`6c2b715`；
- 许可证：Apache-2.0。

| 本仓库路径 | 源仓库路径 | 用途 |
| --- | --- | --- |
| `mjlab_ppo/black_ppo_actor.pt` | `logs/rsl_rl/black_velocity/2026-09-18_19-37-17/exported/black_ppo_actor.pt` | 平地策略（单帧 PPO） |

md5：`e7318ea9019cf27f2d204565659cf0cd`（与源文件逐字节一致，未重新导出）。

模型的观测、动作、关节顺序和运行参数由 `configs/policies/black/*.yaml` 定义；策略加载顺序
由同目录的 `policy_switch.yaml` 定义。
