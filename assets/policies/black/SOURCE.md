# black 策略模型来源

本目录保存 black 当前使用的 TorchScript 策略模型。

## 来源

- 仓库：`rl_sar`；
- 提交：`4abccaa60d4c2242466f82c8ac8ff83f4cefe9b3`；
- 许可证：Apache-2.0。

| 本仓库路径 | 源仓库路径 | 用途 |
| --- | --- | --- |
| `flat/Flat_Jul03_15-01-07.pt` | `src/rl_sar/policy/black/himloco_flat/Flat_Jul03_15-01-07.pt` | 平地策略 |
| `obstacle/best.pt` | `src/rl_sar/policy/black/himloco_obstacle/best.pt` | 障碍策略 |

模型的观测、动作、关节顺序和运行参数由 `configs/policies/black/*.yaml` 定义；策略加载顺序
由同目录的 `policy_switch.yaml` 定义。
