# blackW 策略模型来源

本目录保存 blackW 当前使用的 TorchScript 策略模型。

## 来源

- 仓库：`rl_sar`；
- 提交：`4abccaa60d4c2242466f82c8ac8ff83f4cefe9b3`；
- 许可证：Apache-2.0。

| 本仓库路径 | 源仓库路径 | 用途 |
| --- | --- | --- |
| `flat/Flat_Jun26_20-12-37.pt` | `src/rl_sar/policy/blackW/himloco_flat/Flat_Jun26_20-12-37.pt` | 平地策略 |
| `obstacle/Obstacle_Jun26_12-30-56_.pt` | `src/rl_sar/policy/blackW/himloco_obstacle/Obstacle_Jun26_12-30-56_.pt` | 障碍策略 |
| `stair/Stair_Jul07_01-37-09.pt` | `src/rl_sar/policy/blackW/himloco_stair/Stair_Jul07_01-37-09.pt` | 楼梯策略 |

三个模型的输入形状均为 `[1, 342]`，输出形状均为 `[1, 16]`，其中
`342 = 57 × 6`。模型已接入 blackW 的 16 关节 RL 控制路径；详细张量和动作契约由
`configs/policies/blackW/*.yaml` 及 `docs/blackW_模型与行为.md` 定义。
