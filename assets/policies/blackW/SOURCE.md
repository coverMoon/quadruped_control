# blackW 策略模型来源

本目录保存阶段 9 冻结的 blackW TorchScript 模型，来源仓库 `rl_sar` 的提交
`4abccaa60d4c2242466f82c8ac8ff83f4cefe9b3`，该仓库采用 Apache-2.0 许可证。

| 本仓库文件 | 源路径 |
|---|---|
| `flat/Flat_Jun26_20-12-37.pt` | `src/rl_sar/policy/blackW/himloco_flat/Flat_Jun26_20-12-37.pt` |
| `obstacle/Obstacle_Jun26_12-30-56_.pt` | `src/rl_sar/policy/blackW/himloco_obstacle/Obstacle_Jun26_12-30-56_.pt` |
| `stair/Stair_Jul07_01-37-09.pt` | `src/rl_sar/policy/blackW/himloco_stair/Stair_Jul07_01-37-09.pt` |

三个模型均已用 TorchScript 实际调用验证：输入形状为 `[1, 342]`，输出形状为
`[1, 16]`；`342 = 57 × 6`。阶段 9 不把模型接入当前固定 12 DoF 的 RL 控制器。
