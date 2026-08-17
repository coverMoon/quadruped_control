# black MuJoCo 模型来源

本目录存放 M1 阶段固定使用的 black 平地仿真模型，从专用资产仓库按提交复制，
复制后不再依赖相邻目录。模型参数（关节方向、零点、惯量、执行器范围等）以源提交为准，
本仓库不自行修改。

## 来源

- 源仓库：`XJTURoboCon_quadruped_assets`（机器人模型资产仓库-private）
- 源提交：`22c120bad450a81af2c4d6fcbf221262e2884928`
- 原路径：`mujoco/black/`

## 导入说明

`black_description.xml` 复制后清理了行尾空白（共 12 处），模型语义不变。
本仓库通过上述源提交和自身 Git 历史追踪导入来源及后续修改。

`assets/` 内的 14 个网格为上述两个 XML 实际引用的全部文件。

## 未复制的内容

- `scene_terrain.xml` 和高度图：复杂地形与 M1 的控制接口验证无关；
- `FL_foot.STL`、`FR_foot.STL`、`RL_foot.STL`、`RR_foot.STL`：源目录中存在，
  但当前两个 XML 均未引用。
