# black MuJoCo 模型来源

本目录保存 black 当前使用的 MuJoCo 模型、场景和网格。资产复制到本仓库后独立使用，后续
修改由本仓库 Git 历史追踪。

## 模型来源

- 源仓库：`../URDF`；
- 源提交：`22c120bad450a81af2c4d6fcbf221262e2884928`；
- 原路径：`mujoco/black/`。

`black_description.xml` 导入时只清理了行尾空白，模型语义不变。`scene.xml` 是平地入口，
`assets/` 中的 14 个网格是该模型引用的本地资源。

## 当前场景

- `scene.xml`：平地场景；
- `scene_terrain.xml`：仓库内维护的综合地形场景，包含墙体、台阶、斜坡、桥面和低矮通道等
  测试对象。

场景文件负责世界几何和初始 keyframe，机器人关节名称、限制与控制顺序仍由模型 XML 和
`configs/robots/black.yaml` 共同校验。
