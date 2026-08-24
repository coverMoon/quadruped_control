# blackW MuJoCo 模型来源

本目录保存 blackW 当前使用的 MuJoCo 模型、场景和网格。资产复制到本仓库后独立使用，后续
修改由本仓库 Git 历史追踪。

## 来源

- 直接来源：`real_robot@4662151ab5c72e7fa28d4af2c66b5d6b0d7018d0`；
- 原路径：`blackW/src/robot_description/blackW/`；
- 网格来源：`URDF@fe699df481e4d71d3f9b474f2c77c5033e97b765`。

当前模型使用 2 ms 物理步长，并包含 16 个关节执行器、轮接触参数和 IMU 传感器。`scene.xml` 为
平地入口，`scene_terrain.xml` 为综合地形入口；RobotIO 通过关节名称和 actuator 传动建立
逻辑映射。

来源仓库根目录未提供许可证文件，因此这些队内模型与网格不能作为可向外再分发的第三方素材。
当前运行契约以本仓库模型、`configs/robots/blackW.yaml` 和
`docs/blackW_模型与行为.md` 为准。
