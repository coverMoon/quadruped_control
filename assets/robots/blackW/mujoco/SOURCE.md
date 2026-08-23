# blackW MuJoCo 模型来源

本目录冻结阶段 9 使用的 blackW MuJoCo 模型和网格，复制后不依赖相邻仓库。

## 来源

- 直接来源：`real_robot@4662151ab5c72e7fa28d4af2c66b5d6b0d7018d0`
- 原路径：`blackW/src/robot_description/blackW/`
- 网格可追溯到资产仓库 `URDF@fe699df481e4d71d3f9b474f2c77c5033e97b765`
- `scene.xml`、高度图和全部网格与上述资产提交中的文件哈希一致。

`black_description.xml` 使用 `real_robot` 中的后续版本，其中显式设置 1 ms 物理步长、
16 个关节执行器、轮接触参数和 IMU 传感器。源仓库根目录未提供许可证文件，因此这些
队内资产不得当作可向外再分发的第三方素材。

## 阶段边界

阶段 9 只冻结模型、名称和传动事实。阶段 10 才验证 reset、16 关节 RobotIO、腿轮混合
命令和基础动作；阶段 12 才调整轮接触与越障参数。
