# 文档索引

文档按“当前事实、操作说明、稳定技术契约”组织。已经完成的阶段计划不再保留；未进入当前
开发范围的实机链路也不在本索引展开。

## 使用入口

| 文档 | 内容 | 适合阅读时机 |
| --- | --- | --- |
| [根目录 README](../README.md) | 安装、构建、启动、键盘和手柄操作 | 第一次运行工程 |
| [当前能力与验证边界](current_status.md) | 已实现能力、测试入口和待验收场景 | 判断仓库当前能做什么 |
| [RL 与 MuJoCo 运行说明](rl_mujoco_runtime.md) | 单进程调试、控制频率和模型选择 | 调试策略或 GUI |
| [ROS 2 三进程运行架构](ros2_three_process_runtime.md) | 进程职责、IPC、session、超时和启动方式 | 调试正式仿真链路 |

## 技术契约

| 文档 | 内容 | 适合阅读时机 |
| --- | --- | --- |
| [系统架构与运行语义](quadruped_control_architecture.md) | 依赖方向、数据结构、状态机、IPC 和测试边界 | 修改模块或公共语义之前 |
| [ROS 2 顶层接口契约](ros2_interface_contract.md) | topic/action/service、字段映射、QoS 和请求结果 | 修改 gateway 或 ROS 接口之前 |
| [blackW 模型、策略与行为](blackW_模型与行为.md) | 16 关节顺序、策略张量、Retry、Event chain 和固定轮驱 | 修改 blackW 或共同行为之前 |
| [故障、诊断与回放](fault_logging_replay.md) | 故障结果、诊断字段、CSV 和 ReplayRobotIO | 排查错误或比较控制输出 |

## 参考基线

- `../rl_sar`：策略观测、历史、动作和行为语义参考；基线 `4abccaa`，Apache-2.0；
- `../real_robot`：MuJoCo 场景与历史交互行为参考；基线 `4662151`；
- `../URDF`：模型几何与关节限制来源；基线
  `22c120bad450a81af2c4d6fcbf221262e2884928`。

参考仓库只用于核对行为和参数。当前代码、配置、测试及上述技术契约优先。

## 图表

`diagrams/` 中 `.dot` 是可编辑 Graphviz 源文件，`.svg` 是文档引用的生成结果。只有架构关系
变化时才更新，并保持源文件和 SVG 同步。
