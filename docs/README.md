# 文档导航

根目录 [README](../README.md) 提供项目概览、安装、构建和基本运行方法。`docs/` 只保留系统设计、接口契约和需要长期维护的专项说明。

## 按需求阅读

| 目标 | 文档 | 内容 |
| --- | --- | --- |
| 理解整体设计 | [系统架构](architecture.md) | 模块边界、核心数据、MotionRuntime、策略与配置职责 |
| 理解正式运行链路 | [运行时与 IPC](runtime_ipc.md) | 三进程职责、共享内存、futex、周期、session 和退出语义 |
| 接入 ROS 2 | [ROS 2 接口](ros2_interface.md) | topic、action、service、字段、QoS 和调用示例 |
| 区分机器人能力 | [机器人模型与行为](robot_models_and_behaviors.md) | black/blackW 模型、策略、Retry、Event chain 和轮驱行为 |
| 排查与复现问题 | [诊断与回放](diagnostics_replay.md) | 故障结果、诊断字段、CSV 日志和 ReplayRobotIO |

建议首次阅读顺序为：根 README → 系统架构 → 运行时与 IPC。只使用 ROS 2 接口时，可以从根 README 直接进入 ROS 2 接口文档。

## 文档职责

- 根 README 维护安装、构建、启动、操作和当前能力概览。
- 系统架构维护稳定的模块边界和内部数据语义。
- 运行时与 IPC 是三进程、共享内存和 futex 机制的唯一权威说明。
- ROS 2 接口只维护外部契约，不重复内部调度实现。
- 机器人模型与行为只记录 black/blackW 的结构、策略和行为差异。
- 诊断与回放只记录故障观测、日志格式和离线复现。

## 参考工程

部分行为和模型参数来自以下工程基线：

- `../rl_sar`：策略观测、历史、动作和行为语义；
- `../real_robot`：MuJoCo 场景与历史交互行为；
- `../URDF`：机器人几何与关节限制。

实际运行参数以本仓库代码、`configs/` 和 `assets/` 为准。
