# 文档

根目录 [README](../README.md) 提供安装、构建和基本运行方法。这里记录系统设计、接口和专项运行说明。

| 文档 | 内容 |
| --- | --- |
| [系统架构](quadruped_control_architecture.md) | 模块边界、核心数据结构、MotionRuntime 与配置职责 |
| [三进程运行架构](three_process_runtime.md) | `ros2_gateway`、`motiond`、`mujoco_backendd`、IPC 与 session |
| [ROS 2 接口契约](ros2_interface_contract.md) | topic、action、service、字段映射与 QoS |
| [RL 与 MuJoCo](rl_mujoco_runtime.md) | 单进程调试、控制频率与策略运行 |
| [blackW 模型与行为](blackW_模型与行为.md) | 16 关节模型、轮足策略、Event chain 与固定轮驱 |
| [故障、诊断与回放](fault_logging_replay.md) | 故障结果、诊断字段和 ReplayRobotIO |
| [当前状态](current_status.md) | 已支持功能与仍需场景验证的内容 |

## 参考工程

部分行为和模型参数来自以下工程基线：

- `../rl_sar`：策略观测、历史、动作和行为语义；
- `../real_robot`：MuJoCo 场景与历史交互行为；
- `../URDF`：机器人几何与关节限制。

实际运行参数以本仓库代码和 `configs/` 为准。

## 图表

架构图源文件位于 `diagrams/`。`.dot` 为 Graphviz 源文件，`.svg` 为文档引用的生成结果。
