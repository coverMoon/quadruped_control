# 当前状态

## 已支持

### 公共能力

- black 12 关节四足与 blackW 16 关节轮足；
- MuJoCo GUI / headless 仿真；
- Passive、GetUp、Stand、GetDown；
- TorchScript 强化学习策略；
- 运行时策略切换；
- 键盘、手柄和 ROS 2 `/cmd_vel` 输入；
- `ros2_gateway`、`motiond`、`mujoco_backendd` 三进程运行；
- 共享内存 IPC、session 与 heartbeat；
- 单元测试、MuJoCo 集成测试和 ROS 2 headless 端到端测试；
- 诊断 CSV、故障注入测试和只读回放。

### black

- flat、obstacle 策略；
- Retry；
- Event chain 接口，事件列表为空。

### blackW

- flat、obstacle、stair 策略；
- Retry；
- 10 段 Event chain；
- Bridge、Low-bar、Car 三种固定姿态轮驱行为；
- 腿位置阻抗与轮速度目标阻抗混合控制。

## 验证入口

```bash
./scripts/build.sh
./scripts/test/ctest.sh
./scripts/test/ros2_headless.sh
```

需要单独构建 ROS 2 接口时：

```bash
./scripts/build.sh --target command
```

## 待进一步验证

以下功能已有实现，仍值得继续做场景和长时间运行测试：

- blackW Event chain 的完整墙体接触与通过性；
- Bridge、Low-bar、Car 对应专用障碍场景；
- 不同实体手柄的连接、重连和轴映射；
- MuJoCo GUI 长时间运行及多显示器环境；
- 长时间日志采集、性能统计和绘图；
- 三进程 backend 参数与 `configs/simulation/*.yaml` 的进一步统一。

系统结构见 [系统架构](quadruped_control_architecture.md)，运行方法见根目录 [README](../README.md)。
