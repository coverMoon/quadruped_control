# 故障、诊断与回放

## 1. 故障处理

MotionRuntime 在状态、策略或命令链路出现运行错误时结束当前主动请求，并进入 Passive。

常见结果：

| 情况 | MotionRuntime | ModeResult |
| --- | --- | --- |
| 无有效状态 | Passive | Failed |
| session 改变 | Passive | Failed |
| StateFrame 数值或时间错误 | Passive | Failed |
| IMU / 关节数据非法 | Passive | Failed |
| CommandFrame 被拒绝 | Passive | Failed |
| IPC / backend 断开 | Passive | Failed |
| backend fault | Passive | Failed |
| policy forward 失败 | Passive | Failed |
| policy inference 超时 | Passive | Failed |

共享内存 latest slot 的一次锁竞争不会直接视为断开。RemoteRobotIO 会在同一 backend session 内复用最近有效状态，连接状态仍由 heartbeat 判断。

测试中的故障注入使用 `FakeRobotIO`，不会进入正式 MuJoCo 或 IPC 链路。

## 2. MotionDiagnostics

每次 `MotionRuntime::update()` 都会更新固定容量诊断数据，包括：

- startup / session；
- StateFrame、CommandFrame、effective command 序号；
- 状态年龄；
- 丢帧与命令拒绝统计；
- 最近一次策略推理耗时；
- mode、behavior、policy、请求结果和错误信息。

这些数据可以由低频日志或 ROS 2 状态接口抽样，不在高频控制周期内执行文件 IO。

## 3. CSV 日志

`DiagnosticLogWriter` 可记录：

- 关节实际位置、速度和估算力矩；
- 目标位置、速度；
- KP / KD；
- ControlMode；
- generated / effective command sequence。

`generated_command_applied` 根据执行侧回传的 effective sequence 判断生成命令是否真正生效。

## 4. ReplayRobotIO

ReplayRobotIO 从历史 StateFrame 日志驱动同一套 MotionRuntime，并把生成的 CommandFrame 保存在回放进程中。

回放命令：

```bash
.build/default/apps/replay/quadruped_replay \
  configs/robots/black.yaml \
  configs/controllers/black.yaml \
  state_log.csv \
  diagnostic.csv
```

状态日志包含：

- 格式版本；
- robot name；
- 有序 joint names；
- 完整 StateFrame。

相同输入日志应生成相同的命令序列，可以用于修改前后的回归比较。

当前 replay 只重放 StateFrame，不记录或重放 BaseCommand、ModeRequest 和 Torch policy。因此它适合检查基础 MotionRuntime 和命令生成的一致性，无法复现完整的 RL 操作过程。
