# 故障注入、诊断日志与回放

## 1. 当前边界

故障注入仅存在于测试辅助 `FakeRobotIO`，不会进入 MuJoCo 或共享内存运行链路。
生产代码只增加固定字段的 `MotionDiagnostics` 快照；`MotionRuntime` 不分配日志缓冲、不访问
文件，也不等待日志消费者。

`ReplayRobotIO` 只读取历史 `StateFrame`。MotionRuntime 提交给它的 `CommandFrame` 只保存在
当前回放进程内，用于确定性对比，不会转发到 MuJoCo 或共享内存。

## 2. 故障结果矩阵

| 故障 | MotionRuntime | 活动 ModeResult | RobotIO 状态 | 最终命令 | 恢复条件 |
| --- | --- | --- | --- | --- | --- |
| 状态无数据 | Passive | Failed | Paused | 不生成新命令，旧命令等待过期 | 状态恢复后提交新请求 |
| session 改变 | Passive | Failed | Ready | 新会话 Disabled | 下一周期重新提交请求 |
| 未来时间戳 | Passive | Failed | Ready | 不生成新命令 | 时间恢复后提交新请求 |
| IMU 非法 | RL 回到 Passive | Failed | Ready | Disabled | IMU 恢复后重新起立并启动策略 |
| 关节 NaN/Inf | Passive | Failed | Ready | 不生成新命令 | 状态恢复后提交新请求 |
| 命令过期/拒绝 | Passive | Failed | Ready，reject 增加 | 旧命令在执行侧失效 | 更新序号和有效期后提交新请求 |
| IPC 断开 | Passive | Failed | Disconnected | 不生成新命令 | heartbeat 和状态恢复后提交新请求 |
| backend fault | Passive | Failed | Fault | 不生成新命令 | 后端完成明确 reset 后提交新请求 |
| policy forward 失败 | Passive | Failed | Ready | Disabled | 修复或重载策略后重新启动 |
| policy inference 超时 | Passive | Failed | Ready | Disabled | 推理恢复到 20 ms 截止时间内后重新启动 |

这些行为由 motion、IPC 和 MuJoCo 测试共同覆盖，不依赖 ROS 2。

表中的“状态无数据”和“IPC 断开”指已经确认的 RobotIO 结果，不包括共享内存 latest slot
恰好正在写入的瞬时锁竞争。后者会在同一 backend startup/session 内复用最后一帧有效快照，
并继续由 500 ms heartbeat 时限约束；因此不会仅因一次并发读写就把活动模式锁存为 Passive。

## 3. 诊断字段

每次 `MotionRuntime::update()` 返回以下固定字段，低频调用方可按需抽样：

- startup、session、StateFrame、CommandFrame 和 effective command 序号；
- 状态数据年龄、丢帧数和命令拒绝数；
- 最近一次策略推理耗时；
- 同一输出中的 mode、behavior、policy、请求结果和错误信息。

`DiagnosticLogWriter` 额外把每个关节的实际位置、速度、力矩与生成命令的目标位置、目标
速度、KP、KD 和控制模式写到同一 CSV 行。`generated_command_applied` 根据执行侧回传的
effective sequence 标记该目标是否实际生效。

## 4. 只读回放

状态日志包含格式版本、机器人名称、显式关节名顺序和完整 StateFrame。加载时拒绝机器人
身份、关节顺序、帧维度、数值、时间或同会话序号不合法的数据。

当前运行入口尚未自动录制这种 `state_log.csv`；仓库已经提供格式读写器和测试生成样例，
但不能把 `/state/diagnostic` 导出的诊断 CSV 直接当作回放输入。准备符合格式的状态日志后，
回放命令如下：

```bash
.build/default/apps/replay/quadruped_replay \
  configs/robots/black.yaml \
  configs/controllers/black.yaml \
  state_log.csv \
  diagnostic.csv
```

程序按记录的单调时间驱动 MotionRuntime，并明确提示生成命令没有连接执行设备。相同日志
重复回放必须生成相同的命令序列，可用于修改前后回归比较。

当前回放入口只重放 `StateFrame`，不会加载 Torch policy，也没有记录或重放 ModeRequest 与
BaseCommand，因此尚不能复现一次完整的 GetUp/RL/策略切换操作链；未提供请求时
MotionRuntime 会沿默认 Passive 路径生成安全命令。
