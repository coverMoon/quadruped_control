# RobotIO 后端的低频连接和帧统计快照。
uint8 STATE_DISCONNECTED=0
uint8 STATE_READY=1
uint8 STATE_PAUSED=2
uint8 STATE_FAULT=3

uint8 state
uint64 latest_state_sequence
uint64 latest_command_sequence
uint64 dropped_state_frames
uint64 rejected_command_frames
