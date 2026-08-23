# MotionRuntime 的低频运动状态快照。
uint8 MODE_PASSIVE=0
uint8 MODE_GET_UP=1
uint8 MODE_STAND=2
uint8 MODE_RUNNING=3
uint8 MODE_GET_DOWN=4

uint8 SOURCE_NONE=0
uint8 SOURCE_GAMEPAD=1
uint8 SOURCE_NAVIGATION=2
uint8 SOURCE_REMOTE=3
uint8 SOURCE_TEST=4

uint8 mode
uint8 active_source
string behavior_name
string behavior_phase
string policy_name
string error_message
bool policy_ready
float64[3] command_limits
