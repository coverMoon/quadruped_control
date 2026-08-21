# 一次性运动请求的统一状态和终态结果。
uint8 ACCEPTED=0
uint8 REJECTED=1
uint8 RUNNING=2
uint8 COMPLETED=3
uint8 FAILED=4

uint64 request_id
uint8 state
string message
