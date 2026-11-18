# 电控 -> 决策层的动作状态（0004 业务消息 ActionState 0x82 的 ROS 侧）。
# 状态变化时立即发，执行中 10 Hz 重复
uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_DONE=2
uint8 STATE_FAILED=3

uint8 cmd_seq       # 对应哪条 ActionCommand
uint8 state
uint8 err_code      # state=FAILED 时有效
