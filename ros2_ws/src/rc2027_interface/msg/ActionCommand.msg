# 决策层 -> 电控的动作指令（0004 业务消息 ActionCmd 0x02 的 ROS 侧，草案见
# notes/模板/rc2027_interface_msgs草案.md）。动作码表见 config/action_codes.yaml。
# 串口桥接好以前，由 rc2027_decision 的 mock_action_executor 假装电控回 ActionState
uint8 action_id     # 动作编号，见 config/action_codes.yaml
uint8 cmd_seq       # 本条命令编号，0~255 循环；ActionState 原样带回
int32 param         # 动作参数，无则 0
