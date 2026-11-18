# 0004：下位机串口协议

状态：草稿待电控确认（2026-10-03）。框架和接入方式已定（见
`docs/decisions/0002-session-roles.md` 2026-10-02 条目），这份文件记录的是
**业务消息内容**的草稿，正式生效前要电控看过、同意。

## 背景

去年华工的协议**文档和代码不一致**：`docs/guides/stm32_packet_handling.md`
写的是回传 66 字节、ACK 在 byte 4-5；实际代码 `serial_packet.hpp` 是
`RECV_PACKET_SIZE = 10`，`mock_stm32_ack.py` 里 ACK 在 byte 2-3，三份各写
各的，而且没有校验字段。今年改用 `auto_serial_bridge`（配置驱动代码生成，
ROS 端和 MCU 端协议代码同源），从根上避免这类不一致——细节和接入决定见
0002 2026-10-02 条目、`ros2_ws/src/auto_serial_bridge/NOTES.md`。

## 决定（框架层面，已落地）

- 用 `auto_serial_bridge` v2.0.0 作为 `rc2027_interface` 的底层，已 vendor
  进 `ros2_ws/src/auto_serial_bridge`
- `mock_stm32`（`ros2_ws/src/rc2027_utils/mock_stm32`）编译生成的 C 协议
  代码做没有硬件时的联调，握手/心跳部分已验证
- 协议只有一份权威来源：`auto_serial_bridge/config/protocol.yaml`，
  改协议只改这一个文件，不手动维护文档和代码两份

## 业务消息草稿（待电控确认，还没写进真正的 protocol.yaml）

草稿文件：`~/Documents/ros/notes/模板/protocol_rc2027_draft.yaml`
（已用 `scripts/codegen.py` 验证过能正常生成代码），配套文档
`protocol_rc2027_draft_DOC.md`、给电控看的说明
[Docs](https://claude.ai/artifact/BppYWMCLr1HD61ZUjbSwu1)。

单位全部用 ROS 标准 SI（m、m/s、rad、rad/s，`f32` 直传，不再 ×1000 变整数），
坐标系 x 前、y 左、z 上、逆时针为正（REP-103），和上面 0003 的场地坐标系
约定一致。

六条业务消息：

| 消息 | 方向 | id | 说明 |
|---|---|---|---|
| `CmdVel` | tx | 0x01 | 底盘速度指令，导航约 50 Hz 发布 |
| `ActionCmd` | tx, reliable | 0x02 | 动作指令，带 `cmd_seq` |
| `PoseSync` | tx | 0x03 | 【待定】下位机需要全局位姿时才启用 |
| `WheelOdom` | rx | 0x81 | 编码器速度，约 100 Hz |
| `ActionState` | rx | 0x82 | 动作状态，状态变化时立即发 + 执行中 10 Hz 重复 |
| `SystemStatus` | rx | 0x83 | 电量/急停/使能标志，约 5 Hz |

**2026-10-04 补**：底盘定为舵轮（0005），`CmdVel` 必须是机器人系下的 `vx`、`vy`、`wz`
三个 `f32`（跟 ROS `cmd_vel` 的 `linear.x`/`linear.y`/`angular.z` 一一对应），舵轮
解算在下位机做。`WheelOdom` 同理回报 `vx`/`vy`/`wz`。导航器 `point_navigator` 默认
20 Hz 发 `cmd_vel`，上面"约 50 Hz"按实际调。

## 电控必做清单（框架不管，固件必须自己实现）

1. **失联停车**：超过 200 ms 没收到 `CmdVel` 就停车——MCU 自己不会发现断线，
   框架的心跳机制管的是"ROS 和 MCU 之间断没断连"，不是"多久没收到业务消息"
2. `ActionCmd` 按 `cmd_seq` 去重：`reliable` 的重传机制会在断线重连后补发，
   `on_receive_ActionCmd()` 可能被重复调用
3. 所有 `on_receive_xxx()` 回调里只复制数据、置标志，不做耗时操作（可能跑在
   中断里）
4. `serial_write()` 并发安全：推荐中断里只存字节，主循环里调用
   `protocol_fsm_feed()`
5. 动作开始/完成/失败时主动发 `ActionState`
6. **每次协议更新，整体替换生成文件，不要混用新旧版本**；业务代码写在
   电控自己的 `.c` 文件里，用同名函数覆盖弱定义的 `on_receive_xxx()`，
   **不要改生成文件本身**——生成文件里的 `USER CODE BEGIN/END` 标记只在
   上位机的 build 目录重新生成时保留，电控拷到 MCU 工程后下次整体替换会丢失，
   这是容易踩的坑，已经更正过一次（见 0002 2026-10-03 条目末尾）

## 待定 / 下一步

- 等电控看过草稿、确认或改动后，正式写进
  `ros2_ws/src/auto_serial_bridge/config/protocol.yaml`，替换掉当前的
  `messages: []` 占位
- `rc2027_interface/msg/ActionCommand`、`ActionState`、`SystemStatus` 三个
  自定义消息包还没建，草稿见 `~/Documents/ros/notes/模板/rc2027_interface_msgs草案.md`
- `PoseSync` 要不要真的启用，取决于下位机是否需要知道全局位姿（比如做
  本地避障用）
- `WheelOdom` 要接进 EKF 需要带协方差和时间戳，草稿里提到还需要另写一个
  小节点转换成 `TwistWithCovarianceStamped`
- 动作码表（`action_id` 对应的具体含义）等游戏任务细节定了再填，
  `ARCHITECTURE.md` 第七节仍然标着这项未决
