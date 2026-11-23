# mock_stm32

没有下位机硬件时，假装自己是 STM32，用来测通串口链路这一段。

## 为什么是 C 不是 Python

旧版（华工去年 `strategy/mock_stm32_ack.py`）是在 Python 里手写帧格式，结果和
真实固件的实现**不一致**（文档、Python mock、C 固件三份各写各的，见
`docs/decisions/0002-session-roles.md` 里助教那条记录）。

现在协议交给 `auto_serial_bridge` 统一生成，MCU 端的 `protocol.c` 本来就是
打算原样烧进 STM32 固件的可移植 C 代码——帧解析、CRC8 校验、握手/心跳自动
回包全部实现好了。`mock_stm32.c` 直接编译链接这份生成代码，只实现一个
`serial_write()`（把字节写到串口 fd）和一个读循环（把收到的字节喂给
`protocol_fsm_feed()`），协议细节完全没有在这个文件里重复一遍。好处：
协议只有一份权威来源（`auto_serial_bridge/config/protocol.yaml` +
`scripts/codegen.py`），改协议只需要改 yaml，这边不用跟着改。

## 用法

```bash
./build_mock.sh                       # 从 protocol.yaml 生成 protocol.h/.c 并编译
./mock_stm32 /dev/ttyUSB0 115200       # 真串口
./mock_stm32 /dev/pts/4 115200         # 虚拟串口（见下）
```

`generated/` 和编译出的 `mock_stm32` 可执行文件都不进 git（见本目录
`.gitignore`），改一次协议重新跑一次 `build_mock.sh` 就行。

## 没有串口硬件也能测：虚拟串口对

容器里装了 `socat`（见 `docker/scripts/install_deps.sh`），可以建一对互通的
虚拟串口：

```bash
socat -d -d pty,raw,echo=0,link=/tmp/ttyMCU pty,raw,echo=0,link=/tmp/ttyROS
```

`mock_stm32` 接 `/tmp/ttyMCU` 这头，`auto_serial_bridge` 的 ROS 节点
（`ros2 launch auto_serial_bridge serial_bridge_by_node.launch.py
port:=/tmp/ttyROS`）接 `/tmp/ttyROS` 那头，两边就能像接了真下位机一样
收发握手/心跳——`rc2027_decision`/`rc2027_interface` 这些上层节点的开发、
联调完全不需要等真的 STM32 板子。

开发时验证用的是 Python 标准库的 `pty` 模块（不需要装 socat）现场拉了一对
伪终端，发了一帧握手、一帧心跳，确认 `mock_stm32` 回包字节完全正确
（握手回传了生成代码里的 `PROTOCOL_HASH`，心跳原样回传 `count`）；
测试脚本是临时写的，没有进仓库。

## 业务消息还没法写

`auto_serial_bridge/config/protocol.yaml` 的 `messages` 现在是空的——今年
电控协议还没定（见 `auto_serial_bridge/NOTES.md`）。等协议定了：

1. 把业务消息加进 `../../auto_serial_bridge/config/protocol.yaml`
2. 重新跑 `./build_mock.sh`
3. 生成的 `protocol.h` 会多出新的 `on_receive_<Name>` / `send_<Name>` 声明，
   照着在 `mock_stm32.c` 的"业务消息回调"那一节实现即可

系统消息（Ack/Heartbeat/Handshake）已经是完整实现，不受协议是否定稿影响，
现在就能测。
