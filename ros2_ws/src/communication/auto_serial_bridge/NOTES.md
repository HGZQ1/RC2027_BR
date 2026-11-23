# auto_serial_bridge — vendor 说明

## 来源

- 上游：https://github.com/ConQU2026/auto_serial_bridge
- 版本：v2.0.0（2026-08-13，`package.xml` 里的 version 字段）
- 拉取方式：用户把源码放在 `~/Documents/ros/auto_serial_bridge-main/`（没有 `.git`，
  不是 `git clone` 来的，是下载的源码包），本次直接整份复制进来
- 许可证：Apache-2.0（见同目录 `LICENSE`）

## 为什么放在 `ros2_ws/src` 而不是 `external_ws`

这个仓库对 vendor 代码的约定是：`external_ws` 放"编译但不改动"的第三方源码，
`ros2_ws` 放会被我们自己改配置/业务逻辑的东西。

`auto_serial_bridge` 是一个"配置驱动的代码生成框架"：往 `config/protocol.yaml`
里写协议定义，`colcon build` 时会跑 `scripts/codegen.py` 生成 ROS 端 C++ 代码
和 MCU 端 C 代码（`protocol.c/.h`）+ 协议文档（`PROTOCOL_DOC.md`）。这个
`config/protocol.yaml` 的路径在它自己的 `CMakeLists.txt` 里是写死的
（`set(PROTOCOL_YAML ${CMAKE_CURRENT_SOURCE_DIR}/config/protocol.yaml)`），
不是通过 cmake 参数从外部传入的——这意味着要让它生成我们自己的协议，
就必须直接改这个包自己目录下的 `config/protocol.yaml`，这已经不算"原样未改"
了，所以按约定放进 `ros2_ws/src`，和 `rc2027_interface` 并列。

（以后如果想让它变成"真正不改动"的 vendor，可以给它的 CMake 打个小 patch，
支持 `-DPROTOCOL_YAML=` 从外部指定路径，那样就能挪进 `external_ws`，
`protocol.yaml` 留在 `rc2027_interface` 里。现在没必要做这个 patch。）

## 现状：`config/protocol.yaml` 还是上游自带的占位示例

上游默认的 `messages: []`（没有写任何业务消息，只有框架内置的
Ack/Heartbeat/Handshake 系统消息），**没有改成这个项目自己的协议**——
因为今年和电控约定的串口协议还没定下来，没法现在就编（细节见下面）。

这是故意的占位：先确认整条工具链（codegen → 生成代码 → colcon build）
能跑通，不猜协议内容。等协议定了，改这一份 `config/protocol.yaml`，
重新 `colcon build` 就会生成新的代码，不需要碰这个包的其他文件。

## 已知限制（助教实测记录，见 `docs/decisions/0002-session-roles.md`）

- 字段类型只支持 `uint8/uint16/uint32/int32/float`，**没有 int16**
- `reliable: true` 的 ACK 只表示"收到了"，不表示"动作做完了"；而且断连期间
  不会清空重试队列，重连后可能补发一条已经过时的指令——上层如果用 reliable
  发动作指令，要自己处理这种情况（比如带 cmd_seq 让 MCU 判断是否过期）
- 失联停车（MCU 多久没收到指令就自动停车）框架不管，心跳只负责"ROS 和
  MCU 之间断没断连"，这个安全逻辑要电控在固件里自己实现

## 还需要做的事

1. 和电控定一份串口协议文档（`docs/` 下新建，细节见 0002 里助教那条建议），
   作为协议的唯一权威来源
2. 协议定下来之后，把 `config/protocol.yaml` 换成真实内容
3. 容器里缺的 apt 依赖（`ros-humble-io-context`、`ros-humble-asio-cmake-module`、
   `ros-humble-rclcpp-components`）已经加进 `docker/scripts/install_deps.sh`，
   但完整 `colcon build` 还没在这个仓库里跑过，只在助教用一次性容器单独测试
   `auto_serial_bridge` 自己（15.6s 编译通过）时验证过，还没和 `ros2_ws`
   其他包放一起 build 过
