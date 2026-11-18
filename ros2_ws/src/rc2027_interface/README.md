# rc2027_interface

上位机 ↔ 电控的业务接口（串口协议框架本身是 `auto_serial_bridge`，见 `docs/decisions/0004-serial-protocol.md`）。

- `msg/ActionCommand.msg`、`msg/ActionState.msg`：动作指令 / 动作状态（0004 的 ActionCmd 0x02 / ActionState 0x82）。
  2026-10-04 按 `notes/模板/rc2027_interface_msgs草案.md` 建的，给决策层 Flow 引擎用
- `config/action_codes.yaml`：动作码表，**全是占位**，等机械 / 电控定动作组

话题约定（决策层用）：
- `/action_cmd`（ActionCommand）：决策 → 电控
- `/action_state`（ActionState）：电控 → 决策；`cmd_seq` 对上、`state` = DONE / FAILED 才算这个动作结束

串口桥接好以前，用 `rc2027_decision` 里的 `mock_action_executor` 假装电控（收到 ActionCommand，
按 `duration` 等一会儿回 DONE）。

## 待做
- [ ] 业务消息写进 `auto_serial_bridge/config/protocol.yaml`（等电控确认 0004 草稿），桥接 `/action_cmd`、`/action_state`
- [ ] 动作码"单一数据源"：从 `action_codes.yaml` 生成上位机和下位机两份，避免去年"两边手动维护两份表"
- [ ] SystemStatus（电量 / 急停）
