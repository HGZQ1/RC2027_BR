# 世界模型 /field_state（rc2027_decision/field_state，2026-10-06）
builtin_interfaces/Time stamp
SpotState[] spots          # spot_01..spot_10，按名字排
RegionState[] transfer     # 交接区最近一次看到的（原样转发，新旧看各自的 stamp）
uint8 held                 # 车上拿着几块（按 PICK / PLACE 数的，Retry 清零）
