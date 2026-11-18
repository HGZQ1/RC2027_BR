# 一块方块（rc2027_perception/block_detector，2026-10-04）
uint8 GROUND=0      # 地块 350 mm
uint8 SKY=1         # 天空块 200 mm
uint8 UNKNOWN=2     # 尺寸 / 高度对不上（两块挨着、机器人、杂物）

uint8 type
float64 x           # map 系中心
float64 y
float64 z_bottom    # 底面高度（map 系）
float64 yaw         # 模 90°，[-pi/4, pi/4)
float64 extent_a    # 看到的两个边长（m），检查用
float64 extent_b
uint8 layer         # 从台面数第几层（1 起）
uint32 num_points
bool center_reliable   # false：只看到一部分面，中心是估的
