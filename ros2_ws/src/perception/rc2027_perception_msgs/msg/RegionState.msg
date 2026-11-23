# 一个区域（建造点 / 交接区）里看到的方块
builtin_interfaces/Time stamp
string region           # spot_01..spot_10 / transfer_red / transfer_blue
bool visible            # 有没有看到这个区域
float32 coverage        # 区域里 5 cm 小格有点的比例
float64 surface_z       # 台面高度（map 系）
bool surface_measured   # false：看不到台面，用的预期值
# 下面三项是"区域中心那一摞"的摘要，给建造点用（交接区里看 blocks）
uint8 ground_layers     # 区域中心那一摞有几块地块（top_maybe_clipped 时是"至少"）
bool has_sky            # 那一摞有没有天空块
float64 top_height      # 那一摞最高点离台面多高（m）
bool top_maybe_clipped  # 最高点贴着雷达视场上沿，可能还有看不到的
uint32 num_points
string note             # 中文说明：看不全 / 有不认识的东西
Block[] blocks          # 区域里所有方块，离区域中心近的那摞在前，每摞从下往上
