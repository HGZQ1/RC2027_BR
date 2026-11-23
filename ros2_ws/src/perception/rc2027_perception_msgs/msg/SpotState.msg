# 一个建造点"现在是什么样"（rc2027_decision/field_state，2026-10-06）：
# 多次雷达观测（block_detector）+ 自己放过的（PICK / PLACE）合并出来的结论，不是某一次看到的
uint8 SOURCE_PRIOR=0    # 开局默认（规则：开局建造点都是空的），还没看过
uint8 SOURCE_SEEN=1     # 雷达看到、确认过

uint8 SKY_NONE=0        # 没有天空块
uint8 SKY_OURS=1        # 最上面那块是我们放的
uint8 SKY_OTHERS=2      # 有天空块，但不是我们放的（大概率是对方颜色；雷达看不出红蓝）

string region           # spot_01..spot_10
float64 x               # 建造点中心（map 系）
float64 y
uint8 ground_layers
bool has_sky
uint8 source
uint16 confirmations    # 跟现在这个结论一致的观测有几次
float64 age             # 秒：离最后一次一致的观测多久（SOURCE_PRIOR 时是离开局多久）
uint8 sky_owner
uint8 ours_mask         # 第 k 层是我们放的 → bit (k-1)
uint8 expected_total    # 刚放完、等雷达确认的总块数；0 = 没在等
bool obstructed         # 最近 5 s 里看到建造点上有不认识的东西（车 / 人 / 举着的方块），那几次没用
bool base_known         # 塔基（第 1 层地块）的实测位置，往上放要对着它
float64 base_x
float64 base_y
float64 base_yaw        # 模 90°
string note             # 中文：在等确认 / 看到过别的结果几次 / 被什么挡了
