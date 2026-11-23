# rc2027_perception

感知层（C++，组长定的分层）。两个节点：**`cloud_crop`**（"车体 + 货仓"裁剪框，2026-10-05）和**雷达方块检测** `block_detector`（2026-10-04）：一个模块管三件事——
建造点上叠了几层、交接区里找方块、放方块前的局部对齐（0002 10-04 定的）。相机只用来分天空块红 / 蓝，以后再加。

## 怎么做的

算法在 `include/rc2027_perception/block_detector.hpp`（纯 C++，不依赖 ROS，`test/test_block_detector_main.cpp` 合成点云单测）：

1. **台面高度**：区域里预期高度 ±15 cm 的点按 1 cm 分格，点数是"尖峰"的那一格（方块侧面在高度上是均匀铺开的，不是尖峰）。
   看不到台面（从 L1 看 L2）就用预期值
2. **聚类**：台面以上 3 cm～1.6 m 的点按 xy 投影到 3 cm 栅格、8 邻接连通。一簇 = 一摞（叠着的投影在一起）。
   **两摞之间缝 ≥ 6 cm 才保证分得开**（TR 摆的时候留 10～15 cm）；比一块方块还大的簇再用 1.5 cm 细栅格分一次，
   隔 3～6 cm 的一般也分得开（真挨着的还是分不开，标 UNKNOWN）
   - 一簇在高度上空出 ≥ 8 cm 就拆开（举着的方块离一摞水平上很近时投影会连在一起）
   - 底离台面很高的簇（居中天空块的侧面比地块往里缩 7.5 cm，投影分开了）：并到 xy 25 cm 内、**顶正好接得上它的底**
     （±6 cm）、**中心落在它顶层脚印里**（偏 < 17.5 + 3 cm）的落地簇；接不上、而且它的底本来应该看得到（同一视线方向更近处看得到台面）的 → **悬空的东西**，
     标 UNKNOWN、不数层数、不当"区域中心那一摞"。底看不到（侧面往下只有 −7°）的不算悬空
3. **几层**：一摞的总高度对上 `n × 0.35 + (0 / 0.2)` 里最近的组合（±6 cm），天空块只能在最上面。对不上 = UNKNOWN
   （机器人、杂物）；矮于 15 cm 的（围栏、台面边）不管
4. **每层的位姿**：按高度切片，拟合正方形
   - 朝向：投影最集中的角度（转对了时每个面的点落在一条线上）。模 90°
   - 中心：雷达只看得到朝向自己的一两个面。每个轴上雷达在点的范围外 → 近边往里推半个边长；在范围内 → 范围中点
     （看到的不到 0.8 个边长就标"中心不可靠"）
   - 看到的边长比方块大 → UNKNOWN（两块挨着）
5. **顶部出视场**：最高点贴着雷达往上能看到的角度（前倾 30° 正前方 22°）→ `top_maybe_clipped`，层数报"至少"（不报 0，免得按空的去放）

合成点云（5 mm 噪声）单测：单块中心误差 < 1 cm、朝向 < 1°；完整的塔每层都分得出；只看到一个面也能定中心。

**仿真验收（2026-10-05，d762418，场景 `l1_red.yaml`，雷达离台面 0.23 m、前倾 30°）**：9 块全找到、没有多的；
中心误差中位 0.3 cm、最大 0.9 cm；朝向误差 ≤ 2°；建造点层数全对（含 L2 看不到台面的 spot_05）。
真车雷达会装得更高，看高处（塔顶、L2）只会更好；高度由 TF 自动带进来，`max_view_elevation_deg` 要按真车俯仰角改。

## cloud_crop：车体 + 货仓裁剪框

拿上的方块存在车后（机械说比底盘高 15 cm），一摞 700 mm 的顶部会进前倾雷达往后上方的视场，而且在 Point-LIO 的
`blind`（0.5 m）外——**跟着车走的点**会把 Point-LIO 往"车没动"拉，方块检测也会把它当方块。所以在 Point-LIO 前面加一道：

```
/livox/lidar（CustomMsg，驱动 / 仿真插件）→ cloud_crop → /cloud_crop/custom（CustomMsg）→ Point-LIO
                                                       → /cloud_crop/points（PointCloud2，雷达系）→ block_detector
```

- 盒子在 base_link 系，`config/cloud_crop.yaml` 的 `boxes`（扁平数组，每 6 个数一个）。现在是占位：底盘 + 货仓
  （底面 0.30、850 mm 高的理想化盒子，在车后半边）。等机械图纸
- 真车驱动只能发一种格式（Point-LIO 要 CustomMsg），`/cloud_crop/points` 顺便给方块检测一路 PointCloud2
- 真车驱动的 frame_id 是 `livox_frame`，TF 里没有 → `lidar_frame: lidar_link`
- 仿真 `sim_mapping.launch.py` 默认开（`cloud_crop:=false` 关），`sim_cargo:=true` 在车后挂一个 850 mm 的盒子测

## 能看清方块的最近距离（host 扫描，2026-10-05）

`tools/view_distance_sweep.cpp`：按 Mid-360 视场打射线（5 帧约 10 万点、1 cm 噪声），方块在车正前方 / 正侧面、
随机偏 ±3 cm、转 0～45°，每个距离摆 30 次，直接调 `detect_blocks`。"认对" = 层数、块数对，每块中心 < 2 cm、
朝向 < 3°（天空块 5°）、中心可靠。"稳定" = 从这个距离起 ≥ 90%（更远不掉到 80% 以下）。距离 = 车体中心到方块中心。

| 雷达离台面 / 前倾 30° | 单块地块 | 天空块 | 2 地一摞 700 | 地 + 天 550 | 完整塔 900 |
|---|---|---|---|---|---|
| 0.23 m 正面 | 0.70 | 0.70 | 1.40 | 0.95 | 1.90 |
| 0.23 m 侧面 | 1.00 | 2.00 | 0.95 | 1.00 | ~1.0（远处偶尔 80%） |
| 0.40 m 正面 | 0.70 | 0.65 | 0.95 | ~0.85（偶尔 80%） | 1.50 |
| 0.40 m 侧面 | 2.30 | 2.5 m 内不行 | 2.30 | 2.5 m 内不行 | 2.5 m 内不行 |
| 0.60 m 正面 | 0.80 | 0.90 | 0.80 | 1.10 | 0.95 |
| 0.60 m 侧面 | 2.5 m 内不行 | 不行 | 不行 | 不行 | 不行 |

- **正面看是主力**：前倾 30° 后正前方往下看到 −37°，脚下、方块下半截都看得到；限制只在**塔顶**（往上 22°），
  所以越高的摞要离越远，雷达装得越高越好
- **侧面看反而难，而且雷达越高越差**：侧面往下只有 −7°（前倾不影响正侧面），比雷达低的东西离近了只看得到顶上一截、
  台面也看不到；靠车尾那一头还被截得更多。雷达 0.6 m 时，侧面 2.5 m 内基本认不出方块
- 编译：`g++ -std=c++17 -O2 -Iinclude tools/view_distance_sweep.cpp src/block_detector.cpp -o sweep`，
  `./sweep [雷达离台面 m] [俯仰°] [每个距离几次]`（真车高度 / 俯仰定了重跑）

## 接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `~/detect` | `rc2027_perception_msgs/srv/DetectBlocks` | 停稳后看一眼某个区域（`spot_01`..`spot_10`、`transfer_red`、`transfer_blue`） |
| `~/regions` | `rc2027_perception_msgs/msg/RegionStateArray` | 定时扫（默认 1 Hz），只发看得到的区域。以后世界模型吃这个 |
| `~/markers` | `visualization_msgs/MarkerArray` | RViz：绿 = 地块、蓝 = 天空块、红 = 不认识 |
| `cloud_topic` | 订阅 | **雷达原始点云**（PointCloud2，雷达系；仿真 `/livox/lidar_PointCloud2` 每帧 20000 点），留最近 `frames` 帧，按每帧时间戳查 TF 转到 map。没去畸变，**停稳了再看** |

**不能用 Point-LIO 的 `/cloud_registered`**：那是 `feats_down_world`，0.2 m 体素降采样过（每帧约 1600 点、7 Hz），350 mm
的方块一个面只剩一两个点、高度被量化成 0.2 m 一档。2026-10-05 第一次仿真测就栽在这（量出 0.29 / 0.61 m 这种对不上的高度）。

区域从 `docs/rules/field_geometry.yaml` 读（`building_spots`、`transfer_area`；台面高度按 `levels` 判断 L2 / L1）。
参数见 `config/block_detector.yaml`；仿真 `sim:=true`（台面整体高 5 cm、sim 时间）。

```bash
ros2 launch rc2027_perception block_detector.launch.py sim:=true
ros2 service call /block_detector/detect rc2027_perception_msgs/srv/DetectBlocks "{region: spot_03}"
```

仿真测法（摆方块、自动开过去看、跟真值比）见 `sim_ws/src/rc2027_sim_bridge/README.md`"摆方块测雷达方块检测"。

## 局限 / 待做

- [x] 仿真里跑 `sim_blocks mode:=check`（见上）
- [ ] **真车点云来源**：livox_ros_driver2 给 Point-LIO 发的是 CustomMsg（`xfer_format: 1`），驱动一次只能发一种格式。
  要么加个 CustomMsg → PointCloud2 的小转换节点，要么这个节点直接订 CustomMsg
- [ ] 两块真挨着（缝 < 3 cm）现在只标 UNKNOWN，不拆
- 我们自己的 BR **放第 2 层的过程中**（夹爪举着方块从旁边移过来）就是"旁边举着"的姿态 → 流程上**放完、退开再看**，别在放的过程中调 detect
- [ ] 对方机器人站在区域里：高度对不上就是 UNKNOWN；正好 0.35 / 0.7 高的东西会被当成方块（以后加"多帧都在才算"）
- [x] 接进 Flow：`DETECT` 步骤按检测结果算拿取位姿（rc2027_decision，2026-10-05）
- [ ] 世界模型 `/field_state`
- [ ] 天空块颜色（相机）
