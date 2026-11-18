# rc2027_sim_bridge

仿真胶水包（只用于仿真，不进比赛）：把真车的 URDF、Mid-360 仿真插件、
Gazebo 场地、Point-LIO 接到一起。

## 内容

| 文件 | 作用 |
|---|---|
| `urdf/rc2027_robot_sim.urdf.xacro` | include 真车 `rc2027_bringup/urdf/rc2027_robot.urdf.xacro`（一个字没改），外面加 Gazebo 全向运动插件 `planar_move`（舵轮的替身）、仿真 Mid-360（`ros2_livox_simulation` 的 `mid360` 宏，挂在 `lidar_link` 上）、IMU 传感器、真值 p3d、轮子零摩擦 |
| `config/point_lio_mid360_sim.yaml` | 仿真专用 Point-LIO 参数，以 `mid360_inner.yaml` 为底稿改 4 处（`acc_norm` 9.81、`extrinsic_T` 置零、`tf_send_en` 关、`satu_acc` 29.4），理由见文件头 |
| `config/point_navigator_sim.yaml` | 仿真降速（0.5 m/s、1 rad/s），只覆盖这两项 |
| `urdf/mid360_sim.xacro` | Mid-360 仿真雷达宏，从 vendor 的抄来，点数/量程做成参数（默认 2 万点/帧、40 m），关掉射线可视化 |
| `maps/prior_map_sim.pcd` | 仿真先验地图（MuJoCo 场地网格射线投射，助教生成；生成脚本 `maps/make_prior_map.py`）。10-04 整体 z + 2.5 cm 对齐 Gazebo 场地，复核中位 0.8 cm、无系统偏差 |
| `rviz/sim.rviz` | 固定帧 map，先验地图（灰）+ 实时点云（5 s）+ 车模型 + 导航目标 |
| `worlds/rc2027_sim.world` | 场地 world：模型照用 `robocon_2027_gazebo` 的，物理 500 Hz、关阴影、**场馆墙**（离中心 9 m、高 4 m）、`gazebo_ros_state` 插件 |
| `worlds/rc2027_sim_empty.world` | 同上但没有场馆墙（场地外什么都没有），对照用：爬楼梯时南北方向会退化 |
| `launch/sim_mapping.launch.py` | 场地 + robot_state_publisher（复用 `rc2027_bringup/display.launch.py`）+ 在 Start Zone 放车 + Point-LIO + 定位链路（`localization.launch.py`）+ 点到点导航（`navigation.launch.py`）+ RViz |

## 怎么跑

`sim_ws` 不在镜像里构建，在容器里手动编（`ros2_ws`/`external_ws` 镜像里已经编好，
`.bashrc` 已经 source）：

```bash
cd /workspace/sim_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch rc2027_sim_bridge sim_mapping.launch.py          # 红方 BR 位置（box_b）
```

另开终端开车：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

RViz 固定帧 `lidar_odom`，开车能看到 `/cloud_registered` 一点点拼出场地。

**看不到点云先查这个**：`ros2 topic info /livox/lidar -v`，发布方（插件）和订阅方
（Point-LIO）的 Topic type 必须都是 `livox_ros_driver2/msg/CustomMsg`。类型对不上
DDS 不会连，Point-LIO 也不报错（10-04 用 LCAS 版插件就是这样，已换版本，见
`livox_laser_simulation_ros2/NOTES.md`）。
GUI 卡的话用 GPU 版容器（`.devcontainer/gpu/`）。

## 设计上的几个取舍

- **Point-LIO 不发 TF**（`tf_send_en: False`），TF 链由定位链路补上
  （2026-10-04，0003 决定 8）：
  `map →(field_localizer) lidar_odom →(lio_to_base) base_footprint →(URDF) ...`。
  仿真里 `body_frame=livox_sim`（雷达、IMU 同一个 link）。不想起定位链路：
  `localization:=false`
- **真值**：仿真 URDF 里加了 Gazebo p3d 插件，发 `/ground_truth/odom`（车体
  `base_footprint` 在世界系的真实位姿；世界系 = 场地坐标 = `map`）
- **舵轮用 `planar_move` 代替**（2026-10-04，0005）：Gazebo Classic 没有舵轮插件，
  `planar_move` 直接按 `cmd_vel` 的 vx/vy/wz 设车体速度（理想全向底盘），轮子摩擦
  设 0 不跟它抢。不模拟舵轮转向延迟和打滑；插件把竖直速度设 0，坡道/L1 平台爬不上去。
  不发 odom TF（`publish_odom_tf: false`），`/odom` 话题照发。teleop 按住 Shift 横移
- **雷达 + IMU 在同一个 link、车头外伸、前下倾 30°**（2026-10-04 起；之前是平装在底盘顶中央）。
  雷达位置来自 `rc2027_bringup/config/robot_params.yaml`（真车同一份占位数），仿真 `livox_sim` 挂在
  `lidar_link` 上跟着倾斜；IMU 在同一个 link 上，Point-LIO 开机用重力方向对齐，仿真配置外参仍是零/单位阵。
  前倾后背后只看天，比平装更容易出现方向退化——这正是要提前在仿真里暴露的
- 雷达点数/量程按真机（2 万点/帧、40 m），用本包自己的 `mid360_sim.xacro`，见下面"性能"

## 验证定位链路（2026-10-04 加）

```bash
ros2 launch rc2027_sim_bridge sim_mapping.launch.py
ros2 run rc2027_utils ground_truth_check        # 另开终端，每秒打一行误差
ros2 run tf2_ros tf2_echo map base_link          # 或者直接看 TF
```

1. 车静止几秒，`field_localizer` 日志出现"已锁定"
2. `ground_truth_check`（按同一时刻比对）：**静止时**误差应该是一两厘米；开车绕一圈、
   原地转几圈，误差不应该随朝向变化。运动中的误差会大一些，另外看它打印的延迟
3. **故意让雷达偏心再来一遍**（雷达在车体正中间时，外参乘反了也看不出来）：
   ```bash
   ros2 launch rc2027_sim_bridge sim_mapping.launch.py xacro_overrides:="lidar_x:=0.2 lidar_yaw:=0.3"
   ```
   原地转圈时误差如果在 ±20 cm 之间来回摆，就是外参换算有问题
4. teach_point 现在应该能用了（查 `map → base_link`）
5. Retry：`ros2 service call /field_localizer/retry rc2027_navigation_msgs/srv/Retry "{zone: start}"`
   ——仿真里没法真把车搬回 Start Zone，先开回起点附近再调，看
   `/field_localizer/retry_status` 走 PAUSED → RESUMED、`ground_truth_check`
   误差约等于"车离 Start Zone 中心有多远"（因为锁定假设车正好在 zone 中心）

## 性能（2026-10-04）

助教实测：gzserver 一个线程跑满、RTF 0.56（Ultra 9 275HX）。Gazebo Classic 的物理和
CPU 射线传感器都在**同一个线程**，靠配置用不上多核，只能减负：

| 改了什么 | 原来 | 现在 |
|---|---|---|
| 雷达每帧射线数 | 40000（真机的 2 倍） | 20000（= 真机 20 万点/秒 ÷ 10 Hz） |
| 雷达量程 | 200 m | 40 m（场地对角线 17 m） |
| 射线可视化 | 开 | 关（**同时加了 `always_on=true`**，否则雷达不出数据，见下） |
| 物理步长 | 1 ms（1 kHz） | 2 ms（500 Hz），planar_move 理想模型不需要 1 kHz |
| 阴影 | 开 | 关 |

⚠️ **`always_on` 的坑**（fc903d0 踩的，下一个提交修了）：Gazebo Classic 的射线传感器只在
`always_on=true` 或者它的 gz 扫描话题有订阅者时才更新。vendor 原版没写 `always_on`，一直是靠
`visualize=true` 让 gzclient 订阅画射线才在跑——关掉可视化（或者 `gui:=false`）雷达就一帧
都不出，Point-LIO 卡在 `file opened` 之后不动、RViz 里什么都没有。

还慢的话：`gui:=false` 不开画面（RViz 照样能看点云），`xacro_overrides:="sim_lidar_samples:=10000"`
再减半。`gz stats` 看 RTF。真要用显卡做射线，得换 Gazebo Harmonic（`gpu_lidar`），是大迁移，
见 0002 2026-10-04 助教那条，暂时不做。

## Point-LIO 发散的修复（2026-10-04）

助教在 0002 里查到的三件事，一起改了：
1. **`satu_acc` 没跟单位换算**（头号原因）：Point-LIO 拿原始加速度跟它比，仿真单位
   m/s²，3.0 意味着 z 轴重力每帧都被当成"饱和"丢掉。改成 29.4（3 g）
2. **IMU 初始化混进了落地冲击**：Point-LIO 原来跟 spawn 同时起。现在 spawn 进程结束后
   再等 3 s 才起，spawn 高度从 0.1 降到 0.05
3. **雷达时间戳被 /clock 取整到 0.1 s**（助教后来查到的真正原因，原地转身时定位抖、
   地图跟着漂）：改了 Livox 插件，用 `world->SimTime()` 打时间戳，见插件 NOTES.md。
   导航仿真降速保留
另外 `lio_to_base` 加了看门狗，Point-LIO 停止输出超过 1 s 就打 ERROR。

**上坡仍然不要测**：planar_move 每步把竖直速度设 0，车会跟坡道硬顶、IMU 读到冲击，
这是仿真模型的限制。

## ICP 精修验收（2026-10-04 加，按助教在 0002 列的步骤）

`sim_mapping.launch.py` 默认已经带上先验地图（`prior_map_path:=''` 可以关掉对比）。

0. **先确认仿真雷达量的距离是对的**（10-04 踩过：插件每个点短 10 cm，ICP 被带偏 8 cm）：
   车停着跑 `ros2 run rc2027_utils sim_lidar_range_check`，应该打"通过"（实测 − 真实中位 0 ± 1 cm）
1. **开局精修**：启动后 field_localizer 先"已锁定"，车停着约 1～2 s 后打"ICP（锁定后自动精修）
   已修正：…"。车放在 zone 正中时**修正量应该接近 0**（1 cm 量级），RViz 里灰色先验地图和彩色
   实时点云应该重合
2. **故意放偏**：`spawn_dx:=0.1 spawn_dyaw_deg:=3` 启动（field_localizer 仍按 zone 中心锁定）。
   不开 ICP 时 `ground_truth_check` 静止误差约 10 cm / 3°；开 ICP 后应该回到 1～2 cm，
   日志里的修正量约等于这 10 cm / 3°
3. **跑远漂移**：开一段远路停车，记下 ground_truth_check 的静止误差，然后
   `ros2 service call /field_localizer/relocalize std_srvs/srv/Trigger`，看修正量和误差方向是否
   一致、修正后误差是否回到 1～2 cm
4. **负面测试**：`spawn_dx:=0.8` 启动——开局精修应该**拒绝**（修正量超 0.3 m），日志 WARN，
   保持锁定值。（这对应"start_box 给错了"：宁可报警不跳）
5. **开车时调 relocalize** 应该被拒绝（"车在动，停稳再修正"）；导航到点停稳以后再调就行
6. Retry 后看日志：RESUMED 那行带着 ICP 结果，误差应该比没有 ICP 时（约 6 cm / 1.2°）小

## 爬楼梯回放：测高度变化后的定位（2026-10-04 加，0003 决定 10）

**2026-10-04 第一次跑**：没有场馆墙时，上楼梯后定位往北多了 1.85 m（z、yaw 都对）——爬楼梯时
车头抬 31°，雷达看不到任何"朝南北"的面，南北方向退化，见 0003 决定 10。现在默认 world 有场馆墙；
想复现退化用 `world:=$(ros2 pkg prefix rc2027_sim_bridge)/share/rc2027_sim_bridge/worlds/rc2027_sim_empty.world`。

planar_move 爬不了楼梯，BR 的爬楼梯机构也还没图纸。回放模式下车关掉重力、去掉 planar_move，
由 `sim_climb_replay` 按贴着场地网格的 3D 轨迹（z、俯仰都有）直接设置位姿和速度，Point-LIO 和
ICP 照常跑。原理和局限见 `rc2027_utils/rc2027_utils/sim_check/climb_replay.py` 开头。

**Gazebo 场地西侧的地形**（对碰撞网格实测，红方；蓝方 x 取反）：x −3.9～−3.0 一条带，
南端是 **4 级楼梯**（y −2.725～−1.825，每级高 0.15 m、深 0.3 m，地面 0.05 → 0.65），往北是
平台（y −1.825～−0.8，0.65，往东直接连着 L1），再往北是约 10° 的**坡道**（y −0.8 → +2.7，
0.65 → 0.05）。L1 顶 0.65、L2 顶约 0.95（Gazebo 比规则坐标整体高 5 cm）。

```bash
ros2 launch rc2027_sim_bridge sim_mapping.launch.py replay:=true
# 等 field_localizer "已锁定" + 开局 ICP 修完，另开两个终端：
ros2 run rc2027_utils ground_truth_check
ros2 run rc2027_utils sim_climb_replay          # 约 52 s（仿真时间）
```

路线：起点 → 楼梯底 → **上楼梯**（0.15 m/s，俯仰约 −31°）→ 楼梯顶停 3 s → 上 L1 →
**L1 Retry Zone**（−1.8, 2.6，z 0.65）→ 原地转到**朝南**（L1 Retry 的摆法）停住。

**先确认 Gazebo 里的车真的在动**：`sim_climb_replay` 的日志每 2 s 打一行位置；Gazebo 窗口里车要
跟着走。没动的话它会打 ERROR（真值离指令 > 0.2 m），常见原因：`sim_mapping` 没带 `replay:=true`、
sim_ws 改完没 `colcon build rc2027_sim_bridge`（world 里没有 `gazebo_ros_state`，脚本会直接报
`/gazebo/set_entity_state 不存在`）。**RViz 里的车是"定位认为的位置"，不能用来判断 Gazebo 里车动没动**
（10-04 第一次测就是车没动、在地面上调了 retry l1，RViz 显示车在 L1）。看什么：
1. 爬楼梯过程中 Point-LIO 不发散（RViz 点云不乱飞、`lio_to_base` 没有看门狗 ERROR），
   `ground_truth_check` 的 xy / yaw / **z** 误差保持在几厘米以内（运动中会因为延迟大一点）
2. 到 L1 Retry Zone 停稳后：`ros2 service call /field_localizer/relocalize std_srvs/srv/Trigger`
   —— 应该接受、修正量小，静止误差 1～2 cm、z 误差 1～2 cm
3. **上楼后 Retry**：`ros2 service call /field_localizer/retry rc2027_navigation_msgs/srv/Retry "{zone: l1}"`
   —— 日志"已锁定：…z 0.600…"，然后 ICP 修掉 Gazebo 的 5 cm（dz 约 +5 cm），RESUMED；之后
   静止误差 1～2 cm、z 对得上。（改之前这一步会因为 yaw 是 null 直接失败，就算有 yaw，锁定
   也会低 0.6 m、ICP 拒绝）
4. 楼梯顶停的那 3 s 也可以手动 relocalize，看平台上配准好不好
5. **负面测试**：车还在地面上（不跑回放）时调 `retry {zone: l1}`——应该 **RETRY_FAILED**（ICP
   内点很低、拒绝），`/field_localizer/retry_status` 是 RETRY_FAILED，point_navigator 停着

蓝方：`--ros-args -p side:=blue`（`sim_mapping` 也要 `side:=blue`）。回放模式下别发导航目标
（没有 planar_move，cmd_vel 没人执行）。

## 试导航（2026-10-04 加）

```bash
ros2 launch rc2027_sim_bridge sim_mapping.launch.py     # 已经包含 point_navigator
```

1. 等 `field_localizer` 日志"已锁定"
2. RViz 左侧 Global Options 把 Fixed Frame 改成 `map`，工具栏点 "2D Goal Pose"，在
   场地上点一下拖出朝向——车应该先直线平移过去（车头不转），到了再原地转
3. 或者用 action（决策层以后就这么调）：
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
     "{pose: {header: {frame_id: map}, pose: {position: {x: -3.0, y: -3.0}, orientation: {w: 1.0}}}}" --feedback
   ```
4. `ros2 topic echo /point_navigator/status` 看阶段；`ground_truth_check` 同时开着，
   能分清"定位不准"还是"控制不准"（到点后 status 是 ARRIVED，但真值误差大 → 定位问题）
5. 导航中调 Retry，车应该立刻停、action 被中止

## 摆方块测雷达方块检测（2026-10-04 加）

```bash
ros2 launch rc2027_sim_bridge sim_mapping.launch.py start_zone:=l1   # 车直接放在 L1 Retry Zone（朝南）开机
ros2 launch rc2027_perception block_detector.launch.py sim:=true
ros2 run rc2027_utils sim_blocks                                     # 按场景摆方块（先删上次的）
ros2 run rc2027_utils sim_blocks --ros-args -p mode:=check           # 按 views 开过去看、跟真值比，最后打表
ros2 run rc2027_utils sim_blocks --ros-args -p mode:=clear           # 删掉
```

- 场景文件 `config/block_scenes/l1_red.yaml`：spot_03 完整的塔（每层错开）、spot_02 一块、L2 的 spot_05 一块、
  交接区"2 叠 + 1 单 + 1 天空块"。换场景：`-p scene:=<yaml>`
- `start_zone:=l1`：spawn 在 L1 Retry Zone（z = 0.65），field_localizer 开机就按 L1 Retry Zone 锁定，不用先回放爬楼梯
- RViz 里 `Blocks`（`/block_detector/markers`）：绿 = 地块、蓝 = 天空块、红 = 不认识；半透明 = 中心不可靠；
  区域上方的字是"层数 +S（有天空块）?（顶部可能出视场）"
- 方块是 static 的，车撞上去不会动（但车会被挡住）。views 里的过路点绕开了 L2 和它的楼梯
- 前倾 30° 的雷达正前方往上只看到 22°，背后几乎只看天：**要正对着区域、离完整的塔 2 m 左右**才看得全

## 搭两座塔端到端（2026-10-05 加）

```bash
ros2 launch rc2027_sim_bridge sim_mapping.launch.py start_zone:=l1
ros2 launch rc2027_perception block_detector.launch.py sim:=true
ros2 run rc2027_utils sim_tower          # 假电控 + 假 TR（场景 config/block_scenes/towers_plan2.yaml）
ros2 launch rc2027_decision flow.launch.py use_sim_time:=true mock_actions:=false \
  flows:=/workspace/ros2_ws/src/rc2027_decision/config/flows/two_towers_sim.yaml
ros2 service call /flow_runner/start std_srvs/srv/Trigger
```

- `sim_tower` 替代 mock_action_executor：PICK 把夹爪前面那摞方块"拿走"（从 Gazebo 删掉），PLACE 按**车的真值位姿**在夹爪前面重新摆
  （加 1 cm / 1° 随机误差，定位偏多少就真的放偏多少）；放的高度跟那里已有的层数对不上回 FAILED。交接区拿空、BR 离开以后摆下一趟
- 每次 PLACE 打一张表（每层中心偏差、投影离建造点 500x500 边的余量，负数 = 出界不得分），全放完打总表和用时
- 参数：`time_scale`（动作时长系数）、`place_sigma` / `place_yaw_sigma_deg`、`seed`、`tr_delay`

## 状态

**全部没跑过**（2026-10-03 写完，host 上没有 Gazebo/ROS）。在 host 上能做的验证：
sim xacro 用真实参数展开成功（单根树、`livox_sim` 挂在 `lidar_link` 下、两个传感器
都在 `livox_sim` 上；10-04 换舵轮后插件列表是 livox/imu/planar_move/p3d）；
仿真配置跟 `mid360_inner.yaml` 解析后逐项 diff 确认只差 3 处；launch 语法检查过。

第一次跑最可能出问题的地方：
1. `ros2_livox_simulation` 编译（上游说有警告就再 build 一次）
2. 车放下去后是否稳定（脚轮摩擦为 0、`z=0.1` 落下）
3. Point-LIO 初始化：车静止几秒让它完成重力对齐，再开车
4. 插件发的 CustomMsg 逐点时间（坑 3）——快速转弯时点云会拖影，属已知问题
