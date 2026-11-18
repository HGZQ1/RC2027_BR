# rc2027_navigation

定位链路（把 Point-LIO 的输出接成标准 TF 树，让所有节点都能直接查 `map → base_link`）
+ 点到点导航。C++（rclcpp）。

设计决定：定位见 `docs/decisions/0003-field-localization.md`（尤其决定 8：TF 链、雷达外参），
导航见 `docs/decisions/0005-navigation.md`。

## TF 链

```
map --(field_localizer)--> lidar_odom --(lio_to_base)--> base_footprint --(URDF)--> base_link --> lidar_link --> lidar_imu_link
```

| 节点 | 发什么 | 怎么算 |
|---|---|---|
| `lio_to_base` | `lidar_odom → base_footprint`（跟着 Point-LIO 里程计的频率） | `/aft_mapped_to_init`（Point-LIO 的 IMU 位姿）乘上 URDF 里 `base_footprint → body_frame` 的逆 |
| `field_localizer` | `map → lidar_odom`（20 Hz） | 车摆在已知位置（Start/Retry Zone）时锁定：`(map → base_footprint) · (lidar_odom → base_footprint)⁻¹` |

- `lidar_odom` 原点 = 开机/重置那一刻 **Point-LIO 的 IMU** 所在位置（重力对齐），
  不是车体中心
- **Point-LIO 必须关掉自己的 TF**（`publish.tf_send_en: false`），否则它发的
  `lidar_odom → lidar_link` 跟 URDF 抢父节点
- 雷达外参只在 URDF（`rc2027_bringup/config/robot_params.yaml` 的 `lidar` 节）里有
  一份，两个节点都从 TF 拿
- `body_frame`：真车 `lidar_imu_link`（Mid-360 内置 IMU，URDF 里有），仿真
  `livox_sim`

## 启动

```bash
ros2 launch rc2027_navigation localization.launch.py side:=red          # BR 默认 box_b
ros2 launch rc2027_navigation localization.launch.py side:=blue start_box:=box_a
```

前提：Point-LIO（TF 关掉）和 robot_state_publisher 已经在跑。仿真里
`rc2027_sim_bridge/sim_mapping.launch.py` 已经包含了这个 launch。

没有 Point-LIO 时 `field_localizer` **不发** `map → lidar_odom`（等着锁定，日志
会提示）。只想单独测它的话，手动补一条：

```bash
ros2 run tf2_ros static_transform_publisher --frame-id lidar_odom --child-frame-id base_footprint
```

## Retry

`/field_localizer/retry`（`rc2027_navigation_msgs/srv/Retry`，`zone`=`start`/`l1`）：
发 `PAUSED` → 调 `/point_lio_reset` → 等 2 s → 解除锁定，等重置以后的里程计（时间戳比
reset 前晚 1 s 以上）重新锁定到对应 zone 的位姿 → `RESUMED` / `RETRY_FAILED`。
状态在 `/field_localizer/retry_status`（transient local），格式 `"<状态> <zone>"`，如 `PAUSED l1`、
`RESUMED start`（`IDLE` 不带 zone；2026-10-04 加的 zone，Flow 引擎按它选 Retry 后的流程，只关心状态的取
第一个词）。决策侧要遵守的契约见 0003 决定 6。

## 先验地图 ICP 精修（0003 决定 9）

`field_localizer` 给了 `prior_map_path`（PCD，map 系）就开：把 Point-LIO 最近 10 帧
`/cloud_registered` 配准到先验地图，直接解出新的 `map → lidar_odom`。

- **自动**：每次锁定以后（开机、Retry）做一次；Retry 等它做完才发 RESUMED
- **手动**：`ros2 service call /field_localizer/relocalize std_srvs/srv/Trigger`，返回修正量或拒绝原因
- **车在动不做**（修正时 map → base_link 会跳）；**验收不过保持原值**：位置修正 ≤ 0.3 m、
  朝向 ≤ 5°、内点比例 ≥ 0.5 等，防止对称场地配到镜像位置
- 先验地图发在 `/field_localizer/prior_map`（RViz 里和实时点云叠着看对得齐不齐）
- 配准库 small_gicp（header-only，vendor 在 `third_party/small_gicp/`，见那里的 NOTES.md）
- 真车还没有先验地图（参数空 = 只靠锁定）

参数（都有默认值）：`icp_frames` 10、`still_window` 0.5 s、`still_max_move` 0.02 m、
`still_max_turn` 0.0175 rad、`icp_max_correction_xy` 0.3 m、`icp_max_correction_yaw` 0.0873 rad、
`icp_min_inlier_ratio` 0.5、`icp_map_voxel`/`icp_scan_voxel` 0.1 m、`icp_threads` 4、
`cloud_topic` `/cloud_registered`、`icp_on_latch` true、`icp_crop_half_size` 5.6 m（只用场地范围内的点，
先验地图只有场地，场外的墙/观众席会拉低内点比例；0 = 不裁）、
`icp_min_xy_info` 1.5（方向退化：每点平均约束强度低于它的水平方向不修，两个方向都低就拒绝，0003 决定 9）、
`icp_deadband_xy` 0.01 m / `icp_deadband_yaw` 0.0087 rad（水平修正都小于它就不改水平）、
`icp_deadband_z` 0.02 m（高度单独判断：水平在死区内时高度差超过它照样修）。
日志里每次 ICP 都打"平面约束（每点）弱 / 强"，按实测调阈值。

**"只有 z 错"自愈**（2026-10-06）：高度修正超过 0.1 m 一般是"换了一层"的错误解（内点低、水平和朝向也被拉歪），
拒绝。但水平修正 ≤ `icp_z_only_max_xy` 0.02 m、朝向 ≤ 1°、内点 ≥ `icp_z_only_min_inlier` 0.85 时是 Point-LIO 自己的 z 跑了
（仿真里车顶着方块抖，z 掉了 0.6 m），ICP 的高度可信，放行（≤ `icp_z_only_max_dz` 1.0 m，0 = 关），日志打 WARN。
不满足时拒绝信息里提示"建议 Retry"。
**收敛判据**：`icp_translation_eps` 3 mm / `icp_rotation_eps` 0.0035 rad（small_gicp 默认 1 mm / 0.1°，仿真底噪 4～5 cm 下
常 30 次迭代都到不了，约束够、修正小也报"没收敛"）。
**高度合理性检查**：停稳时车脚下 z 离最近的台面高度（0 / L1 / L2，`field_geometry.yaml` 的 levels，加 `z_check_offset`）
差超过 `z_check_tolerance` 0.10 m（0 = 关），每 5 s 打一条 WARN："定位 z 疑似跑了"。

开机锁定的 zone：`start_zone` 参数（`start` 默认 / `l1`，仿真直接放在 L1 Retry Zone 开机用）。

## 点到点导航 point_navigator（0005）

学 GAFA 去年的最终方案（自研，不用 Nav2 规划/控制），舵轮全向底盘：先只平移到位置
容差内（车头不转），再原地转到目标朝向（同时稳住 xy）。

```bash
ros2 launch rc2027_navigation navigation.launch.py             # 参数 config/point_navigator.yaml
```

| 接口 | 类型 | 说明 |
|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 沿用 Nav2 标准接口（只装了 `nav2_msgs`），用参数里的到位门限 |
| `navigate_to_point` | `rc2027_navigation_msgs/action/NavigateToPoint` | 决策层 Flow 引擎用：每个目标自带到位门限（≤ 0 用参数），结果带 `outcome`（ARRIVED/TIMEOUT/STUCK/PAUSED/PREEMPTED/CANCELED） |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz "2D Goal Pose"，手动测试 |
| `/cmd_vel` | `geometry_msgs/Twist` | 输出，机器人系 vx/vy/wz；空闲时不发（不跟 teleop 抢） |
| `/point_navigator/status` | `std_msgs/String` | IDLE/TRANSLATE/ROTATE/ARRIVED/TIMEOUT/STUCK/PAUSED |
| `/field_localizer/retry_status` | 订阅 | PAUSED/RETRY_FAILED（第一个词）时停车、中止目标、拒绝新目标 |

目标 `frame_id` 必须是 `map`（或空）。控制 `base_link`，跟 teach_point 录点同一个坐标系。
**卡住判定**：TRANSLATE 阶段 `stuck_timeout` 5 s 没靠近 2 cm；ROTATE 阶段转身时被推开（回 TRANSLATE）`max_pushes` 3 次，
或进 ROTATE 超过 `rotate_timeout` 8 s 还没到位（2026-10-06：贴着方块来回"到点 → 被顶开"，原来每次回 TRANSLATE 都重置计时，
只有 30 s 总超时兜底）。判卡住就停车、outcome = STUCK，Flow 失败停住。
两个 action 入口共用一个控制器，新目标顶掉旧目标。**到点后不会自己再动**：relocalize 修正以后要
"再发一次同一个目标、用更严的门限"才会把剩下的 1～3 cm 走掉（Flow 里的 `ALIGN`）。

## 代码结构

- `include/rc2027_navigation/field_geometry.hpp` / `src/field_geometry.cpp`：读
  `field_geometry.yaml`，按 side/zone 算位姿
- `include/rc2027_navigation/pose_math.hpp` / `src/pose_math.cpp`：3D 刚体变换和
  上面两个换算公式，不依赖 ROS/Eigen
- `include/rc2027_navigation/map_matcher.hpp` / `src/map_matcher.cpp`：读 PCD、small_gicp 配准、
  修正量和验收。单独编成 `map_matching` 库（要 Eigen + OpenMP）；`test_map_matcher` 用仓库里
  真实的仿真先验地图跑（偏 12 cm/3° 拉回到 0.3 cm，偏 0.8 m / 镜像都拒绝）
- `include/rc2027_navigation/point_nav_controller.hpp` / `src/point_nav_controller.cpp`：
  导航控制律（分阶段、速度曲线、限幅、超时/卡住），跟 GAFA 的差异写在头文件注释里
- 上面三个编成 `localization_core` 库，**host 上能单测**（`test/`，assert 风格，
  ctest 注册）：`test_pose_math` 里有"雷达偏心+转角""30° 俯仰安装"两种情况下
  车开走以后 `map → base_footprint` 等于真值的端到端检查；`test_point_nav_controller`
  用理想全向车闭环跑控制律
- `src/lio_to_base_node.cpp`、`src/field_localizer_node.cpp`、`src/point_navigator_node.cpp`：
  rclcpp 薄壳

## 验证状态

- 2026-10-03：field_localizer（旧版，直接发配置位姿）容器编译通过，`tf2_echo` 数值正确
- 2026-10-04：改成锁定 + 新增 lio_to_base + 新增 point_navigator。纯逻辑部分 host 上
  单测通过；**三个节点的新版本没在容器里编译过**（point_navigator 需要镜像里有
  `ros-humble-nav2-msgs`，要重 build 镜像），仿真验证方法见 `rc2027_sim_bridge/README.md`

## 待做

- [ ] 真车：外参数值、30° 倾斜时 `body_frame` 怎么取、雷达时钟和系统时钟（见 0003
      决定 8"还没解决"）
- [ ] 真车 bringup launch（Point-LIO 用 `mid360_inner.yaml` + 覆盖 `tf_send_en: false`）
- [ ] EKF 配置：轮速计只用速度，不用累积位置（去年 GAFA 的坑）
- [x] ICP 精修（small_gicp，0003 决定 9）——host 单测过，仿真待验收
- [ ] 真车先验地图（实地建图 + 对齐场地坐标）
- [ ] 场地地图：按 `docs/rules/field_geometry.yaml` 生成
- [ ] 导航：Gazebo 里调参、多点路径、视觉伺服交接（见 0005"待定"）
