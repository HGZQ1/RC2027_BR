# 0005：导航方案（自研点到点）、底盘（舵轮）

状态：已决定（2026-10-04，用户拍板）。控制律在理想全向车上单测通过；**ROS 节点和
Gazebo 联调都还没跑过**，参数全是起点值。

## 背景

助教 10-04 B 段仿真建图跑通后提出：项目里还没有任何导航/运动控制代码，要先定用
Nav2 还是自己写点到点控制器（0002 2026-10-04 条目）。用户第一反应是"用 Nav2，跟去年
一样"。主设计师核对去年两家代码后发现这个前提不对：

- **GAFA**（`decision_processor/waypoint_navigator.py`、`项目说明文档v5.0.md`）：
  一开始做了 Nav2（`rc2026_navigation/launch/nav2.launch.py`、DWB 控制器、
  `generate_field_map.py` 生成的代价地图），后来**代价地图"被废弃了"**，比赛主线
  换成自研 `WaypointNavigator`（读 TF 的 `map → base_link`，PID 直接算 `cmd_vel`），
  只保留了 Nav2 的 `NavigateToPose` action 接口给决策 `game_controller` 调用
- **华工**：完全没用 Nav2

用户得知后改成：**学 GAFA 的最终方案，因为要精度；用 C++ 写；主动轮用舵轮**。

## 决定

1. **底盘：舵轮**（全向，`cmd_vel` 的 `vx`/`vy`/`wz` 都用）。模块数量、位置、轮径
   还没定，`rc2027_bringup` 的 URDF 和 `robot_params.yaml` 先按四角 4 个模块占位，
   模块关节暂时是固定关节（没有东西发 `joint_states`，见 URDF 注释）。
   **对串口协议的影响**：0004 的 `CmdVel` 必须带 `vx`、`vy`、`wz` 三个量，不能只有
   线速度+角速度（那是差速的写法），舵轮解算（每个模块的转角和轮速）在下位机做

2. **导航：自研点到点 `point_navigator`**（`rc2027_navigation`，C++）
   - 控制律学 GAFA：先**只平移**到位置容差内（车头朝向不动），再**原地转**到目标
     朝向。不边走边转：全向底盘边走边转时，定位误差（尤其雷达外参误差）会随朝向
     变化掺进位置里，分开做更好调、更准
   - **接口沿用 Nav2 的 `NavigateToPose` action**（`navigate_to_pose`），跟 GAFA 一样。
     决策层只依赖这个接口：以后真要换成 Nav2 或者别的实现，决策代码不用改。只装
     `ros-humble-nav2-msgs`（消息定义），不装 Nav2 本体
   - 目标点直接是场地坐标（`map` 系），**不在导航器里做坐标换算**——GAFA 在导航器里
     用 `loc_offset`/`coord_mode` 做的那套换算，我们在 TF 里由 `field_localizer` 做了
     （0003 决定 8）。目标 `frame_id` 不是 `map` 一律拒绝
   - 控制的是 `base_link`（跟 `teach_point` 录点同一个坐标系，录的点直接能当目标）
   - 手动测试入口：`/goal_pose`（RViz "2D Goal Pose"）

3. **跟 GAFA 不一样的地方**（控制律在 `point_nav_controller.hpp`，纯逻辑有单测）：
   - **速度曲线**：GAFA 在减速区（`decel_distance` 以内）用
     `max_speed * dist / decel_distance` **替换**了 `kp * dist`，结果离目标 0.29 m 时
     给 0.97 m/s、0.5 m 时只给 0.2 m/s——越靠近越快，是加速度限幅兜住了才没出事。
     改成 `min(max_speed, kp*dist, sqrt(2*decel*dist))`：远处限速、中间按刹车距离
     减速、最后指数收敛
   - **转身时继续稳住 xy**（小增益、无最低速度）。GAFA 转身阶段完全不管位置，转完
     可能已经偏出容差；舵轮能同时平移+转，顺手纠正
   - 转身时被推开超过 2 倍容差回到平移阶段
   - 卡住检测简化成一个参数（`stuck_timeout` 内没靠近 `progress_min_delta`）；GAFA 是
     "3 次 × 3 s 没进展"，效果一样
   - **Retry 硬拦截**：订阅 `/field_localizer/retry_status`，`PAUSED`/`RETRY_FAILED`
     时立刻停车、中止当前目标、拒绝新目标。0003 决定 6 原本只要求决策层"自觉"停车，
     而 `cmd_vel` 是从导航器发出去的，在这里拦一道成本最低
   - 视觉伺服交接（GAFA 的第三阶段）还没做——没有视觉节点。以后加在"到达"之后
   - **每个目标自带到位门限**（2026-10-04）：第二个 action 入口 `navigate_to_point`
     （`rc2027_navigation_msgs/action/NavigateToPoint`），普通走点用参数的 3 cm，拿 / 放前最后一步用
     1 cm / 1°。到点后导航器不会自己再动，所以"到点 → relocalize → 严门限再发一次"由决策层 Flow 的
     `ALIGN` 来做

4. **语言**：用户定 C++。导航介于感知和决策之间，组长的分层规则（感知/通信 C++、
   决策 Python）没有直接覆盖，按用户的决定来

5. **仿真**：Gazebo Classic 没有现成舵轮插件，用 gazebo_ros 自带的 `planar_move`
   （直接设车体速度，理想全向底盘）。舵轮转向延迟、打滑都不模拟；插件把竖直速度
   每次设 0，上坡道/L1 平台不会像真车那样爬，平地没问题

## 参数（`rc2027_navigation/config/point_navigator.yaml`）

全是没调过的起点值。位置容差 3 cm、朝向容差 0.03 rad（约 1.7°），比 GAFA 的
5 cm / 0.1 rad 紧——"要精度"是这次选型的理由，但能不能做到取决于定位精度和
底盘响应，要在 Gazebo 和真车上实际调。理想全向车上从红方 BR 起点开到场地中间
约 10 m 用时 14.7 s，误差 1.7 mm / 0.03 rad。

## 待定 / 下一步

- Gazebo 里实际调参（`ground_truth_check` 能同时看定位误差，区分"定位不准"和
  "控制不准"）
- 路径：现在只有点到点直线，中间有障碍（L1 平台、柱子）要靠决策层排中间点。
  需要的话以后在导航器里加"按顺序走多个点"（`NavigateThroughPoses` 的接口）
- 舵轮模块数量/位置定了以后更新 URDF；串口桥能报模块角度后改 continuous 关节
- 视觉伺服交接
- 0004 `CmdVel` 字段确认带 `vy`（等电控）

## 相关文件

- `ros2_ws/src/rc2027_navigation/include/rc2027_navigation/point_nav_controller.hpp`——
  控制律和跟 GAFA 的差异说明
- `ros2_ws/src/rc2027_navigation/src/point_navigator_node.cpp`——ROS 节点
- `~/Documents/ros/广美ros/GAFA-Artlnnov.RC2026-main/.../decision_processor/waypoint_navigator.py`——
  GAFA 原版
- `docs/decisions/0002-session-roles.md` 2026-10-04 条目——原始讨论
