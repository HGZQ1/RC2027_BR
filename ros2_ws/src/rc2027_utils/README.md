# rc2027_utils

标定脚本、调试工具、mock 节点。

## 雷达外参标定（calibrate_lidar_base.py）

用法、原理见 `rc2027_utils/calibration/lidar_extrinsic.py` 顶部的说明。

跟华工去年的 `calibrate_chassis_center.py`（2D 圆拟合，只能标水平偏移、
假设转轴严格竖直）不同，这版用全 3D 最小二乘（`p_i + R_i @ b = P`），
能在数据质量不够（比如只做了纯偏航旋转）时用奇异值自动给出警告，
而不是悄悄给一个不可信的结果。

- 核心数学 `lidar_extrinsic.py` 不依赖 ROS，单测已经在 host 上跑过
  （`python3 -m pytest test/test_lidar_extrinsic.py`，8 个测试全过，
  不需要容器/车）：验证了一般旋转下 b/P 精确可解、纯偏航旋转下竖直分量
  退化会被正确标记、四元数转旋转矩阵本身是对的。
- ROS 节点 `calibrate_lidar_base.py` 是薄壳，订阅 Point-LIO 的
  `/aft_mapped_to_init`（nav_msgs/Odometry），没在容器里跑过，
  等有车之后第一件事就是验证这部分。

## 示教录点（teach_point）

把车放到目标位置，按 j 开始
采样、k 停止并算平均位姿、`s <名字>` 存进点文件。照搬华工
`calc_mean_position.py` 的交互方式和角度圆均值数学（直接算术平均角度在跨
±180°边界时是错的，这部分原样保留），区别：

- 默认查 `map → base_link`（这个项目的架构要求目标点是车体位姿，不是雷达），
  不是华工那边 `lidar_odom → lidar_link`
- 多了 `s <名字>` 命令，把结果存进 yaml 文件，不是只在终端打印看一眼
- **点文件按场地 + 红蓝方分开**：默认存到
  `/workspace/docs/field_data/<site>_<side>.yaml`（参数 `site` 默认
  `training`、`side` 默认 `red`），比如赛场蓝方就是
  `ros2 run rc2027_utils teach_point --ros-args -p site:=venue -p side:=blue`。
  2026-10-03 组长提醒的：示教点是现场实测、每换一个场地都要重录的数据，
  跟 `docs/rules/` 里"规则书翻译出来"的数据性质不同，不该混在一起；分文件
  方便赛场上快速切换。想完全自己指定路径就传 `points_path`（目录不存在
  会自动建）

- `rc2027_utils/teach_point/pose_stats.py`：纯逻辑（圆均值 + 读写点文件），
  不依赖 ROS，单测在 host 上跑过（`python3 -m pytest test/test_pose_stats.py`，
  8 个测试全过，包括角度跨边界的情况）。
- `rc2027_utils/teach_point/teach_point_node.py`：ROS 节点薄壳。10-04 用静态 TF
  冒充车位姿冒烟测试通过。`map → base_link` 这条链 10-04 已经接通（0003 决定 8，
  `rc2027_navigation` 的 lio_to_base + field_localizer），仿真里待实测。

## 仿真定位误差检查（ground_truth_check）

仿真专用：每秒打印 `map → base_footprint`（定位链路算出来的）和 **同一时刻** 的 Gazebo
真值（`/ground_truth/odom` 历史按 TF 时间戳插值）的误差，另外单独打印延迟和车速。
2026-10-04 之前是"最新 TF 比最新真值"，把 0.1～0.2 s 的定位延迟也算成了误差（运动中
差 5～10 cm、方向总落后于运动方向），助教指出后改的。看精度主要看"静止"那几行。用法和怎么判断结果见
`sim_ws/src/rc2027_sim_bridge/README.md`"验证定位链路"。误差计算是纯函数
`sim_check/pose_error.py`（单测 `test/test_pose_error.py`），节点本身是薄壳。

## 五色石（mustika.py）

交接区里认球，不走方块检测。半径 0.10 m（场地碰撞体）。只看见朝向雷达的那一侧，
用已知半径把球心从表面推进去。天空块直径也大约 0.20 m，靠残差分开，不靠外接尺寸。
纯函数，单测 `test/test_mustika.py`。

## 上 L2（sim_l2_hop）

仿真底盘上不了 L2 楼梯，机构也还没有图纸。先用和 `sim_climb_replay` 同一个
`set_entity_state`，把车直接放到刚跨上的 L2 台沿（蓝方镜像），不浮在楼梯上，也不靠中央柱。
上楼时会调 `/field_localizer/floor_change`（data=true）把高度门限从 0.10 m 放到 0.40 m，
否则重定位过不了一层楼。下楼时把 data 设成 false 收回。
车到位之后调 `/field_localizer/relatch`：重置 Point-LIO，按已知落点锁定。
不要在瞬移之后直接 `relocalize`，水平会差将近 1 m，0.3 m 的门限会拒。`sim_mapping`
要 `replay:=true`。落点是纯函数 `sim_check/l2_hop.py`（单测 `test/test_l2_hop.py`）。
图纸来了再换成沿楼梯的轨迹。

## 爬楼梯回放（sim_climb_replay）

仿真专用：测高度变化后定位还准不准（BR 上楼梯、上 L1 以后的 ICP 和 Retry）。仿真底盘爬不了
楼梯，就按贴着 Gazebo 场地网格的 3D 轨迹直接设置车的位姿和速度。轨迹生成是纯函数
`sim_check/climb_replay.py`（单测 `test/test_climb_replay.py`：楼梯上下的高度、俯仰、车体不切进
台阶、速度积分回位置），地形查询 `mesh_ray.HeightField`。用法见
`sim_ws/src/rc2027_sim_bridge/README.md`"爬楼梯回放"。

## 仿真雷达距离检查（sim_lidar_range_check）

仿真专用、一次性：取一帧仿真雷达点云，用 Gazebo 真值把每个点沿原方向往场地碰撞网格打射线，
比"实测距离 − 真实距离"，应该是 0 ± 1 cm。2026-10-04 用它查出插件每个点短 10 cm（ICP 被带偏
8 cm 的根因）。助教写的诊断脚本收进来的，射线求交拆成纯函数 `sim_check/mesh_ray.py`（单测
`test/test_mesh_ray.py`）。`ros2 run rc2027_utils sim_lidar_range_check`

## 待做
- [x] mock 下位机（`mock_stm32/`）—— 没有沿用华工的 `mock_stm32_ack.py`（纯
      Python 手写帧格式，会和真实固件实现不一致）；改用编译 `auto_serial_bridge`
      生成的 C 协议代码（原样打算烧进 STM32 固件的那份），协议只有一份权威
      来源。握手/心跳框架部分已经写完并用虚拟串口测试通过，业务消息等电控
      协议定了才能填，细节见 `mock_stm32/README.md`
- [x] 雷达外参标定脚本（calibrate_lidar_base.py）—— 代码和纯数学单测已完成，
      ROS 节点部分和实机标定流程还没有用真实数据验证过
- [x] 示教录点脚本（`teach_point/`）—— 代码和纯数学单测已完成，ROS 节点
      默认配置依赖的 TF 链路还没接通，细节见上面一节
