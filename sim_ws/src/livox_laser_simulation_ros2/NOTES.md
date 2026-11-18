# livox_laser_simulation_ros2 — vendor 说明

## 来源

- 上游：https://github.com/LihanChen2004/livox_laser_simulation_ros2
- 版本：`cee09dc9eea6e0a9822735bbc98c925441bbb019`（2023-10-19，`main` 最新，
  之后没有更新）
- 许可证：MIT（见同目录 `LICENSE`，版权人是上游的上游 Ricardo Casimiro /
  stm32f303ret6）
- 包名 `ros2_livox_simulation`（`package.xml` 里的名字），不是目录名

## 为什么从 LCAS 版换成这个版本（2026-10-04）

10-03 第一次 vendor 的是 `LCAS/livox_laser_simulation_ros2`（2025-05 还在维护、
带 Mid-360 网格）。B 段仿真跑起来后助教发现 Point-LIO 一帧点云都收不到：

```
$ ros2 topic info /livox/lidar -v
  livox_sim_plugin  sensor_msgs/msg/PointCloud2           PUBLISHER
  laserMapping      livox_ros_driver2/msg/CustomMsg       SUBSCRIPTION
```

LCAS 在 2025-05 把发布改成了**只发 PointCloud2**（README 没跟着改，还写着两种都发），
而 Point-LIO `lidar_type: 1` 只订 CustomMsg。同名话题、类型不同，DDS 不会连，
**也不报错**。10-03 调研只看了 README 和旧源码，没核对 LCAS 最新源码，所以选错了。
详细排查见 `docs/decisions/0002-session-roles.md` 2026-10-04 那条。

换版本前逐文件 diff 过两个版本：**除了 `src/livox_points_plugin.cpp`，头文件、
`livox_ode_multiray_shape`、`csv_reader`、`urdf/mid360.xacro`、包名全部逐字节相同**
（头文件里两版都声明了 `custom_pub`，LCAS 只是 .cpp 里不再创建它）。所以 xacro 接口
`xacro:mid360 name parent topic` 不变，`rc2027_sim_bridge` 一行不用改。

LCAS 额外的 `meshes/mid-360-scaled.dae` 没带过来：`mid360.xacro` 本来就没引用它
（雷达外观是个方块），带过来也用不上。

## 插件发什么

| 话题（`topic` 参数 = `livox/lidar` 时） | 类型 | 谁用 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Point-LIO（`lidar_type: 1`，跟真车一致） |
| `/livox/lidar_PointCloud2` | `sensor_msgs/msg/PointCloud2` | 只给 RViz 看原始点云 |

- 时间戳是**仿真时间**：10-04 起用 `world->SimTime()`（射线打出去的精确时刻）。
  上游原来用 `node_->get_clock()->now()`——是仿真时钟没错（gazebo_ros 节点被强制
  `use_sim_time=true`），**但只有 /clock 的精度（10 Hz，取整到 0.1 s）**，跟 IMU 对不上，
  见下面"为什么不是原样未改"
- CustomMsg 里 `line`/`tag` 没赋值（都是 0），`timebase` 10-04 起填了。查过 Point-LIO
  `preprocess.cpp` 的 `avia_handler`：`line < N_SCANS`、`tag & 0x30 == 0x00` 都能过，
  `timebase` 不读；没打到东西的射线 range 置 0，会被 `blind` 过滤掉
- `depend` 了 `livox_ros_driver2`（消息定义），`external_ws` 里已经有真实源码

## 为什么不是"原样未改"

1. **`CMakeLists.txt`**：`CSV_FILES`/`URDF_FILES` 两个列表从 7 种雷达型号删到只剩 `mid360`，
   对应的 6 份 CSV（加起来 150 MB 左右）和 xacro 没带进来
2. **`src/livox_points_plugin.cpp` 的 `OnNewLaserScans()`（2026-10-04，唯一一次改插件逻辑）**，
   所有改动处都标了 `RC2027 改动` 注释：
   - **时间戳**：`header.stamp` 从 `node_->get_clock()->now()` 改成 `world->SimTime()`。
     原来的 `now()` 是"插件节点最近收到的 /clock"，`/clock` 默认只有 10 Hz，时间戳被向下
     取整到 0.1 s 的整数倍（助教在容器里实测：`220.8`、`222.7`、`224.3`），比实际采集时刻
     最多晚 0.1 s；IMU 插件用的是精确仿真时间，两边差出来的这段时间在原地转 2 rad/s 时
     相当于 11°——到点后原地转身时定位抖、点云地图跟着漂就是这个原因
   - **`timebase`** 填上同一个时间（真驱动也填，Point-LIO 不读，填了跟真机一致）
   - **`offset_time` 置 0**：Gazebo 的射线是同一个仿真时刻一次性打完的（瞬时快照），一帧内
     本来没有时间差；上游用墙钟算"处理到这个点花了多久"，是电脑快慢不是扫描时间。查过
     Point-LIO：全 0 时 `lidar_end_time = lidar_beg_time`，整帧当成同一时刻处理，没问题
   - **PointCloud2 每个点重复两遍**的 bug 顺手删了一遍（只影响 RViz 看的那路，少转换一半点）

   - **每个点短 10 cm（2026-10-04 下午，助教用 Gazebo 真值 + 场地网格射线实测：实测 − 真实距离
     恒为 −10.0 cm，跟远近方向无关）**：`LivoxOdeMultiRayShape::Init()` 覆盖了 Gazebo
     `MultiRayShape::Init()`，只把 `<range><min>/<max>` 读进了**同名局部变量**；基类的
     `minRange`/`maxRange` 是 private 成员，默认 0 / 1000，一直没被设置。查了 gazebo11 源码：
     `GetRange() = GetMinRange() + ray->GetLength()`，`Update()` 每次把射线长度重置成
     `GetMaxRange() - GetMinRange()`。所以 `GetRange()` 实际是"从射线起点（0.1 m 处）量的长度"
     → 短 minDist；没打到的射线长度 1000 → 返回 1000。改法在 `OnNewLaserScans()`：
     `>= RangeMax()`（1000）当没打到，否则 `+ minDist`，再按 sdf 配置的 `maxDist` 截断。
     （改不了基类私有成员，所以在出点时补；**副作用：射线实际一直打到 1000 m，配置的 40 m 量程
     只在出点时生效**，10-04 上午为性能把量程从 200 降到 40 其实没减少射线长度）
     ICP 因此把正确位姿"修"偏 8 cm、Point-LIO 也吃的是"缩水"点云——之前测的 0.7% 漂移、偏心测试
     的常值 +0.52° 要重测。验证工具：`ros2 run rc2027_utils sim_lidar_range_check`（应为 0 ± 1 cm）

   为什么这次改源码而不是绕开：时间戳只能在插件里打，外面没法修；绕开的办法（把 `/clock`
   提到 1 kHz）只能把误差降到 1 ms，而且 `/clock` 消息量大十倍。改动只有十几行，都在一个
   函数里

## 已知的坑

1. **插件不发 IMU**。已在 `rc2027_sim_bridge` 的仿真 URDF 里加 Gazebo 自带的
   `libgazebo_ros_imu_sensor.so`，挂在雷达同一个 link 上，remap 到 `livox/imu`，200 Hz
2. **IMU 单位不一样**：真实 Livox IMU 加速度单位是 g（`mid360_inner.yaml` 里
   `acc_norm: 1.0`），Gazebo IMU 是 m/s²。已处理：仿真专用配置
   `rc2027_sim_bridge/config/point_lio_mid360_sim.yaml` 里 `acc_norm: 9.81`，
   **`satu_acc` 也要跟着乘 9.81**（3.0 → 29.4，10-04 漏了这个导致 Point-LIO 发散，
   底稿注释说它"跟单位无关"是错的）
3. ~~**逐点时间戳不真实**~~：**2026-10-04 更正 + 已修**。10-03 调研时以为 `offset_time`（墙钟算的
   处理耗时）"会造成运动畸变"，其实 Gazebo 射线是瞬时快照，本来就没有运动畸变；真正的问题在
   `header.stamp` 被 /clock 取整到 0.1 s。两处都改了，见上面"为什么不是原样未改"第 2 条。
   修完的检查：`ros2 topic echo /livox/lidar --field header.stamp` 不再全是 0.1 s 的整数倍
4. ~~**`/livox/lidar_PointCloud2` 里每个点都重复了一次**~~：上游 bug，2026-10-04 跟时间戳一起修了
   （只影响 RViz 看的那一路，不影响 Point-LIO）
5. 上游提示"编译有警告时再 build 一次"。（`.cpp` 原来用 `boost::chrono` 算 offset_time，
   10-04 改完已经不用了；`CMakeLists.txt` 里的 `find_package(Boost ... chrono)` 和链接没动）
6. `urdf/mid360.xacro` 默认 `laser_max_range=200`、`samples=40000`、`update_rate=10`，
   跟真机（约 40 m、每帧约 2 万点）不一致，而且 4 万条射线/帧是 gzserver 跑不满实时的
   大头。**2026-10-04 已处理，没改 vendor 文件**：`rc2027_sim_bridge/urdf/mid360_sim.xacro`
   是抄过去的宏，点数/量程做成参数（默认 20000、40 m），仿真 URDF 改用它；这个包只提供
   插件 `.so` 和扫描模式 CSV
7. **30° 倾斜安装**：仿真里没有驱动层"IMU 跟随外参旋转"的 patch（那个在
   `livox_ros_driver2` 里）。雷达和 IMU 都在 `livox_sim` 一个 link 上（2026-10-04 起这个 link 跟着
   `lidar_link` 前下倾 30°，之前是平装；Point-LIO 开机按重力方向对齐，不需要那个 patch），
   仿真配置 `extrinsic_T/R` 是零/单位阵
8. **`CMakeLists.txt` 写死了 `libprotobuf.so.9`、`libboost_chrono.so.1.71.0`**
   （Ubuntu 20.04 的版本号）。**不用修**：host 上用最小 CMake 工程验证过，CMake 会把
   带版本号的库名改写成 `-lprotobuf`/`-lboost_chrono`；LCAS 版同样写法 10-04 已在容器里
   编译通过。真正需要的是 `libboost-chrono-dev`，已在 `install_deps.sh`

**依赖**：`gazebo`/`gazebo_dev`/`gazebo_ros`/`gazebo_plugins` 都会被
`ros-humble-gazebo-ros-pkgs` 传递装上（`gazebo_dev` 再依赖 `gazebo11` +
`libgazebo11-dev`）。

## 怎么编译

`sim_ws` 不在镜像里构建，容器里手动编：

```bash
cd /workspace/sim_ws
colcon build --symlink-install
```

**从 LCAS 版换过来之后，第一次编之前先清掉旧产物**（包名没变，colcon 会复用旧的
build 目录，旧 CMake 缓存里还记着 `meshes` 的 install 规则）：

```bash
rm -rf build/ros2_livox_simulation install/ros2_livox_simulation
```

验证：`ros2 topic info /livox/lidar -v` 发布方和订阅方的 Topic type 都应该是
`livox_ros_driver2/msg/CustomMsg`。

## 可参考的完整工程

[LihanChen2004/pb_rm_simulation](https://github.com/LihanChen2004/pb_rm_simulation)
（MIT，2025-03 已归档）：ROS2 Humble + Gazebo Classic 11 + Mid-360 + IMU，
Point-LIO/FAST-LIO 都接好了。跟这个插件同一作者，这一系"发 CustomMsg"的实现
和 Point-LIO 是有人跑通过的。

## 接入情况

URDF Gazebo 部分、放车 + 建图 launch、teleop 在 `sim_ws/src/rc2027_sim_bridge/`，
用法见那个包的 README。
