# 本地备注（非上游内容）

来源：华工（SCUT）去年 `rc2026_ros2_ws` 仓库里的 `src/localization/Point-LIO`
（已经是 ROS2 Humble 移植版，不是港大官方的 ROS1 原版——CMakeLists 用
`ament_cmake`，`find_package(livox_ros_driver2 REQUIRED)` 链接的是 ROS2 版驱动）。
原样拷贝，没有改动。

## 依赖链

- `find_package(livox_ros_driver2 REQUIRED)`：必须先编译好 `external_ws/src/livox_ros_driver2`
  （已经在，见同级目录）。Dockerfile 里分两层处理，先编译 driver
  （`build_driver.sh`）、装好之后 point_lio 的源码才被 COPY 进去，
  再用 `build_point_lio.sh` 单独编译，顺序和隔离都在 Dockerfile 层面保证了
  （先 driver 后 point_lio）。
- `find_package(PythonLibs REQUIRED)`：cmake 配置期检查，需要 `python3-dev`
  （`install_deps.sh` 里已经有）。实际代码（`laserMapping.cpp`）没有真的
  `#include` matplotlibcpp.h，这个检查只是配置门槛，不影响运行时。
- 消息包：`nav_msgs` / `visualization_msgs` / `std_srvs`——这三个不确定之前
  apt 列表里是不是已经被别的包（`rviz2`、`robot_localization`）顺带装上了，
  为了不留侥幸，`install_deps.sh` 里已经显式加上。
- `include/ivox/ivox3d.h` 第 8 行 `#include <glog/logging.h>`——**代码里没声明这个
  依赖**（CMakeLists/package.xml 都没提 glog），编译时才会报 `fatal error:
  glog/logging.h: No such file or directory`。已经在 `install_deps.sh` 里加了
  `libgoogle-glog-dev`。只需要头文件：文件里唯一用到的 `LOG(ERROR)` 调用
  （约第 256 行）已经被注释掉了，不需要链接 glog 库、运行时也不依赖它。

## 配置文件

`config/` 下有好几份雷达安装角度对应的 yaml（`mid360_inner.yaml`/`mid360_outer.yaml`/
`mid360_renni.yaml`），**都是华工自己车的标定值，不能直接拿来用**——按之前讨论的结论，
我们这边：

- `extrinsic_R`/`extrinsic_T` 用 Mid-360S 官方手册值（单位阵 + 出厂平移），
  不用华工那几份里任何一份；
- 重力对齐这件事 Point-LIO 自己在 `laserMapping.cpp` 里做了（`Set_init`），
  不需要像 FAST-LIO 那样额外改源码；
- 真正要用哪份 config 跑，得等雷达装上车、定了侧装角度之后现场定，
  现在先别选，选错了比没有更麻烦。

## 没做的事

- 没有针对我们自己车的参数文件（之前说的 mid360 config），这个等硬件定了再加，
  不在这次"先让它能编译"的范围内。
- ~~编译验证进行中~~（2026-10-02 已编译通过并实机跑通，见下方"验证记录"）：第一次尝试缺 glog 头文件（见上面那条和调试记录），已修。
  还没确认补完 glog 之后能不能一路编译到底，`parameters.cpp` / `li_initialization.cpp` /
  `Estimator.cpp` / `laserMapping.cpp` 这四个文件当时是因为同一个缺头文件问题
  一起失败的，修完理论上应该一起过，但还需要实际跑一次确认。

## 验证记录

### 2026-10-02：编译通过 + 手持实机建图 ✅

- 补上 glog 之后编译通过（13:35 构建的镜像里已有 `/opt/external_ws/install/point_lio`），上面"编译验证进行中"那条可以划掉
- 运行方式（**注意驱动要用 msg_ 版 launch**）：
  ```bash
  ros2 launch livox_ros_driver2 msg_MID360s_launch.py   # xfer_format=1，发 CustomMsg
  ros2 launch point_lio point_lio.launch.py             # 默认 mid360 + inner → mid360_inner.yaml，自带 RViz
  ```
  yaml 里 `lidar_type: 1` 时，laserMapping.cpp 只订阅 `livox_ros_driver2::msg::CustomMsg`。如果驱动用 `rviz_MID360s_launch.py`（xfer_format=0，发 PointCloud2），Point-LIO 会收不到数据，而且**不报错**
- 条件：雷达手持，启动时平放在桌上；驱动外参 `pitch: 30`；临时直接用华工的 `mid360_inner.yaml`（extrinsic_R 为单位阵，extrinsic_T 是 Mid-360 官方偏移，acc_norm 1.0），没有改任何文件
- 结果：地图正常累积，绕屋一圈后轨迹基本闭合
- **观察到的重力对齐行为**：启动后 `lidar_odom`（世界系）是水平的，房间地图也是水平的；`body`/`lidar_link` 的坐标轴一启动就倾斜约 30°。原因是驱动按 30° 转动了点云和 IMU，Point-LIO 初始化时按 IMU 测到的重力方向把世界系对齐到真实的竖直方向。所以：
  - **地图是否水平，不取决于驱动外参 pitch 填得对不对**，Point-LIO 会自动按重力摆正
  - 外参填错，表现在 **body 系的姿态上**：车停在水平地面上时，body 坐标轴不竖直（roll/pitch 不为 0）
  - 装车后可以用这一点检查外参：车停在平地上启动 Point-LIO，body 坐标轴应该是竖直的
