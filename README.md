# RC2027 ROS2 算法工程

BR 机器人的雷达定位、点到点导航、方块检测、世界模型与规则决策工程。
本说明按 2026-11-19 本地源码核对：HEAD 为 `e7686d9`，全场规划代码基线为 `8c231d2`。
当前已完成主要算法与仿真链路，下一阶段进行真车和电控联调。

- 组长及其他 AI 请先读 [系统联调与 AI 对接文档](docs/INTEGRATION.md)。其中包含接口约定、实现位置和必要设计原因。
- 上传范围、依赖补齐与历史排除见 [上传说明](docs/UPLOAD.md)。
- 原 `ARCHITECTURE.md` 和部分包 README 存在早期状态，判断实现情况以本说明、对接文档及源码为准。

## 目录及职责

- `ros2_ws/src/`：项目 ROS2 包，日常业务开发的主体。
  - `rc2027_bringup`：车体 URDF、雷达/相机安装参数、模型发布。
  - `rc2027_navigation`：Point-LIO 位姿换算、场地坐标锁定、先验地图配准、点到点控制，C++。
  - `rc2027_navigation_msgs`：导航 Action、Retry 与仿真重锁定服务定义。
  - `rc2027_perception`：车体/货仓点云裁剪、雷达方块及层数检测，C++。
  - `rc2027_perception_msgs`：检测结果、建造点及场地状态消息、检测服务定义。
  - `rc2027_decision`：Python Flow 执行引擎、世界模型、规则规划器、假动作执行器。
  - `rc2027_interface`：上位机动作命令/反馈消息及占位动作码。
  - `auto_serial_bridge`：串口帧、握手、心跳、可靠传输与协议代码生成框架；业务映射尚未填写。
  - `rc2027_utils`：外参标定、示教录点、真值比对、模拟供货/动作、上下楼回放等工具。
- `external_ws/src/`：Livox ROS2 驱动及 Point-LIO 源码。独立构建，避免每次业务修改都重编算法和驱动。
- `sim_ws/src/`：Gazebo 场地、场地几何资产、Mid-360 仿真插件、仿真 launch/参数/先验地图。
- `docker/`：Humble 开发镜像、Compose、依赖安装与构建脚本；`docker_deps/` 是本机依赖缓存。
- `.devcontainer/`：复用 Compose 的开发容器入口。
- `docs/rules/`：运行时场地几何数据，必须保留。
- `docs/field_data/`：示教点位；仿真与真场地应分别维护。
- `docs/decisions/`：技术决策与版本存档；上传只保留必要结论，排除长调试记录。
- `bags/`、各级 `build/`、`install/`、`log/`：本地录包及构建产物，不作为源码交付。

## 当前功能

已实现雷达与 IMU 输入、Point-LIO 里程计、标准 TF 链、已知起点/Retry 定位、静止时先验地图 ICP 精修；导航支持机器人坐标系 vx/vy/wz、到位门限、超时和卡住停车。

已实现雷达区域检测、方块中心/层数/天空块形状判断，多帧观测与自身拿放记录合并的世界模型，以及按现场货物和塔状态动态重选的全场规划。Flow 支持导航、校准、检测、动作反馈和子流程。

当前导航仍是**先平移、后旋转**，路线由预设绕行点组成。边走边转、连续路点衔接和动态避障列入后续开发。

串口框架已接入， `auto_serial_bridge/config/protocol.yaml` 的 `messages` 仍为空；动作码、真实机构及底盘速度映射待与电控定稿。抓放和 L2 上下层目前采用模拟执行，真实机构与五色石取放待接入。EKF 依赖已安装，未作为当前定位必经链路接入；RealSense 核心 SDK 有构建配置，相机 ROS 节点及视觉业务未集成。

## 定位和导航怎么实现

定位链路：`Livox 点云 + IMU → Point-LIO → lio_to_base → field_localizer → map 下车体位姿`。

TF：`map → lidar_odom → base_footprint → base_link → 雷达/IMU`。`lio_to_base` 用 URDF 外参把 IMU 位姿换算为车体位姿；`field_localizer` 在已知起点锁定场地变换，必要时用 small_gicp 对齐先验地图。关闭 Point-LIO 自带 TF，避免同一坐标系出现两个父节点。真实场地地图和实际安装外参仍需采集、标定。

导航计算 map 下位置误差，生成带减速和加速度限制的平移速度，再按当前 yaw 转到机器人坐标系输出 `/cmd_vel`；位置到达后调整 yaw 并小幅保持位置。导航采用自研控制器，并复用 `nav2_msgs` 的 Action 定义。

## 环境与第三方依赖

镜像基于 Ubuntu 22.04 / ROS2 Humble（`ros:humble-ros-base-jammy`），Python 3.10，C++/CMake/colcon。定位与感知用 C++，规则决策和离线工具用 Python。

主要依赖：Livox-SDK2、livox_ros_driver2、Point-LIO、small_gicp、Eigen、PCL、yaml-cpp、OpenMP、TF2、rclcpp/rclpy、nav2_msgs；仿真用 Gazebo Classic 11、gazebo_ros 和 Livox 射线插件。工具依赖 NumPy（锁定 1.26.4）、PyYAML、Open3D、Matplotlib、pyserial 等，安装清单以 `docker/scripts/install_deps.sh` 为准。

librealsense 默认源码构建，PyTorch 默认关闭。当前 Dockerfile 即使关闭 PyTorch 也执行其 wheel 目录的 COPY；新机器按上传说明准备本机依赖目录。GPU 渲染使用可选 `docker-compose.gpu.yml`，与 PyTorch 开关独立。

## 构建与启动

先补齐 [上传说明](docs/UPLOAD.md) 中不随源码分发的依赖及仿真资源，再在仓库根目录执行：

```bash
# 宿主机；使用 X11 图形显示时
xhost +local:docker
cd docker
docker compose build br
docker compose up -d br
docker compose exec br bash
```

容器内项目工作区是 `/opt/ros2_ws`，外部工作区 `/opt/external_ws`，完整仓库挂在 `/workspace`。业务源码同时挂在 `/opt/ros2_ws/src`，项目包统一在 `/opt/ros2_ws` 构建。

```bash
# 容器内，增量更新项目包
source /opt/ros/humble/setup.bash
source /opt/external_ws/install/setup.bash
cd /opt/ros2_ws
colcon build --symlink-install
source /opt/ros2_ws/install/setup.bash

# 仿真是独立工作区，当前镜像不构建它
cd /workspace/sim_ws
colcon build --symlink-install
source /workspace/sim_ws/install/setup.bash
```

单独启动定位导航仿真：

```bash
export ROS_DOMAIN_ID=2
ros2 launch rc2027_sim_bridge sim_mapping.launch.py start_zone:=l1
```

全场规划脚本（与单独仿真二选一）：

```bash
export ROS_DOMAIN_ID=2
bash /workspace/ros2_ws/src/rc2027_decision/scripts/run_two_towers_sim.sh
```

脚本会启动 Gazebo、定位导航、检测、模拟器、真值检查和规划器；输出汇总到终端，日志在 `/tmp/rc2027_sim_run`，下次启动会覆盖。脚本在专用测试容器中管理仿真进程的启动与清理。

真车启动入口待整合，当前按对接文档逐层联调。

## 版本与验证

当前全场版：`e7686d9` 的规划代码基线 `8c231d2`；空手重选修复 `4a50048`，顶部出视场拿取保护 `bfaf956`，共享区离开绕 L2 修复 `8c231d2`。全场版执行多塔任务。

短赛程单独保留在 `b51bc28`：南自己 `spot_07` 加一座 L2，历史仿真约 170 秒到“宣布去五色石”，不含实际取石、开场大楼梯或真实机构上下楼。

规划器与世界模型的 97 项测试在恢复时通过；最新绕行代码仍需完整仿真验收。本机未提交的仿真试验 F 参数为线速度 1.0 m/s、线加速度 2.0 m/s²、角速度 3.0 rad/s、线减速 1.6 m/s²。真车参数在实机联调时确定。

纯逻辑检查（仓库根目录，无需启动 ROS/Gazebo）：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=ros2_ws/src/rc2027_decision \
  python3 -m pytest -p no:cacheprovider \
  ros2_ws/src/rc2027_decision/test/test_planner.py \
  ros2_ws/src/rc2027_decision/test/test_field_model.py -q
```
