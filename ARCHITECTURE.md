# RC2027 算法组 — 架构草案

状态：**草稿 / 搭骨架阶段**。规则已定（ABU Robocon 2027），车和场地还没有，先把结构和 Docker 环境定下来，细节随开发推进。

对应讨论背景：去年（RC2026）代码被认为"屎山"，今年决定不留历史包袱——基础设施（驱动、定位算法、标定方法、解耦架构）可以借鉴广美/华工两套旧代码的经验，但游戏相关的内容（动作码、流程、坐标）必须全部按新规则重写。

---

## 一、总体分层

```
rc2027/
├── ros2_ws/        项目工作空间 —— 自己写的代码，天天改，进 git
├── external_ws/    外部工作空间 —— 驱动 + 定位算法，很少改，单独编译
├── sim_ws/         仿真工作空间 —— 没有真车真场地阶段的主战场
├── docker/         统一的开发/部署环境
└── docs/           规则派生的静态数据 + 关键设计决策记录
```

两个工作空间分离，是刻意的决定（参考 GAFA 去年的 `docker配置指南.md` 的做法）：
- `external_ws` 里是雷达驱动、Point-LIO 这类第三方/半第三方代码，改动频率低，不需要跟着项目代码一起反复编译，也不进自己的 git 历史；
- `ros2_ws` 只放自己写的业务代码，保持仓库干净、编译快。

容器启动时顺序是 `source external_ws` → `source ros2_ws`，见 `docker/scripts/entrypoint.sh`。

---

## 二、`ros2_ws/src` 下各包职责

| 包名 | 职责 | 对应旧代码参考 |
|---|---|---|
| `rc2027_bringup` | 顶层 launch、URDF、`robot_params.yaml` 式的集中参数表 | GAFA `rc2026_bringup` |
| `rc2027_navigation` | 定位链路：`lio_to_base`（Point-LIO 的 IMU 位姿换成车体位姿，发 `lidar_odom → base_footprint`）、`field_localizer`（锁定并发布 `map → lidar_odom`，Retry Zone 复位后重新锁定，先验地图 + small_gicp ICP 精修），见 0003 决定 8、9；`point_navigator` 点到点导航（学 GAFA 最终方案，`NavigateToPose` 接口，见 0005）；以后的 EKF 融合配置、地图。**C++**（定位算感知层，组长 2026-10-03 定的） | GAFA `rc2026_navigation`，但 ICP 触发逻辑要改成"从 Retry Zone 复位后触发" |

**分层语言（组长 2026-10-03 明确）**：感知层、通信层用 **C++**（rclcpp）；
决策层用 Python；赛前工具（标定、示教录点、生成先验地图）用 Python 不受约束；
视觉例外，仍用 Python。launch 文件不算"运行时代码"，一律 Python。
| `rc2027_decision` | 今年重写的 Flow/FlowRegistry 状态机引擎 + 新游戏的动作码表；世界模型 `field_state`（`/field_state`：每个建造点几层、谁放的） | 架构学华工 `stm32_control` 的 Flow 设计，内容全部重写 |
| `rc2027_interface` | 下位机串口协议、动作码表的"单一数据源"生成工具 | 参考 GAFA `auto_serial_bridge` / 华工 `stm32_control` 协议帧结构 |
| `rc2027_utils` | 标定脚本（雷达外参最小二乘法）、mock 下位机、调试工具 | 参考华工 `calibrate_chassis_center.py`、`mock_stm32_ack.py` |

命名统一按 **Point-LIO** 的话题/坐标系名字（`lidar_odom`、`body`、`aft_mapped_to_init`），不要混用 FAST-LIO 的 `camera_init`——这是这次讨论里明确要求的命名一致性。

## 三、`external_ws/src` 下放什么

```
external_ws/src/
├── livox_ros_driver2/   # 官方360S驱动 + 已移植的"IMU跟随外参旋转"patch
└── point_lio/           # 定位算法本体
```

`livox_ros_driver2` 已经是真实源码（Mid-360S 专用版 + 移植的 IMU 旋转 patch，见其 `NOTES.md`）。
它体积不大（1.2M）且带了我们自己的改动，**这份直接提交进 git**，不当成纯第三方代码对待——
不然谁 clone 这个仓库都拿不到这个 patch。`point_lio`（来自华工去年仓库，ROS2 版本）现在也放进来了，原样拷贝没有改动，
跟 `livox_ros_driver2` 一样直接提交进 git（见其 `NOTES.md`）。真正"大、没动过、不想进 git"的第三方东西放
`docker/docker_deps/`（已 gitignore），比如 Livox-SDK2、librealsense 源码、PyTorch wheel。

## 四、`sim_ws` —— 没有真车真场地阶段的主战场

| 包 | 内容 |
|---|---|
| `robocon_2027_gazebo` | 复用/改造 Gazebo 场地包（`YaoZA233/ABU-Robocon-2027-Gazebo`），作为"能跑起来的壳子" |
| `rc2027_field_assets` | 存放 MuJoCo 场地模型（`heartpain-kong/rc2027_field`）及其精确坐标数据，作为"权威数据源"（建塔点、Start Zone 等坐标从这里抄，不用手算） |
| `rc2027_sim_bridge` | 仿真胶水（我们自己的包）：带 Gazebo 插件的仿真 URDF（include 真车 URDF）、仿真专用 Point-LIO 配置、放车+建图 launch（2026-10-03 已写）。以后需要的话再加 GAFA 思路的"作弊式"真值里程计（把仿真真实位姿伪装成 `/Odometry`） |
| `livox_laser_simulation_ros2` | vendor 的 Mid-360 Gazebo Classic 仿真插件（LihanChen2004 版，包名 `ros2_livox_simulation`；10-04 从 LCAS 版换过来，LCAS 版只发 PointCloud2，Point-LIO 收不到），见其 NOTES.md |

第二阶段如果需要验证真实点云 SLAM 效果，再考虑给 Gazebo 接 Livox 仿真插件，优先级低于第一阶段。
**2026-10-03 第二阶段已经提前开始**：用户要求在 Gazebo 里用仿真雷达跑 Point-LIO，
`livox_laser_simulation_ros2`（LihanChen2004 版，包名 `ros2_livox_simulation`）已 vendor 进
`sim_ws/src/`；仿真胶水（带 Gazebo 插件的 URDF、仿真专用 Point-LIO 配置、放车+建图
launch）在 `sim_ws/src/rc2027_sim_bridge/`，见其 README。原计划的"作弊式"真值里程计
（mock_ground_truth_odom）还没做，需要时也放这个包。

## 五、`docs/rules` —— 规则翻译成的静态配置

把规则书（`Robocon_2027_Rulebook_v1-1.pdf`）和两个场地仓库里的精确数字，翻译成结构化文件，供 `rc2027_bringup` / `rc2027_navigation` 读取。现在就能填，不依赖硬件。

## 六、两台车（TR/BR）之间的隔离

规则 11.10：TR 和 BR 比赛中禁止互相通信。即使代码里没写通信逻辑，ROS2 的 DDS 默认走局域网组播，两车若在同一网段也可能"意外通到对方"。**在 Docker 层面用不同的 `ROS_DOMAIN_ID` 强制隔离**，而不是靠人工约定不写重叠话题名，见 `docker/docker-compose.yml`。

## 七、尚未决定、需要两人尽快拍板的事项

- [x] 场地坐标系原点选在哪个角 / 用场地中心（跟 MuJoCo 模型一致）？——已决定用
      场地中心，见 `docs/decisions/0003-field-localization.md`
- [ ] `external_ws` 的第三方源码用 git submodule 还是 Docker 构建时拉取？
- [ ] 今年的动作码表用什么形式做"单一数据源"——`rc2027_interface` 的串口协议
      本体已经用 `auto_serial_bridge` 的 yaml 驱动代码生成解决了（见
      `docs/decisions/0004-serial-protocol.md`），但 `action_id` 对应的具体
      含义表还是空的，等游戏任务细节定了再填
- [x] Retry 时怎么重新定位、谁来触发——已定：操作员笔记本命令触发，调用
      point_lio 自带的 `/point_lio_reset` 服务重新初始化定位，不整车重启，
      见 `docs/decisions/0003-field-localization.md`。决策侧怎么响应这个
      事件还是开放问题
