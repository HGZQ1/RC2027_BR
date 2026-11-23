# RC2027 系统联调与 AI 对接文档

核对日期为 2026-11-23。
本文件面向组长、电控和接手 AI，独立描述运行契约与待办，不依赖学习记录或聊天历史。根 README 负责环境和目录入口；消息定义、源码及参数文件是接口的最终依据。

## 1. 接手顺序

1. 先看根 README、本文件、`docs/rules/field_geometry.yaml`。
2. 查看 Git 当前提交、工作区差异及本机仿真参数。
3. 检查容器挂载、ROS 包实际查找路径、Python 模块实际文件及已安装配置。编译后重启相关节点，使安装副本和进程同步更新。
4. 按后文的联调顺序核对输入、定位、底盘和机构接口。

关键设计原因见第 7 节。

## 2. 系统数据链

定位、配准和运行时 TF 见[定位与点云配准](LOCALIZATION.md)。方块检测见[雷达方块识别](BLOCK_DETECTION.md)。

- 真雷达：`livox_ros_driver2` 发布 `livox_ros_driver2/msg/CustomMsg` 点云和 IMU；仿真由 Livox/Gazebo 插件提供对应输入。
- Point-LIO 融合点云与 IMU，输出 `/aft_mapped_to_init`（`nav_msgs/msg/Odometry`）、`/cloud_registered` 等。
- Point-LIO 发布 `lidar_odom → imu_link`，URDF 发布 `imu_link → base_link` 以及底盘下的车脚和传感器外参。
- `field_localizer` 将起点或 Retry Zone 的已知位姿与当前里程计相接，发布 `map → lidar_odom`；静止时可用先验地图 ICP 修正。
- `cloud_crop` 去除车体/货仓点，`block_detector` 将区域点云解析为方块、堆叠层数和可见性。
- `field_state` 把多帧检测与成功的拿放动作合并；规则规划器读世界模型和可拿货物，编译下一段 Flow。
- Flow 发导航目标、检测请求及机构动作，等待结果后重选；导航输出 `/cmd_vel`，机构输出 `/action_cmd`。

运行坐标采用米、弧度、秒；场地 map 几何以 `docs/rules/field_geometry.yaml` 为准。底盘坐标 x 向前、y 向左、z 向上，正 yaw 绕 z 逆时针；底盘联调时核对方向与安装。

## 3. ROS 接口契约

### 底盘导航

`/cmd_vel`：`geometry_msgs/msg/Twist`。机器人坐标系 `linear.x`、`linear.y` 单位 m/s，`angular.z` 单位 rad/s，要求底盘具备全向速度执行能力。当前控制器空闲时不持续发消息，电控负责速度指令超时停车、急停与限速。实际看门狗超时值、最大速度和制动能力待双方确认。

`/navigate_to_point`：`rc2027_navigation_msgs/action/NavigateToPoint`。

- goal：`pose` 为 PoseStamped，frame_id 是 `map` 或空；`xy_tolerance` 为米，`yaw_tolerance` 为弧度，非正值使用节点参数。
- result：`outcome` 为 ARRIVED / TIMEOUT / STUCK / PAUSED / PREEMPTED / CANCELED，另含剩余距离。
- feedback：当前位姿及剩余距离。只有 ARRIVED 可作为成功到位。

另有 `/navigate_to_pose`（`nav2_msgs/action/NavigateToPose`）与 `/goal_pose`（PoseStamped）用于兼容和人工测试。多个入口共享控制器，新目标可抢占旧目标；teleop 与导航切换使用。

位置闭环按 map 误差计算世界系速度，再转为车体系。先平移后旋转，转身时小幅稳住位置；含速度曲线、指令变化率限制、超时/无进展停车。绕障依靠预设路线。普通门限默认 3 cm / 0.03 rad，精细拿放常用 1 cm / 1°，按定位估计判断到位。

### 机构动作

`/action_cmd`：`rc2027_interface/msg/ActionCommand`：

```text
uint8 action_id
uint8 cmd_seq
int32 param
```

`/action_state`：`rc2027_interface/msg/ActionState`：

```text
uint8 cmd_seq
uint8 state       # IDLE=0, RUNNING=1, DONE=2, FAILED=3
uint8 err_code
```

回执通过 `cmd_seq` 匹配命令。串口 ACK 确认收包，DONE 确认动作完成并推进流程，FAILED 携带错误码。序号为 uint8 循环，双方需确认重传去重、旧回执处理及断线恢复。Flow 当前动作超时默认 30 秒，真实长动作需主线与电控共同确定。

`rc2027_interface/config/action_codes.yaml` 当前是占位约定：PICK=1（param=拿几块）、PLACE=2（param=放到第几层）、FLIP_SKY=3、CLIMB_L1=4、CLIMB_L2=5、PLACE_MUSTIKA=6、RELEASE_ALL=7、CLIMB_DOWN=8。动作编号在电控对接时统一定稿。CLIMB_L2/DOWN 目前由模拟换位支撑，真实机构尚未对接。

### 检测与世界模型

- `/block_detector/detect`：`rc2027_perception_msgs/srv/DetectBlocks`。请求 `region` 和 `frames`（0 使用节点设置），返回 `success`、`message`、`RegionState state`。
- `/block_detector/regions`：`rc2027_perception_msgs/msg/RegionStateArray`，持续区域观测。
- `/field_state`：`rc2027_perception_msgs/msg/FieldState`，包含 `spots`、`transfer` 和 `held`。

区域包含层数、是否有天空块、覆盖率、台面高度是否实测、顶部是否可能出视场和块列表。`top_maybe_clipped` 表示“顶未看全”，规划时使用对应的可见性保护。世界模型区分先验、确认次数、状态年龄、遮挡、塔基实测位置、待确认放置及天空归属。

`sky_owner` 结合自身动作记录和观测变化推断天空块归属；雷达提供形状信息。`held` 按成功动作记账，真实夹持结果由电控反馈。

### 启停、定位与 Retry

- `/flow_runner/start`、`/flow_runner/stop`：`std_srvs/srv/Trigger`，启动/停止流程。硬急停由电控执行。
- `/flow_runner/manual_retry`：Trigger；车已搬回配置的 Retry Zone 后调用。
- `/field_localizer/retry`：`rc2027_navigation_msgs/srv/Retry`，请求 `zone`（start/l1）和可选 `start_box`。
- `/field_localizer/retry_status`：`std_msgs/msg/String`，transient-local；PAUSED、RESUMED、RETRY_FAILED 等状态，可能带 zone。
- `/field_localizer/relocalize`：Trigger，停稳后先验地图精修，失败保留原变换。
- `/point_lio_reset`：Trigger，由 Retry 流程调用以重置里程计。
- `/field_localizer/relatch`：`rc2027_navigation_msgs/srv/Relatch`，给定 x/y/z/yaw，用于仿真换位后的已知位姿重锁定。
- `/field_localizer/floor_change`：`std_srvs/srv/SetBool`，上下层期间调整高度检查许可。

导航 STUCK 后停车检查。Retry 时将车搬回指定区域，重置里程计并锁定该区域位姿，收到 RESUMED 后继续。

## 4. 电控对接任务

框架源码在 `ros2_ws/src/communication/auto_serial_bridge`，当前 `config/protocol.yaml` 的 `messages: []`，只具备系统协议，尚未配置 `/cmd_vel`、`/action_cmd`、`/action_state` 的业务映射。

当前系统配置为串口 `/dev/ttyACM0`、115200，帧头 0x5A/0xA5、CRC8、握手版本检查、心跳 1000 ms、超时 3000 ms。接入时双方统一这些参数及业务消息编号。协议格式和生成方法以该框架源码及配置为准。

主线与电控还需共同完成：速度消息字段/坐标/单位与失联制动；动作码单一数据源；消息 ID、字段顺序及类型；错误码；动作忙闲、重复命令幂等性、急停与故障清除；机构完成条件；上下楼的定位交接；真实货仓数量/夹持反馈。先虚拟串口或台架验证，再接电机。

真实电控接入时关闭 `mock_action_executor`（`flow.launch.py mock_actions:=false`）；由真实电控替换 `sim_tower` 等模拟动作响应者，统一提供动作回执。

## 5. 部署核对与最小联调

Compose 当前 BR 的 ROS_DOMAIN_ID=2，TR=1；同车联调端需同域。两车使用独立 DDS 域；TR 服务预留，业务待接入。

宿主源码与容器路径：仓库 `/workspace`；业务源码 `/opt/ros2_ws/src`；外部源码 `/opt/external_ws/src`。项目构建只在 `/opt/ros2_ws`，仿真单独在 `/workspace/sim_ws`。仿真 overlay 不由 entrypoint 自动 source，需要单独加载。真车使用真实时钟，仿真节点使用 use_sim_time。

以下为容器内只读排查，运行时按需执行：

```bash
ros2 pkg prefix rc2027_decision
python3 -c 'import rc2027_decision.world.planner as p; print(p.__file__)'
ros2 node list
ros2 topic info /cmd_vel -v
ros2 topic info /action_state -v
ros2 run tf2_ros tf2_echo map base_footprint
```

宿主机另用 `docker inspect rc2027_br --format '{{json .Mounts}}'` 核对真实挂载。通过模块路径或哈希核对安装版本，统一加载 `/opt/ros2_ws` 的项目 overlay。

联调顺序：

1. 安装环境、确认无重复定位/导航/模拟动作节点。
2. 静止测点云、IMU单位和时间戳；核对 URDF 外参、Point-LIO 的原生 IMU TF 发布设置。
3. 锁定已知起点，人工量测 map 下车位，核对移动/转向与 TF 方向。
4. 架空或低速验证 vx/vy/wz、零速度、丢包/失联停车；再做单点导航及取消/卡住测试。
5. 不带导航单测机构命令的 RUNNING、DONE、FAILED、超时与重传。
6. 校验检测中心、层数、顶部可见性和夹持反馈；再接一趟取放，最后跑全场。

每次记录源码提交、未提交参数、安装副本、启动参数、时钟/ROS域、场景种子、关键日志和真值（若有）。仿真结果记录 Flow DONE/FAILED、放置余量和最终塔状态。

## 6. 当前版本与待验证事项

全场先按货物安排四座己方塔，再处理 L1 南北共享。短赛程仅安排南侧己方塔和一座 L2，以宣告去五色石为结束条件。

仿真 L2 用换位与定位 relatch；真实上下楼的动作时间和动力学在实机测定。定位包含台面高度拉回与恢复逻辑，运动验收同时记录原始里程计、修正/重锁日志与真值。

拿放预备段支持低速边走边对齐。实车联调时分别验证纯平移、纯旋转和组合运动，核对车身、外伸雷达和货仓的扫掠范围。当前仿真点是同一时刻快照，offset_time=0与模型一致；真实雷达另外验证扫描期间的运动畸变。

## 7. 必要设计原因

- 外部依赖、业务代码、仿真分工作区：减少重编译并避免把 Gazebo 依赖带进真车业务层。
- 统一 URDF 外参与 TF：避免多份安装偏移互相矛盾；Point-LIO 只保留一条父子链。
- 已知起点锁定加先验地图精修：把相对里程计接到规则场地坐标；ICP在静止时执行，并检查修正幅度、内点和方向约束，避免对称场地误配。
- 自研全向点到点控制：通过已知绕行点组织路线，控制器保持精简，路线按车身几何验证。
- 世界模型与规则纯逻辑分离：可以脱离 ROS 单测；分别保存观测、动作预期和实测确认。
- 整摞取放和减少重复观察：减少往返及停车；顶部不全、共享区变化仍需保护。
- 电控协议配置生成：两端应从同一份定稿定义生成，避免动作码和字段手工分叉；目前业务接入尚未完成。
