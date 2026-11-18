# 0003：场地坐标系、SLAM 坐标换算、Retry 定位

状态：核心部分已决定（2026-10-03）。坐标系结构、原点、坐标轴方向、
红蓝镜像轴、Retry 触发机制、决策侧响应契约都定了；2026-10-04 补了 TF 链
结构和雷达外参的处理（决定 8）、先验地图 ICP 精修（决定 9）；先验地图对齐时机、
ICP 算法选型这些要等硬件/先验地图就绪才能定的细节还留着，见下面
"待定/下一步"。

## 背景

助教和用户讨论后，把结论和详细推理整理进了
`docs/decisions/0002-session-roles.md` 2026-10-03 那条记录，推理过程在
`~/Documents/ros/notes/`（`概念/SLAM坐标到场地坐标.md`、
`概念/场地原点怎么选.md`、`模板/field_localizer草案.md`）。这份文件是
主设计师评估后的正式版本，取代 0002 里那条"待确认"的草稿状态，也取代
0001 末尾"待定"里"场地坐标系原点最终选哪个"那一项。

核心问题：Point-LIO 的坐标原点是"开机时雷达所在的位置"，不是场地上的固定点；
决策/导航要用的是规则书给的场地坐标。去年两家都没有解决好这个换算：
GAFA（广美）把换算放在决策节点内部（`waypoint_navigator` 的 `loc_offset`
参数），`map → odom` 发的是单位阵、不是真的场地坐标，而且起点这组数字散落在
`config.py`/`waypoint_navigator`/`map_relocalizer` 三处；华工没在 ROS 侧做
换算，SLAM 坐标直接发给下位机，场地点位写死在固件里。两家都没有把场地坐标
放进 TF 里广播给所有节点，细节见 `概念/SLAM坐标到场地坐标.md`。

## 决定

1. **`map → odom` 由 `rc2027_navigation` 里一个新节点 `field_localizer` 唯一
   负责发布**（动态 TF，约 20 Hz，符合 REP-105 的标准分工）。其他节点
   （决策、导航、RViz）一律只查询 `map → base_link`，不关心这个值怎么来的，
   也不自己做场地坐标换算。这是 notes 里说的"第三种方案"：既像 GAFA 那样
   能重定位时更新，又符合 ROS 惯例把场地坐标放进 TF 给所有人用。

2. **起始位姿渐进实现**：
   - 第一步（现在就能做，不需要硬件）：`field_localizer` 启动时从
     `docs/rules/field_geometry.yaml` 的 `start_zone`（按 launch 参数
     `side:=red/blue` 选）读配置位姿，直接发布 `map → odom`
   - 第二步（可选增强，待先验地图/硬件就绪后再做）：加 `/relocalize` 服务，
     用配置位姿当 ICP 初值，拿 Point-LIO 的 `/cloud_registered` 和预建的场地
     点云地图做匹配，通过质量检查（fitness 阈值 + 修正量上限，防止对称场地
     匹配到镜像位置）才更新，不通过就保持原值并报警
   - 起始位姿、Retry Zone 位姿**只写在 `field_geometry.yaml` 一处**，不像
     GAFA 那样散落在三个文件里

3. **场地坐标系原点 = 场地中心，坐标轴直接用 field.xml 的原生轴，不旋转**
   （2026-10-03 确认）。实测了 `field.xml` 里的网格数据（`start_red.obj`/
   `start_blue.obj`/`sky_blocks_*.obj`/`storage_*.obj`/`mustika_pillar.obj`
   的顶点坐标，不是凭规则书插图猜的）：
   - **x = 0 是红蓝对称轴**：红方 Start Zone 在 x∈[-5.5,-4.0]，蓝方在
     x∈[4.0,5.5]，L1 Retry Zone 红/蓝分别是 x∈[-2.15,-1.45] / [1.45,2.15]——
     这组数据主线和助教各自独立算了一遍（助教还精确到了 Start Zone 两个
     700mm 子格子各自的中心），结果完全一致，交叉验证过
   - **镜像公式**：`(x, y, yaw) → (-x, y, π - yaw)`，朝向要翻成 `π - yaw`
     不是简单取负——这是镜面反射对朝向的作用，和 GAFA 去年 `config.py`
     里 `_mw()` 的公式一致。**⚠️ 公式只管点位，不管车本身**：机构装在车体
     哪一侧是固定的，不会跟着镜像翻过去，"从哪一侧接近目标"这类动作不能
     直接套用镜像后的坐标，要在策略层单独处理
   - **y 轴方向自然对应游戏语义**：灵石基座（Mustika Pillar）在
     y≈+4.25（和规则书写的 "(0, +4.25)" 吻合），天空块区在 y≈-4.25、
     跟 Start Zone 同侧。考虑过把"开局朝向"（field.xml 里是 +y，机器人在
     Start Zone 面朝场内）转成地图 +x（REP-103 里 yaw=0 的习惯方向），
     但那样要把已经从 field.xml 抄的 10 个建塔点坐标跟着做一次 90° 变换——
     **最终决定不转**，保持和 field.xml 完全一致的坐标，代价是开局朝向对应
     yaw=90° 而不是 0°，换来的是不用对已经核对过的数据做二次变换、以后再从
     field.xml 抄新数据也不用记得"还要转一下"
   - 结论：**地图系 x/y 和 `field.xml` 完全一致**，x 轴正方向指向蓝方，
     y 轴正方向是灵石基座/储存区那一侧（开局时机器人面朝 +y）

4. **Retry 的基础原则：不整车重启**。物理上把车搬到 Retry Zone 这个动作，
   对 Point-LIO 来说是一次位置的不连续跳变（车在被搬动的过程中没有可供
   LIO 跟踪的连续运动），所以**定位这一层需要重新初始化**，但 ROS2 的
   其他部分（决策、导航、串口节点）不重启、不丢状态。`field_localizer`
   重新初始化定位后，用 Retry Zone 的配置位姿当初值，重新发布 `map → odom`
   （可选再做一次 ICP 精修）。对决策/导航/RViz 来说，`map → base_link` 这条
   链路全程没有中断——它们不知道、也不需要知道底下的 `odom` 换了一个新的。
   这是"不整车重启"在技术上能成立的原因：**不重启的是决策状态机和已经完成
   的任务进度，不是定位算法**。

5. **Retry 的具体机制（2026-10-03 确认）**：
   - **触发方式：操作员笔记本命令**，不做物理按钮。理由：零硬件改动、不用
     多走一条串口协议消息、不需要电控额外实现，赛场操作台本来就会有笔记本
     连着车的 wifi 盯场——工程风险最低。这是跟用户确认过的选择，物理按钮
     方案（经 STM32 `SystemStatus` 上报）作为备选记录在案，不是现在的实现
   - **"重新初始化定位"不是重启进程，是调用 point_lio 自带的
     `/point_lio_reset` 服务**（`std_srvs/srv/Trigger`，见
     `external_ws/src/point_lio/docs/RESET_SERVICE_USAGE.md`）——这是华工
     原版 point_lio 自带的功能，原样拷贝过来的，不是我们加的 patch。
     它内部设 `flg_reset` 标志，下一个处理循环清空地图、重置卡尔曼滤波器和
     IMU 处理器状态、重新走一遍初始化，**不需要杀进程重启、不需要重新
     `colcon build`/`ros2 launch`**，比"重启整个 Point-LIO 节点"轻得多。
     **这个服务目前只存在于源码里，还没有被实际调用验证过**，是有车之后
     要做的第一批验证之一
   - **操作员触发的那一条命令，完整做这几件事**（由 `field_localizer` 的
     一个服务统一编排，操作员只需要调一次）：
     1. 带上参数：这次用哪个 Zone（`start` 或 `l1`，对应规则 5.3.1 里 BR
        可以二选一、TR 只能选 `start`）
     2. **立刻**在 `/field_localizer/retry_status` 上发布 `PAUSED`——这是
        决策节点的停止信号，见决定 6
     3. 调用 `/point_lio_reset`，清空 Point-LIO 状态
     4. 等待 Point-LIO 完成重新初始化（类似开机时的静止初始化阶段）
     5. 用对应 Zone 在 `field_geometry.yaml` 里的配置位姿，重新发布
        `map → odom`（可选 ICP 精修）
     6. 成功就在 `/field_localizer/retry_status` 上发布
        `RESUMED(zone, pose)`；失败就发布 `RETRY_FAILED` 并保持暂停，
        不让决策在定位没锚定好的情况下继续动
     7. 把成功/失败结果返回给操作员的那条命令

   **依据（查了 `Robocon_2027_Rulebook_v1-1.pdf` 第 5 节）**：Retry 必须由
   裁判确认后才开始（5.2），确认后队员被明确允许物理搬动/接触机器人
   （5.3.1；6.7"未经授权触碰"违例条款本身留了"经过授权的 Retry 声明"这个
   例外）——所以人工触发不违反"BR 全程自主"的要求，那条要求管的是比赛
   进行中，Retry 窗口是规则明文的例外。另外规则本身甚至没有禁止整车重启
   （5.3.3），"不整车重启"纯粹是我们自己的工程选择，不是规则逼出来的，
   对外沟通时要说清楚这一点。

6. **决策侧的响应契约（2026-10-03 确认）**：复用操作员那一条命令本身作为
   编排点，不另外再搞一套机制。

   - `field_localizer` 发布一个状态话题 `/field_localizer/retry_status`
     （transient local，决策晚订阅也能拿到最新状态），取值
     `IDLE` / `PAUSED` / `RESUMED` / `RETRY_FAILED`
     （2026-10-04 起后面带 zone：`"PAUSED l1"`、`"RESUMED start"`，Flow 引擎按 zone 选 Retry 后的流程）
   - **`rc2027_decision` 的 Flow 引擎必须实现的最小契约**：收到 `PAUSED`
     立刻停止发布/发出任何运动指令（`CmdVel` 归零或直接停止发布），把当前
     正在执行的任务标记为"被打断，待重新评估"——不是"失败"也不是"继续"，
     是一个中间态，留给 Flow 引擎自己的恢复逻辑处理；收到 `RETRY_FAILED`
     保持停止状态，不恢复；收到 `RESUMED(zone, pose)` 才允许恢复运动，
     具体恢复到哪个任务、要不要重新规划，是 Flow 引擎自己的逻辑，这次不
     展开设计（Flow 引擎本身还没写，也不该在它不存在的时候先定死细节）
   - 这是个**最小安全契约**，不是完整方案：目前完全依赖决策节点"自觉"配合，
     没有在传输层强制拦截 `CmdVel`。后面如果想做得更保险（比如在
     `rc2027_interface` 的串口节点里加一道硬拦截，`PAUSED` 时直接丢弃
     `CmdVel` 不转发给下位机），等 `rc2027_decision` 有实际代码能联调了再加，
     现在加没有东西可以测，纯属猜

7. **Start Zone 两个 700mm 子格子的分配（2026-10-03 暂定）**：TR 用靠边的
   `box_a`，BR 用靠里的 `box_b`——减少两车开局路线交汇（TR 沿边去储存区，
   BR 往里靠近 L1 的台阶/坡道，路线也更顺）。本仓库这台车是 BR，
   `field_localizer` 的 `start_box` 参数默认值已经改成 `box_b`
   （launch 文件和节点代码两处都改了）。TR 不在这个仓库里，不用管它的默认值。
   "暂定"是因为这是队内策略安排，不是规则或几何上的硬约束，后面如果两车
   实际路线规划出来发现不合适，随时可以改，只用改这一个默认值

8. **TF 链怎么接、雷达外参放哪（2026-10-04 定，原来"待定"第一条）**：

   原来"等有车再定"是因为外参没有真实值。但这件事分两半：**链路结构**现在就能定，
   **外参数值**只是 URDF 里的参数，以后改数就行。不做的话 teach_point 和导航在
   仿真里都测不了（助教 10-04 B 段实测提出），所以提前做了。

   ```
   map --(field_localizer)--> lidar_odom --(lio_to_base)--> base_footprint --(URDF)--> base_link --> lidar_link --> lidar_imu_link
   ```

   - **`lidar_odom` 是哪个物理点**：开机（或 Retry 重置）那一刻 **Point-LIO 的 IMU**
     所在位置，并且重力对齐过（`laserMapping.cpp` 里状态量就是 IMU 位姿，
     `/aft_mapped_to_init` 的 `child_frame_id` 叫 `body`，指的就是 IMU）。
     不是车体中心，也不是雷达光心
   - **Point-LIO 的 TF 一律关掉**（`publish.tf_send_en: false`）。它发的
     `lidar_odom → lidar_link` 跟 URDF 抢 `lidar_link` 的父节点，而且名字也
     不对（它发的其实是 IMU 位姿）。**真车 bringup launch 写的时候必须覆盖这个
     参数**，vendor 的 `mid360_inner.yaml` 里是 `True`，不改 vendor 文件
   - **新节点 `lio_to_base`**（`rc2027_navigation`，C++，感知层）：订阅
     `/aft_mapped_to_init`，从 TF 查一次 URDF 里的 `base_footprint → <body_frame>`，
     发 `lidar_odom → base_footprint`。发 `base_footprint` 不发 `base_link`：
     URDF 根节点是 `base_footprint`，`base_link` 已经有父节点
   - **`body_frame`**：真车是新加进 URDF 的 `lidar_imu_link`（Mid-360 内置 IMU
     在雷达系里的出厂位置，从 `mid360_inner.yaml` 的 `extrinsic_T` 取负得到）；
     仿真是 `livox_sim`（雷达、IMU 挂同一个 link，仿真配置外参为 0）
   - **`field_localizer` 改成"锁定"**：车被摆在已知位置（Start Zone / Retry Zone，
     `field_geometry.yaml`）时，查 TF 里同一时刻的 `lidar_odom → base_footprint`，
     算 `map → lidar_odom = (map → base_footprint) · (lidar_odom → base_footprint)⁻¹`，
     锁定后 20 Hz 一直发。完整 6 自由度，不压成平面（雷达有俯仰安装角时
     `lidar_odom` 本身不是"车体平放"的朝向）。锁定之前不发——发一个没对齐的值
     比"查不到"更危险
   - **外参只有一份**：在 URDF（`robot_params.yaml` 的 `lidar` 节），两个节点都从
     TF 拿，不会出现"field_localizer 一份、lio_to_base 一份，改了一边忘了另一边"
   - **Retry 重新锁定**：Point-LIO 的 reset 服务只置标志位、下一帧才真正重置，
     所以记下调用 reset 前最后一帧 TF 的时间戳，重新锁定要求 TF 时间戳比它晚
     至少 1 s（雷达时间）。比的是里程计自己的时间戳，不跟 `now()` 比——真车上雷达
     时钟和系统时钟不一定是同一个。重置期间不发 `map → lidar_odom`
   - 纯数学部分（`pose_math.hpp`）在 host 上单测过，包括"雷达偏心 + 转角""30°
     俯仰安装"两种情况下，车开走以后算出的 `map → base_footprint` 等于真值
   - **仿真验证手段**：仿真 URDF 加了 Gazebo 真值插件（`/ground_truth/odom`），
     `ros2 run rc2027_utils ground_truth_check` 每秒打印定位误差；
     `sim_mapping.launch.py xacro_overrides:="lidar_x:=0.2 lidar_yaw:=0.3"`
     故意让雷达偏心验证外参换算

   **还没解决、等真车的部分**：
   - 外参数值（`robot_params.yaml` 的 `lidar` 节）——标定脚本已经有了
   - 真车 30° 倾斜安装时，`livox_ros_driver2` 里"IMU 跟随外参旋转"那个 patch 会把
     IMU 数据转到别的坐标系，`body_frame` 是否还等于 `lidar_imu_link`、要不要再加
     一个旋转，要装车后对着 patch 的实际行为确认
   - 真车上 Point-LIO 里程计时间戳是不是系统时间（Mid-360 没做 PTP 同步时可能是
     雷达自己的时钟）。`field_localizer` 发 `map → lidar_odom` 用的是 `now()`，
     如果两个时钟差很多，TF 查询会因为时间对不上失败——到时候要么同步时钟，
     要么改成用里程计时间戳发

9. **ICP 精修：先验地图 + small_gicp（2026-10-04 做了"决定 2 第二步"）**

   起因：仿真实测 Point-LIO 累积漂移约 0.7% 行驶距离（开 9.5 m 停车后差 6.5 cm，静止不再
   增长），Retry 锁定假设车正好在 zone 中心（实际摆偏几厘米、一两度）。这两个都只能靠和
   场地先验地图比对来纠正。用户要求开始做。

   - **配准**：把 Point-LIO 最近 `icp_frames`（10）帧 `/cloud_registered`（lidar_odom 系）
     配准到先验地图（map 系），**直接解出 map → lidar_odom**，以当前值为初值。small_gicp 的
     GICP，粗（对应点最远 1 m）→ 细（0.3 m）两级。small_gicp 组长建议的；rosdistro Humble
     没有它，但它是 header-only，vendor 在 `rc2027_navigation/third_party/`，不用改镜像
   - **什么时候做**：
     - 每次锁定以后自动一次（开机、Retry）。Retry 等这次精修做完（最多 5 s）才发 `RESUMED`，
       免得决策开始动了才跳
     - `/field_localizer/relocalize`（`std_srvs/Trigger`）随时手动触发，返回修正量和原因
     - **车在动时不做**：最近 0.5 s 车体挪动 > 2 cm 或转动 > 1° 就拒绝（"停稳再修正"）。修正那一刻
       `map → base_link` 会跳，导航中跳会让 point_navigator 猛改方向；停着时点云也最干净
     - 没有周期性自动修正——先看手动/锁定后两种够不够，够的话不加复杂度
   - **验收（不过就保持原值并报警）**：修正量按"车体在 map 里被挪了多少"算（不按
     map → lidar_odom 本身：车离 lidar_odom 原点 10 m 时它转 1° 平移就差 17 cm）。
     位置 ≤ 0.3 m、朝向 ≤ 5°、高度 ≤ 0.1 m、倾斜 ≤ 2°、内点比例（配准后离地图 ≤ 10 cm 的点）≥ 0.5、
     点数 ≥ 300、收敛——任何一条不过就拒绝。位置上限 0.3 m 是防镜像的关键：场地红蓝对称，初值
     差太多（比如 start_box 给错了 0.8 m）宁可拒绝报警，也不冒险跳过去
   - **高度上限 0.1 m 不放宽（2026-10-04 用户问过）**：仿真里在 L1 上 relocalize 被拒，"高度修正
     −0.615 m"。host 上用 Gazebo 场地网格射线模拟 L1 Retry Zone 的扫描复现：初值只差 ±20 cm /
     ±6 cm / ±2° 时 81 种组合全部配回真值；**初值朝向差 180° 时 81 种全部掉进同一个错误解**——高度
     −0.58 m、内点 0.5～0.7，和用户看到的一样。也就是车实际朝北、L1 Retry 按朝南锁定（多半用的是
     还没加"转到朝南"的旧回放路线）。0.1 m 上限挡住的正是这种"换了一层"的错误解；放宽的话 map 系
     会被拉低 0.6 m
   - **方向退化检查 + 死区（2026-10-04 加）**：助教查到 p1（西围栏边、车头朝南、前倾 30° 雷达）relocalize
     后稳定往 +y 偏 1～2 cm，原因是那里只看得到平行于 y 的立面，**y 方向没约束**（像长走廊），不是地图问题。
     - 配准完取 small_gicp 的信息矩阵 H，旋转用 Schur 补边缘化掉，转到 map 系，取 x/y 2x2 块的特征值，
       **除以点数**（绝对值跟点数成正比，没法定阈值）→ "每点平均约束强度"弱 / 强两个数 + 最弱方向
     - host 上对 Gazebo 场地射线模拟（前倾 30°）标定：西围栏边 p1、L1 走廊、p3 弱方向 0.5～0.7；起点格 4.9、
       L1 Retry Zone 3.8；理想长走廊 0.7 → 阈值 `icp_min_xy_info` **1.5**
     - 弱方向 < 1.5：**只修强方向**（`drop_weak_direction`：去掉沿弱方向的平移修正，另一个水平方向、高度、
       朝向照修），日志写"沿 (x, y) 方向约束不足，这个方向不修"；两个方向都 < 1.5 → 拒绝
     - **死区**：要应用的水平修正 < 1 cm 且 < 0.5°（`icp_deadband_xy` / `icp_deadband_yaw`）就不改水平，返回成功、
       说明"在死区内"。助教实测 ICP 重复性噪声 0.1～0.7 cm，本来就准时修正 = 加噪声。**高度单独判断**
       （`icp_deadband_z` 2 cm）：10-05 第一版把 dz 一起丢了，锁定时的 5 cm 高度差（Gazebo 台面高 5 cm）一直不修
     - 验收的上限（0.3 m 等）仍然按完整的修正量判断
     - 局限：p3 这种"中等"位置也被判成弱（0.7），那里 ICP 实际修得对，现在只修一半方向。阈值按真车 / 仿真
       每个拿放点"连调 5 次 relocalize + 看真值"再调；日志里每次都打这两个数
   - **车头摆反提示**：验收拒绝时再用"车体原地转 180°"的初值配一次，能通过就在拒绝信息里提示
     "车头是不是摆反了"。只提示、不自动采用（`flip_base_yaw`，单测覆盖）
   - **Retry 和点云**：`point_lio_reset` 以后 lidar_odom 是新坐标系，旧点云和位姿历史全部清掉，
     只收时间戳晚于"reset 前最后一帧 + 1 s"的点云（跟重新锁定同一个门槛）
   - **先验地图**：参数 `prior_map_path`（PCD，map 系），空 = 关掉 ICP、只靠锁定
     - 仿真：`rc2027_sim_bridge/maps/prior_map_sim.pcd`，助教从 MuJoCo 场地网格射线投射生成
       （`make_prior_map.py` 同目录）。仿真车看的是 Gazebo 场地（另一个人做的模型）。
       **10-04 更正**：第一次核对只看了无符号距离（"100% < 7 cm、中位 2 cm"），以为 2 cm 是采样
       间距，其实是 Gazebo `model.sdf` 把场地整体抬了 2.5 cm（助教查到）。地图已整体 z + 2.5 cm，
       复核后中位 0.8 cm、平均偏差向量约 0。教训：核对几何要看**有符号偏差**，不能只看距离
   - **仿真验收第一次跑出来 ICP 把正确位姿修偏 8 cm**（助教 10-04）：根因不在 ICP——Livox 仿真
     插件把每个点都画近了 10 cm（见插件 NOTES.md），已修。"地图 vs 网格"和"host 单测（扫描从地图
     截）"都测不出实时点云的问题，所以加了 `sim_lidar_range_check`（实时点云 vs 真实几何）
     - 真车：**还没有**。要在训练场/赛场用 Point-LIO 实地建图，再对齐到场地坐标（对齐方法、
       用不用 field.xml 做初值，等真车时再定，见下面"待定"）
   - **host 上验证过**（`test/test_map_matcher_main.cpp`，用真实先验地图）：从地图截一块当扫描，
     初值偏 12 cm / 3° → 配准后 0.26 cm / 0.02°，约 11 ms；偏 0.8 m → 拒绝；放到镜像那边 →
     内点 0.17，拒绝
   - 先验地图发在 `/field_localizer/prior_map`（transient local），RViz 里和实时点云叠着看

10. **高度：zone 带脚下表面高度；L1 Retry Zone 朝向暂定 90°（2026-10-04）**

    用户提醒：BR 上楼以后 Retry 就回 **L1 Retry Zone，在 0.6 m 高的平台上**。查代码发现两个会让
    "上楼后 Retry"直接失败的问题：
    - L1 Retry Zone 的 yaw 还是 null → `zone_pose` 抛异常，Retry 请求直接被拒
    - 锁定按 z = 0 算：在平台上锁定，map 系整体低 0.6 m，ICP 的高度验收（≤ 0.1 m）还会把修正拒掉

    决定：
    - `field_geometry.yaml` 的 zone 可以带 `z`（脚下表面高度，不写就是 0）。L1 Retry Zone 写
      `z: 0.6`（= `levels.l1.height_m`）。`Pose2D` 加了 `z`，锁定时 `map → base_footprint` 用它
    - L1 Retry Zone yaw **朝南（−90°）**，用户 10-04 定的（同一天先暂填过 90°）。镜像 π − yaw 后
      还是朝南，红蓝一样
    - 仿真 Gazebo 场地整体比规则坐标高 5 cm（地面 0.05、L1 顶 0.65），这 5 cm 由 ICP 修掉
      （在高度验收的 0.1 m 以内），配置里写的还是规则的真实高度

    - **Retry 后 ICP 验收拒绝 → RETRY_FAILED**（不是 RESUMED）。用户第一次测时车还在地面上就调了
      `retry {zone: l1}`：锁定到 L1 位姿、ICP 正确地拒绝（内点 0.34、修正 94 cm），但原来照样发
      RESUMED，决策会按错误位姿开车——在 L1 上可能直接开下平台。拒绝说明车没摆在这个 zone 里
      （或 zone 选错），宁可停着让操作员重摆再 retry。ICP 只是没来得及做（车没停稳）仍然 RESUMED
      并在日志里提示。决定 6 的契约不变（RETRY_FAILED 时决策保持停止），point_navigator 本来就会
      在 RETRY_FAILED 时停车

    - **爬楼梯时南北方向会退化（2026-10-04 仿真实测 + host 分析，对真车也重要）**：回放上楼梯后
      定位 y 往北多了 1.85 m（z、yaw 都对）。host 上对场地网格射线模拟扫描、算 GICP Hessian：
      楼梯中段 y 方向约束强度 839，z 是 237334、地面上的 y 是 21851（弱 26 倍）。原因是几何：
      车头抬起约 31°，雷达向前几乎全是天空，两侧是 L1 侧墙和场地（法向 ±x），台阶面水平（法向 z），
      看得到的"法向朝 ±y"表面只有 34 个点（地面上 545）。Point-LIO 南北方向只能靠 IMU，就漂了
      - **雷达往下倾装没用**：倾 15° / 30° 算出来 y 约束一样弱
      - **场馆的墙能解决**：加四面场馆墙（离中心 9 m、高 4 m）后同一位置 763 个，L1 走廊 44 → 916。
        仿真 world 已经加了墙（`rc2027_sim.world`；空场地对照 `rc2027_sim_empty.world`）
      - **对真车的含义**：真实场馆一般有墙、看台、横幅，Point-LIO 能看到；但如果场馆特别空旷、或者
        墙在 Mid-360 量程（约 40 m）外，就会出现同样的退化。到场地以后先看一眼；能做的补救：
        EKF 融合底盘里程计（平地上能补，爬楼梯过程中机构里程计不一定靠得住）、爬完在楼梯顶停下
        relocalize（楼梯顶平台 y 约束也不强，但比楼梯上好）
      - ICP 只用场地范围内的点（`icp_crop_half_size` 5.6 m），先验地图里没有场馆墙

    **怎么测**（仿真底盘爬不了楼梯）：`rc2027_utils` 的 `sim_climb_replay`——按贴着场地网格的 3D
    轨迹（含 z、俯仰）直接设置车的位姿和速度，Point-LIO / ICP 照常跑。Gazebo 的 IMU 是对速度
    差分算加速度、再减世界重力，所以速度和位姿一致时 IMU 读数是对的。路线：红方起点 → 西侧
    楼梯（4 级、每级 0.15 m，地面 0.05 → 0.65，爬升时俯仰约 −31°）→ L1 → L1 Retry Zone，原地转到
    朝南停住。
    局限：没有真实震动 / 冲击，IMU 饱和、机构磕碰只能真车测。步骤见 sim README"爬楼梯回放"

## 待定 / 下一步（这是接下来要专门设计的事，不在这次决定范围内）

- ~~`lidar_odom` 原点到底是车体还是雷达 / Point-LIO 的 TF 跟 URDF 抢 `lidar_link`~~
  ——2026-10-04 已定，见决定 8
- 先验地图什么时候建、怎么对齐到场地坐标系（训练场和赛场有几厘米差异，
  计划利用赛前适应时间在赛场重新建图、重新录点），细节见
  `概念/场地原点怎么选.md` 的"补充"部分
- ~~`/relocalize` 的 ICP 匹配算法选型、修正量阈值~~——2026-10-04 定了，见决定 9。
  阈值是起点值，仿真里按助教的验收步骤再调
- 真车先验地图怎么建、怎么对齐到场地坐标；要不要加周期性自动修正（现在只有锁定后 + 手动）
- ~~示教录点工具查不到 `map → base_link`~~——跟上面同一个根因，决定 8 解决后
  链路通了，仿真里待验证

## 相关文件

- `docs/decisions/0002-session-roles.md` 2026-10-03 条目——原始讨论记录
- `~/Documents/ros/notes/概念/SLAM坐标到场地坐标.md`——方案 A/B 对比、
  去年两家代码怎么做的、七条答疑
- `~/Documents/ros/notes/概念/场地原点怎么选.md`——原点候选方案对比
- `~/Documents/ros/notes/模板/field_localizer草案.md`——节点设计草稿、
  核心代码骨架
- `docs/rules/field_geometry.yaml`——场地坐标系原点约定已更新
