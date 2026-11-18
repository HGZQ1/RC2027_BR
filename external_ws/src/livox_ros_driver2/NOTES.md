# 本地备注（非上游内容）

来源：广美去年准备的 `MID-360S的开发资料/livox_ros_driver2`（支持 Mid-360S，带正确的
`MID360s_config.json` 协议字段），不是官方仓库原始 clone，已经是针对 360S 修过的版本。

## 本地 patch：IMU 跟随外参旋转

移植自华工仓库 `src/comm/pub_handler.cpp` 的同名修改，三处改动，全部标了
`// RC2027 patch` 注释，方便 diff：

1. 新增全局 `extrinsic_global` 旋转矩阵（初始为单位阵）
2. `SetLidarsExtParam()` 里，算完点云用的 `extrinsic_.rotation` 后，
   用同一组 roll/pitch/yaw 再算一份 `extrinsic_global.rotation`
3. IMU 回调里，陀螺仪/加速度计数据从"原样透传"改成"乘 `extrinsic_global.rotation`"

**原因**：官方驱动的外参（`MID360s_config.json` 里的 `extrinsic_parameter`）只旋转
点云，不旋转 IMU。雷达侧装/倒装时点云被转正了，IMU 却还是斜的，会导致 FAST-LIO /
Point-LIO 的世界坐标系跟着倾斜（沿倾斜方向的距离会按 cos(倾角) 被压缩）。

**效果**：`MID360s_config.json` 里正常填实际安装角度（比如 `pitch: 30`）即可，
点云和 IMU 会同时转正，下游 FAST-LIO/Point-LIO 的 `extrinsic_R` 保持单位阵就行，
不需要再手动算一个倾斜补偿矩阵。

**没验证的点**：这个 patch 本身没有在真实硬件上跑过（代码层面跟华工的改法逐行对照
过，结构确认一致），车到了之后第一件事应该是验证：静止时 `ros2 topic echo
/livox/imu --field linear_acceleration` 的 z 值接近 1g、x/y 接近 0。

## 验证记录

### 2026-10-02：IMU 旋转 patch 静态验证（加速度）✅

- 条件：Mid-360S 拆下来**平放在桌面**，`MID360s_config.json` 外参为 `pitch: 30`（按装车角度填写），镜像为 2026-10-02 13:35 构建的 `rc2027:br-dev`
- `ros2 topic echo /livox/imu --field linear_acceleration` 静止读数（单位 g）：x ≈ 0.49～0.51，y ≈ -0.01，z ≈ 0.84～0.87
- 模长 √(0.505² + 0.868²) ≈ 1.00 g，倾角 atan2(0.505, 0.868) ≈ 30.2°，和外参的 30° 一致
- 同时 RViz 里点云也整体倾斜：雷达平放、却按 30° 去"转正"，点云和 IMU **一起被转了 30°**，两者一致。说明 patch 让 IMU 跟随了外参旋转。如果 patch 没生效，IMU 应该显示 x≈0、z≈1，和点云对不上
- **还没验证的**：陀螺仪（angular_velocity）的旋转。静止时陀螺仪读数接近 0，看不出转没转，需要绕轴转动雷达对比
- 补充：用手把雷达倾斜到大约装车角度后，读数变为 x ≈ -0.09，y ≈ 0，z ≈ 0.98～1.00，残余倾角约 5°。这是手持角度不准造成的，属于预期之内。点云地面也基本和 RViz 网格平行。链路结论：外参 → 点云旋转 + IMU 旋转，两路都通了
- 提示：装车后可以反过来用 IMU 求真实安装角。先把外参 pitch 临时设为 0，车停在水平地面上，读静止加速度，pitch ≈ atan2(x, z)。这样得到的角度比"设计图上写的 30°"更准

### 2026-10-02：IMU 旋转 patch 陀螺仪验证 ✅

- 条件：雷达平放在桌面，外参 `pitch: 30`，绕竖直轴在桌面上原地来回旋转 10 秒，采集 `/livox/imu`（约 200 Hz，2005 个样本，其中 1545 个是转动中的）
- 对转动中的样本把角速度方向归一化后取平均，得到转轴方向 x=+0.499，y=-0.015，z=+0.867，偏离 z 轴 29.9°
- 和加速度的结果（x≈+0.50，z≈+0.87）方向、符号完全一致，说明陀螺仪和加速度计被同一个旋转矩阵转动，**方向没有反**
- 结论：IMU 旋转 patch 的加速度和陀螺仪两部分都实机验证通过
