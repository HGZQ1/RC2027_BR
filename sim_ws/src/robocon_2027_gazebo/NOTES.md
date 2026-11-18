# 本地备注（非上游内容）

上游仓库：https://github.com/YaoZA233/ABU-Robocon-2027-Gazebo
Vendored 时的 commit：4be62dd（2026-10-01 拉取）
已移除 .git，当前按"直接拷贝源码"管理，不是 submodule。

**⚠️ 文件夹名从占位符 `rc2027_field_gazebo` 改成了 `robocon_2027_gazebo`**，
跟上游 package.xml 里 <name>robocon_2027_gazebo</name> 保持一致，
避免 `ros2 pkg list` / `ros2 launch` 看到的包名跟文件夹名对不上导致confuse。

**⚠️ License 未确认**：package.xml 写 "Proprietary"，仓库根目录没有 LICENSE 文件。
正式依赖这份代码、或者要把它合并进自己的发布物之前，先跟作者确认授权条款。

---

# rc2027_field_gazebo

复用/改造 Gazebo 场地包：https://github.com/YaoZA233/ABU-Robocon-2027-Gazebo
（Gazebo Classic + ROS2 Humble，ament_cmake 包，能直接 colcon build）。

注意：
- 原仓库 package.xml 写的 license 是 "Proprietary"，根目录又没有 LICENSE 文件，
  正式并入本仓库前先跟作者确认授权条款。
- 原仓库只有静态场地碰撞体，没有机器人、没有仿真传感器、没有可动比赛道具，
  这些需要自己补（机器人 URDF 用 spawn_entity.py 接入，雷达用社区的 Livox gazebo 插件）。
