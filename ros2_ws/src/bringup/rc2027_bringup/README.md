# rc2027_bringup

顶层启动文件、URDF、机器人物理参数集中管理（参考 GAFA 去年 `robot_params.yaml`
的集中管理思路）。

## 现状：占位骨架（舵轮），尺寸都没定

2026-10-04 底盘定为**舵轮**（`docs/decisions/0005-navigation.md`），URDF 从之前的
差速两驱占位换成：`imu_link` → `base_link` → 四角 4 个舵轮模块（`fl/fr/rl/rr`，
**固定关节**占位）+ `lidar_link`、`base_footprint`+
`camera_link`（含 `camera_optical_frame`）。

**所有尺寸都是占位值**，不是任何实测结果，细节见 `config/robot_params.yaml` 顶部注释。
- 模块数量如果是 3 个：改 URDF 里那 4 行 `swerve_module` 调用
- 模块关节现在是固定的：没有东西发 `joint_states`（下位机协议还没定）。串口桥能报
  模块角度以后，改成转向+驱动两个 continuous 关节——纯可视化，不影响定位导航
- 临时改尺寸不动 yaml：`display.launch.py xacro_overrides:="lidar_x:=0.2"`

## 怎么跑（不需要车，只需要容器）

```bash
ros2 launch rc2027_bringup display.launch.py
```

会启动 `robot_state_publisher` + `joint_state_publisher` + `rviz2`。
`display.launch.py` 读 `config/robot_params.yaml`，把每个数字转成
`xacro:arg` 传给 xacro 文件——跟去年 GAFA `rc2026_bringup` 的约定一样，
改尺寸只改 yaml，不用碰 launch 文件或 xacro 文件本身。

rviz2 打开后没有现成的 display 配置，手动加 `RobotModel` + `TF` 两个
display 就能看到骨架和 TF 树。

## 已验证 / 还没验证

写的时候没有容器环境（docker build 按约定不是我自己跑），在 host 上单独装了
`xacro`、`urdf_parser_py` 做了能做到的验证：

- ✅ `xacro urdf/rc2027_robot.urdf.xacro` 能正常处理，没有报错
- ✅ 用 `urdf_parser_py` 解析处理后的 URDF：9 个 link、8 个 joint，是一棵
  单根连通的树，没有悬空 link、没有 parent/child 指向不存在的 link
- ✅ 完整走了一遍 `display.launch.py` 实际会用的路径（读 `robot_params.yaml`
  → 拼 `xacro:arg` 参数串 → 喂给 xacro → 解析结果），参数确实能改到对应的
  joint 位置上（改了 `wheel_separation`/`lidar_z` 验证过数值联动正确）
- ✅ **2026-10-03 容器里跑通了**，`colcon build` 过了，rviz2 里能看到模型。
  但看出来两个几何 bug（见下面），已经修了：
  - 轮子/脚轮都不着地：`drive_wheel`/`caster_wheel` 两个宏的 joint z 偏移
    重复加了一次 `wheel_radius`（`base_footprint_joint` 已经把 `base_link`
    抬到轮轴高度了，轮子关节自己不该再加）。改法：驱动轮 joint 的 z 改成
    `0`；脚轮半径跟驱动轮不一样，改成 `${cr - wr}` 把高度差补回来
  - 雷达和相机悬空飘在半空：`lidar_z`/`camera_z` 之前是拍脑袋填的 0.25/0.20，
    完全没检查过跟 `chassis_height`(0.15) 的关系。改成"底盘高度 + 自身半高"
    （0.18/0.16），让它们正好坐在底盘顶面上，不留一段看不出来撑着什么的空隙

  **教训**：之前的验证只查了拓扑（连通、没有悬空 link），没有查几何上
  "东西有没有贴在该贴的地方"——这两种检查是互补的，拓扑对不代表位置对。
  用 `urdf_parser_py` 沿 TF 链累加各 link 的绝对高度、跟半径比对，这类数值
  检查其实在 host 上也能做，这次是实际 rviz 看出来才补的，以后类似改动
  应该先做这步数值检查再丢进容器
- ❌ `realsense2_camera`/相机节点、`joint_state_publisher` 给的轮子转动
  效果还没看过（现在只是静态摆位对不对）

## 待做
- [x] URDF 骨架（占位尺寸；10-04 从差速两驱换成舵轮，等机械定稿后更新）
- [x] `robot_params.yaml`：雷达/IMU/相机安装位置，先填占位值
- [ ] `full_system.launch.py`：整车一键启动（等 `rc2027_interface`/
      `rc2027_decision`/`rc2027_navigation` 有实际内容后再写，现在只有
      `display.launch.py` 这个"只看 URDF"的半成品）
- [x] 在容器里实际跑一遍 `display.launch.py`，确认 rviz2 渲染正常——
      2026-10-03 跑通，顺带发现并修了两个几何 bug，见上面"已验证"
- [x] 底盘方案定了（舵轮），`urdf.xacro` 轮子部分已重写（2026-10-04）。舵轮版在 host 上
      检查过：单根树、轮子底部 z=0 着地、雷达底面正好坐在底盘顶面
- [ ] 雷达/相机的占位尺寸，等 `rc2027_utils/calibrate_lidar_base.py`
      实机标定 + 卷尺实测后替换成真实值
