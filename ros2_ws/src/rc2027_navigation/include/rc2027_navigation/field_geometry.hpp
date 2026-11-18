// 从 field_geometry.yaml 算出某个 side/zone 的位姿——纯逻辑，不依赖 ROS。
//
// field_localizer_node 只是这个模块的薄壳：读配置、挑 zone、广播 TF。真正
// "该用哪个数字"全部在这里，方便不开容器也能单测（见 test/ 下的测试）。
//
// 2026-10-03 组长明确：定位算感知层，field_localizer 要改成 C++
// （见 docs/decisions/0002-session-roles.md）。这是 Python 版
// rc2027_navigation/field_geometry.py 的直接移植，逻辑和异常条件完全
// 照抄，没有改行为。
#pragma once

#include <array>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace rc2027_navigation
{

struct Pose2D
{
  double x;
  double y;
  double yaw;
  // 车脚下表面的高度（m，map 系）。平面导航用不到；锁定 map -> lidar_odom 时要用：
  // L1 Retry Zone 在 0.6 m 高的平台上，按 z=0 锁定的话 map 系整体低 0.6 m，
  // ICP 的高度验收（<= 0.1 m）还会把修正拒掉（2026-10-04）
  double z{0.0};
};

/// 读取 field_geometry.yaml。文件不存在/解析失败会抛 YAML::Exception 或
/// std::runtime_error，不在这里吞掉——field_localizer_node 启动时失败要
/// 让它直接崩，不要带着读不到配置的状态继续跑。
YAML::Node load_geometry(const std::string & path);

/// 返回 side/zone 对应的位姿 (x, y, yaw)。
///
/// @param geometry   load_geometry() 的结果。
/// @param side       "red" 或 "blue"。
/// @param zone       "start" 或 "l1"。
/// @param start_box  zone == "start" 时，选 "box_a" 或 "box_b" 两个 700mm
///                   子格子中的哪一个（具体哪个给 TR 哪个给 BR 是队伍自己
///                   选，不是本模块决定的）。zone == "l1" 时忽略。
/// @throws std::invalid_argument side/zone 不合法，或者 yaml 里对应的 yaw
///                   还是 null（没定，不能当成定位初值用）。
/// 返回的 z 是 yaml 里该 zone 的 z（脚下表面高度），没写就是 0（Start Zone 在地面上）。
Pose2D zone_pose(
  const YAML::Node & geometry, const std::string & side, const std::string & zone,
  const std::string & start_box = "box_a");

/// 绕 z 轴的平面旋转，返回 (x, y, z, w) 四元数分量。
std::array<double, 4> yaw_to_quaternion(double yaw);

}  // namespace rc2027_navigation
