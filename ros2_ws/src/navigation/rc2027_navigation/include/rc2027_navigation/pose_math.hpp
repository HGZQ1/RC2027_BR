// 刚体变换的最小实现 + 定位链路上两处要用的换算——纯数学，不依赖 ROS/Eigen，
// 方便在 host 上单测（host 没有 Eigen，容器里有，但不想让单测依赖容器）。
//
// 为什么需要这两个换算，见 docs/decisions/0003-field-localization.md 决定 8：
//
//   Point-LIO 给的是 lidar_odom -> body（body = Point-LIO 的 IMU 坐标系，
//   不是车体），URDF 给的是 base_footprint -> ... -> body。
//
//   lio_to_base 节点：   lidar_odom -> base_footprint
//                       = (lidar_odom -> body) * (base_footprint -> body)^-1
//
//   field_localizer：   map -> lidar_odom
//                       = (map -> base_footprint，即车被摆在 Start/Retry Zone 的位姿)
//                         * (lidar_odom -> base_footprint，同一时刻 TF 里查到的)^-1
//
// 这样"雷达装在车上哪里、歪了多少"只在 URDF 里写一次，两个节点都从 TF 拿，
// 不会出现两边各配一份外参、改了一边忘了另一边。
#pragma once

#include "rc2027_navigation/field_geometry.hpp"

namespace rc2027_navigation
{

/// 3D 刚体变换：平移 (x, y, z) + 单位四元数 (qx, qy, qz, qw)。
/// 语义跟 geometry_msgs/Transform 一样：把子坐标系里的点变换到父坐标系。
struct Transform3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

/// a * b：先 b 后 a（parent<-mid 乘 mid<-child 得到 parent<-child）。
Transform3D compose(const Transform3D & a, const Transform3D & b);

/// 逆变换。
Transform3D inverse(const Transform3D & t);

/// 平面位姿 -> 3D 变换（z = pose.z，只绕 z 轴转）。
Transform3D from_pose2d(const Pose2D & pose);

/// 取变换的 yaw（绕 z 轴，弧度，[-pi, pi]）。
double yaw_of(const Transform3D & t);

/// lidar_odom -> base_footprint = (lidar_odom -> body) * (base_footprint -> body)^-1
Transform3D odom_to_base(const Transform3D & odom_to_body, const Transform3D & base_to_body);

/// map -> lidar_odom = (map -> base_footprint) * (lidar_odom -> base_footprint)^-1
///
/// base_in_map 是车在场地上的位姿（Start/Retry Zone 配置值，z 是脚下表面高度，只有 yaw），
/// odom_to_base_now 是同一时刻 TF 里的 lidar_odom -> base_footprint（6 自由度，
/// 带 Point-LIO 重力对齐和雷达安装角带来的 roll/pitch）。结果是完整 6 自由度，
/// 不压成平面——雷达有俯仰安装角时 lidar_odom 本身就不是"车体平放"的朝向，
/// 压平会把这部分误差丢到 map -> base_footprint 里。
Transform3D map_to_odom(const Pose2D & base_in_map, const Transform3D & odom_to_base_now);

}  // namespace rc2027_navigation
