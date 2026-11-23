// 先验地图 + 点云配准（ICP 精修）——不依赖 ROS，host 上用真实先验地图单测。
//
// 这是 0003 决定 2 里 field_localizer 的"第二步"：开局 / Retry 锁定之后，以及操作员
// 调 /field_localizer/relocalize 时，把 Point-LIO 最近的点云（lidar_odom 系）配准到
// 场地先验地图（map 系），直接解出 map -> lidar_odom。决定写在 0003 决定 9。
//
// 配准用 small_gicp（组长建议的；header-only，vendor 在 third_party/，不用改镜像）的
// GICP，两级：先粗（对应点最远 1 m，容忍初值差几十厘米），再细（0.3 m）。
//
// "算出来了"不等于"能用"：场地红蓝对称，初值差太多可能配到镜像位置。所以 decide()
// 做验收——修正量（车体在 map 里被挪了多远、转了多少）超阈值、内点比例太低、点太少、
// 没收敛，任何一条不满足就拒绝，保持原值并报警（0003 决定 2 原文的要求）。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "rc2027_navigation/pose_math.hpp"

namespace rc2027_navigation
{

using Points = std::vector<Eigen::Vector3f>;

/// 读 PCD（只取 x y z，支持 DATA ascii / binary，float32）。读不了抛 std::runtime_error
Points load_pcd_xyz(const std::string & path);

struct MatchParams
{
  double map_voxel{0.1};          // m，先验地图降采样
  double scan_voxel{0.1};         // m，实时点云降采样
  int num_neighbors{10};          // 估计协方差用的邻居数
  int num_threads{4};
  double coarse_max_corr{1.0};    // m，粗配准对应点最远距离
  double fine_max_corr{0.3};      // m，细配准
  int max_iterations{30};
  // 收敛判据：一次迭代的平移 / 旋转更新量都小于这个就算收敛。small_gicp 默认 1 mm / 0.1°，
  // 仿真雷达和先验地图有 4～5 cm 的底噪，对应点来回跳，更新量常在 1～3 mm 晃、30 次也到不了 1 mm
  // （2026-10-06 助教：far_look 约束够、修正才 2.5 cm、内点 0.92 却报"没收敛"）。
  // 死区 1 cm，3 mm 以内的抖动不影响结果
  double translation_eps{3e-3};   // m
  double rotation_eps{3.5e-3};    // rad（约 0.2°）
  double inlier_dist{0.1};        // m，算内点比例：配准后离地图最近点不超过这个算内点
  // 只用 map 系里 |x|、|y| <= crop_half_size 的点（按初值变到 map 系判断）；0 = 不裁。
  // 先验地图只有场地，场馆墙、观众席这些场外的点地图里没有，留着只会把内点比例拉低
  // （2026-10-04 仿真加了场馆墙以后需要）
  double crop_half_size{0.0};
};

struct MatchResult
{
  bool converged{false};
  Transform3D map_to_odom;        // 配准结果
  size_t num_points{0};           // 降采样后参与配准的点数
  double inlier_ratio{0.0};       // 0~1
  double rmse{0.0};               // m，内点到地图最近点的均方根距离
  int iterations{0};
  // 方向退化（2026-10-04 加，0003 决定 9 补充）：配准的信息矩阵（small_gicp 的 H，6x6）把旋转边缘化掉
  // （Schur 补）以后，平面内（map 系 x/y）平移那 2x2 块的两个特征值 = 两个方向上"挪一点点代价涨多快"。
  // 最小的那个特别小 = 沿那个方向挪几厘米代价几乎不变（像在长走廊里），配准在这个方向上的结果不可信。
  // **除以了点数**（每个点平均贡献多少约束）：绝对值跟点数成正比，同一个位置多扫几帧就变大，没法定阈值。
  // 2026-10-04 host 上对 Gazebo 场地射线模拟（前倾 30° 雷达）：西围栏边 p1 / L1 走廊 / p3 弱方向 0.5～0.7，
  // 起点格 4.9、L1 Retry Zone 3.8；理想长走廊 0.7
  double info_xy_min{0.0};
  double info_xy_max{0.0};
  double weak_dir_x{1.0};         // 最弱方向（map 系单位向量）
  double weak_dir_y{0.0};
};

/// 修正量：同一个 lidar_odom -> base_footprint 下，换了 map -> lidar_odom 以后
/// 车体在 map 里移动了多少。用"车体挪了多少"而不是"map->lidar_odom 本身变了多少"：
/// 车离 lidar_odom 原点 10 m 时，map->lidar_odom 转 1° 平移就差 17 cm，但车体只转了 1°
struct Correction
{
  double dx{0.0};
  double dy{0.0};
  double dz{0.0};
  double dyaw{0.0};               // rad
  double dtilt{0.0};              // rad，map->lidar_odom 的 roll/pitch 变化（合成角）
  double xy() const;
};

Correction correction_between(
  const Transform3D & before_map_to_odom, const Transform3D & after_map_to_odom,
  const Transform3D & odom_to_base);

/// 把 map -> lidar_odom 改成"车体原地转 180° 以后"对应的值（车体位置不变）。
/// 用途：ICP 被拒时，试一下"车头摆反了"的假设——操作员在 L1 Retry Zone 摆车摆反方向是很容易
/// 犯的错（2026-10-04 仿真里就出现过：朝向差 180° 时 ICP 会掉进一个错误解，高度差 0.58 m）
Transform3D flip_base_yaw(const Transform3D & map_to_odom, const Transform3D & odom_to_base);

/// 去掉 after 里沿 (weak_dir_x, weak_dir_y)（map 系单位向量）的那部分平面平移修正，其它（另一个水平方向、
/// 高度、朝向、倾斜）照常——方向退化时只修约束得住的方向（0002 10-04 助教查 p1 偏差那条的建议）。
/// 修正量按车体算（跟 correction_between 一样）
Transform3D drop_weak_direction(
  const Transform3D & before_map_to_odom, const Transform3D & after_map_to_odom,
  const Transform3D & odom_to_base, double weak_dir_x, double weak_dir_y);

/// 只保留 after 里的高度修正：车体的 x、y、朝向、倾斜都用 before 的，z 用 after 的。
/// 死区用：水平修正小到不值得改时，高度差照样要修（2026-10-05 助教：锁定按 z = 0 / 0.6，Gazebo 台面高 5 cm，
/// 死区把 dz 一起丢了，高度一直错 5 cm；L1 / L2 上会影响方块检测）
Transform3D keep_only_z(
  const Transform3D & before_map_to_odom, const Transform3D & after_map_to_odom,
  const Transform3D & odom_to_base);

struct AcceptParams
{
  double max_correction_xy{0.3};          // m
  double max_correction_yaw{0.0873};      // rad（5°）
  double max_correction_z{0.1};           // m
  double max_tilt_change{0.0349};         // rad（2°）
  double min_inlier_ratio{0.5};
  size_t min_points{300};
  // "只有 z 错"自愈（2026-10-06）：Point-LIO 的 z 在车顶着东西抖的时候会掉 0.6 m（仿真里 NAV @pick 贴方块那次），
  // ICP 其实算对了（水平几乎不动、内点 0.88、约束强），却被 max_correction_z（0.1 m，挡"换了一层"的错误解）拒掉，
  // 之后层数检测全错。错误解的特征是 xy / 朝向也跟着被拉歪、内点低（0003 决定 9 / 10：内点 0.5～0.7）；
  // 水平和朝向都几乎不动、内点又高的大幅高度修正才放行。z_only_max_dz = 0 关掉这个放行
  double z_only_max_dz{1.0};              // m，再大的高度修正也不放行
  double z_only_max_xy{0.02};             // m
  double z_only_max_yaw{0.0175};          // rad（1°）
  double z_only_min_inlier{0.85};
};

struct AcceptDecision
{
  bool accepted{false};
  bool large_z{false};            // 接受了，但这是超过 max_correction_z 的"只有 z 错"修正（要打日志）
  std::string reason;             // 拒绝原因（中文），接受时为空
};

/// 超过 max_correction_z 的高度修正，是不是"只有 z 错"（水平 / 朝向几乎不动、内点高）：是就放行
bool z_only_error(const MatchResult & result, const Correction & correction, const AcceptParams & params);

AcceptDecision decide(
  const MatchResult & result, const Correction & correction, const AcceptParams & params);

class MapMatcher
{
public:
  /// 先验地图（map 系）预处理：降采样、建 KD 树、估协方差，只做一次
  MapMatcher(const Points & map_points, const MatchParams & params = MatchParams{});
  ~MapMatcher();
  MapMatcher(const MapMatcher &) = delete;
  MapMatcher & operator=(const MapMatcher &) = delete;

  /// 降采样后的地图点数
  size_t map_size() const;

  /// 降采样后的地图点（给 RViz 发 /prior_map 看用）
  Points map_points() const;

  /// 把 lidar_odom 系的点云配准到地图，init 是当前的 map -> lidar_odom
  MatchResult match(const Points & scan_in_odom, const Transform3D & init_map_to_odom) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  MatchParams params_;
};

}  // namespace rc2027_navigation
