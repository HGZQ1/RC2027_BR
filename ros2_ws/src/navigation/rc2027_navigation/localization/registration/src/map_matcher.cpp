#include "rc2027_navigation/map_matcher.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace rc2027_navigation
{

namespace
{

Eigen::Isometry3d to_eigen(const Transform3D & t)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = Eigen::Quaterniond(t.qw, t.qx, t.qy, t.qz).normalized().toRotationMatrix();
  out.translation() = Eigen::Vector3d(t.x, t.y, t.z);
  return out;
}

Transform3D from_eigen(const Eigen::Isometry3d & e)
{
  const Eigen::Quaterniond q(e.rotation());
  Transform3D t;
  t.x = e.translation().x();
  t.y = e.translation().y();
  t.z = e.translation().z();
  t.qx = q.x();
  t.qy = q.y();
  t.qz = q.z();
  t.qw = q.w();
  return t;
}

}  // namespace

// ───────────────────────── PCD ─────────────────────────

Points load_pcd_xyz(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("打不开先验地图 " + path);
  }

  std::vector<std::string> fields;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  size_t num_points = 0;
  std::string data;

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ss(line);
    std::string key;
    ss >> key;
    if (key == "FIELDS") {
      for (std::string f; ss >> f;) {fields.push_back(f);}
    } else if (key == "SIZE") {
      for (int v; ss >> v;) {sizes.push_back(v);}
    } else if (key == "TYPE") {
      for (char c; ss >> c;) {types.push_back(c);}
    } else if (key == "COUNT") {
      for (int v; ss >> v;) {counts.push_back(v);}
    } else if (key == "POINTS") {
      ss >> num_points;
    } else if (key == "DATA") {
      ss >> data;
      break;  // 后面就是数据
    }
  }
  if (counts.empty()) {
    counts.assign(fields.size(), 1);
  }
  if (fields.size() != sizes.size() || fields.size() != types.size() ||
    fields.size() != counts.size())
  {
    throw std::runtime_error("PCD 头不完整: " + path);
  }

  // 每个字段在一个点里的字节偏移 / 列号
  int offset[3] = {-1, -1, -1};
  int column[3] = {-1, -1, -1};
  int byte_pos = 0;
  int col_pos = 0;
  for (size_t i = 0; i < fields.size(); ++i) {
    for (int k = 0; k < 3; ++k) {
      if (fields[i] == std::string(1, "xyz"[k])) {
        if (types[i] != 'F' || sizes[i] != 4) {
          throw std::runtime_error("PCD 的 x/y/z 必须是 float32: " + path);
        }
        offset[k] = byte_pos;
        column[k] = col_pos;
      }
    }
    byte_pos += sizes[i] * counts[i];
    col_pos += counts[i];
  }
  if (offset[0] < 0 || offset[1] < 0 || offset[2] < 0) {
    throw std::runtime_error("PCD 里没有 x y z 字段: " + path);
  }

  Points points;
  points.reserve(num_points);
  if (data == "binary") {
    std::vector<char> buf(byte_pos);
    for (size_t i = 0; i < num_points; ++i) {
      if (!in.read(buf.data(), byte_pos)) {
        throw std::runtime_error("PCD 数据比 POINTS 声明的少: " + path);
      }
      float v[3];
      for (int k = 0; k < 3; ++k) {
        std::memcpy(&v[k], buf.data() + offset[k], sizeof(float));
      }
      if (std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2])) {
        points.emplace_back(v[0], v[1], v[2]);
      }
    }
  } else if (data == "ascii") {
    for (size_t i = 0; i < num_points && std::getline(in, line); ++i) {
      std::istringstream ss(line);
      std::vector<float> vals;
      for (float f; ss >> f;) {vals.push_back(f);}
      if (static_cast<int>(vals.size()) < col_pos) {
        continue;
      }
      points.emplace_back(vals[column[0]], vals[column[1]], vals[column[2]]);
    }
  } else {
    throw std::runtime_error("PCD 的 DATA 只支持 ascii / binary（不支持 " + data + "）: " + path);
  }
  return points;
}

// ───────────────────────── 修正量 / 验收 ─────────────────────────

double Correction::xy() const {return std::hypot(dx, dy);}

Correction correction_between(
  const Transform3D & before_map_to_odom, const Transform3D & after_map_to_odom,
  const Transform3D & odom_to_base)
{
  const auto before = compose(before_map_to_odom, odom_to_base);
  const auto after = compose(after_map_to_odom, odom_to_base);
  Correction c;
  c.dx = after.x - before.x;
  c.dy = after.y - before.y;
  c.dz = after.z - before.z;
  c.dyaw = std::atan2(std::sin(yaw_of(after) - yaw_of(before)), std::cos(yaw_of(after) - yaw_of(before)));

  // map->lidar_odom 的倾斜变化：两个旋转之间去掉 yaw 以后剩下的角度。
  // 用 z 轴夹角：重力方向变了多少
  const Eigen::Vector3d z_before = to_eigen(before_map_to_odom).linear().col(2);
  const Eigen::Vector3d z_after = to_eigen(after_map_to_odom).linear().col(2);
  c.dtilt = std::acos(std::max(-1.0, std::min(1.0, z_before.dot(z_after))));
  return c;
}

Transform3D flip_base_yaw(const Transform3D & map_to_odom, const Transform3D & odom_to_base)
{
  Transform3D turn;  // 绕车体自己的 z 轴转 180°
  turn.qz = 1.0;
  turn.qw = 0.0;
  const auto base_flipped = compose(compose(map_to_odom, odom_to_base), turn);
  return compose(base_flipped, inverse(odom_to_base));
}

Transform3D drop_weak_direction(
  const Transform3D & before_map_to_odom, const Transform3D & after_map_to_odom,
  const Transform3D & odom_to_base, double weak_dir_x, double weak_dir_y)
{
  const auto before = compose(before_map_to_odom, odom_to_base);
  auto base = compose(after_map_to_odom, odom_to_base);
  const double along = (base.x - before.x) * weak_dir_x + (base.y - before.y) * weak_dir_y;
  base.x -= along * weak_dir_x;
  base.y -= along * weak_dir_y;
  return compose(base, inverse(odom_to_base));
}

Transform3D keep_only_z(
  const Transform3D & before_map_to_odom, const Transform3D & after_map_to_odom,
  const Transform3D & odom_to_base)
{
  auto base = compose(before_map_to_odom, odom_to_base);
  base.z = compose(after_map_to_odom, odom_to_base).z;
  return compose(base, inverse(odom_to_base));
}

bool z_only_error(const MatchResult & r, const Correction & c, const AcceptParams & p)
{
  return p.z_only_max_dz > 0.0 && std::abs(c.dz) <= p.z_only_max_dz &&
         c.xy() <= p.z_only_max_xy && std::abs(c.dyaw) <= p.z_only_max_yaw &&
         r.inlier_ratio >= p.z_only_min_inlier;
}

AcceptDecision decide(
  const MatchResult & r, const Correction & c, const AcceptParams & p)
{
  AcceptDecision d;
  std::ostringstream why;
  why.precision(3);
  if (r.num_points < p.min_points) {
    why << "点太少（" << r.num_points << " < " << p.min_points << "）";
  } else if (!r.converged) {
    why << "配准没收敛";
  } else if (r.inlier_ratio < p.min_inlier_ratio) {
    why << "内点比例 " << r.inlier_ratio << " < " << p.min_inlier_ratio << "（点云和地图对不上）";
  } else if (c.xy() > p.max_correction_xy) {
    why << "位置修正 " << c.xy() << " m 超过上限 " << p.max_correction_xy
        << " m（初值差太多，可能配到了镜像位置）";
  } else if (std::abs(c.dyaw) > p.max_correction_yaw) {
    why << "朝向修正 " << c.dyaw * 180.0 / M_PI << "° 超过上限 "
        << p.max_correction_yaw * 180.0 / M_PI << "°";
  } else if (std::abs(c.dz) > p.max_correction_z && !z_only_error(r, c, p)) {
    why << "高度修正 " << c.dz << " m 超过上限 " << p.max_correction_z << " m";
    if (p.z_only_max_dz > 0.0 && std::abs(c.dz) <= p.z_only_max_dz && c.xy() < 0.05 &&
      std::abs(c.dyaw) < 0.035)
    {
      why << "（水平 / 朝向几乎没动，像是只有 z 错了，但内点 " << r.inlier_ratio << " < "
          << p.z_only_min_inlier << " 或水平修正 > " << p.z_only_max_xy
          << " m，不敢自动修，建议 Retry）";
    }
  } else if (c.dtilt > p.max_tilt_change) {
    why << "倾斜修正 " << c.dtilt * 180.0 / M_PI << "° 超过上限 "
        << p.max_tilt_change * 180.0 / M_PI << "°";
  } else {
    d.accepted = true;
    d.large_z = std::abs(c.dz) > p.max_correction_z;
    return d;
  }
  d.reason = why.str();
  return d;
}

// ───────────────────────── 配准 ─────────────────────────

using small_gicp::KdTree;
using small_gicp::PointCloud;

struct MapMatcher::Impl
{
  std::shared_ptr<PointCloud> target;
  std::shared_ptr<KdTree<PointCloud>> target_tree;
};

MapMatcher::MapMatcher(const Points & map_points, const MatchParams & params)
: impl_(std::make_unique<Impl>()), params_(params)
{
  if (map_points.empty()) {
    throw std::runtime_error("先验地图是空的");
  }
  const PointCloud raw(map_points);
  impl_->target = small_gicp::voxelgrid_sampling_omp(raw, params_.map_voxel, params_.num_threads);
  impl_->target_tree = std::make_shared<KdTree<PointCloud>>(
    impl_->target, small_gicp::KdTreeBuilderOMP(params_.num_threads));
  small_gicp::estimate_covariances_omp(
    *impl_->target, *impl_->target_tree, params_.num_neighbors, params_.num_threads);
}

MapMatcher::~MapMatcher() = default;

size_t MapMatcher::map_size() const {return impl_->target->size();}

Points MapMatcher::map_points() const
{
  Points out;
  out.reserve(impl_->target->size());
  for (size_t i = 0; i < impl_->target->size(); ++i) {
    out.emplace_back(impl_->target->point(i).head<3>().cast<float>());
  }
  return out;
}

MatchResult MapMatcher::match(const Points & scan_in_odom, const Transform3D & init_map_to_odom) const
{
  MatchResult out;
  out.map_to_odom = init_map_to_odom;
  if (scan_in_odom.empty()) {
    return out;
  }

  Points cropped;
  const Points * input = &scan_in_odom;
  if (params_.crop_half_size > 0.0) {
    const Eigen::Isometry3f init_f = to_eigen(init_map_to_odom).cast<float>();
    cropped.reserve(scan_in_odom.size());
    for (const auto & p : scan_in_odom) {
      const Eigen::Vector3f m = init_f * p;
      if (std::abs(m.x()) <= params_.crop_half_size && std::abs(m.y()) <= params_.crop_half_size) {
        cropped.push_back(p);
      }
    }
    input = &cropped;
  }
  if (input->empty()) {
    return out;
  }

  const PointCloud raw(*input);
  auto source = small_gicp::voxelgrid_sampling_omp(raw, params_.scan_voxel, params_.num_threads);
  out.num_points = source->size();
  if (source->size() < static_cast<size_t>(params_.num_neighbors) + 1) {
    return out;
  }
  auto source_tree = std::make_shared<KdTree<PointCloud>>(
    source, small_gicp::KdTreeBuilderOMP(params_.num_threads));
  small_gicp::estimate_covariances_omp(
    *source, *source_tree, params_.num_neighbors, params_.num_threads);

  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> reg;
  reg.reduction.num_threads = params_.num_threads;
  reg.optimizer.max_iterations = params_.max_iterations;
  reg.criteria.translation_eps = params_.translation_eps;
  reg.criteria.rotation_eps = params_.rotation_eps;

  // 粗 -> 细两级
  Eigen::Isometry3d T = to_eigen(init_map_to_odom);
  int iterations = 0;
  bool converged = true;
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
  for (const double max_corr : {params_.coarse_max_corr, params_.fine_max_corr}) {
    reg.rejector.max_dist_sq = max_corr * max_corr;
    const auto r = reg.align(*impl_->target, *source, *impl_->target_tree, T);
    T = r.T_target_source;
    iterations += static_cast<int>(r.iterations);
    converged = r.converged;  // 以最后一级为准
    H = r.H;
  }
  out.converged = converged;
  out.iterations = iterations;
  out.map_to_odom = from_eigen(T);

  // 方向退化：small_gicp 的 H 按 [旋转 3, 平移 3] 排，扰动右乘（source 系，也就是 lidar_odom 系）。
  // 先把旋转边缘化（Schur 补：旋转不确定也会让平移变弱），再转到 map 系取 x/y 那 2x2 块
  {
    const Eigen::Matrix3d Hrr = H.block<3, 3>(0, 0) + 1e-9 * Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d Htr = H.block<3, 3>(3, 0);
    const Eigen::Matrix3d S = H.block<3, 3>(3, 3) - Htr * Hrr.ldlt().solve(Htr.transpose());
    const Eigen::Matrix3d S_map = T.linear() * S * T.linear().transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(S_map.block<2, 2>(0, 0));
    const double n = static_cast<double>(source->size());
    out.info_xy_min = std::max(0.0, es.eigenvalues()(0)) / n;
    out.info_xy_max = std::max(0.0, es.eigenvalues()(1)) / n;
    out.weak_dir_x = es.eigenvectors()(0, 0);
    out.weak_dir_y = es.eigenvectors()(1, 0);
  }

  // 内点比例 / RMSE：配准后每个点到地图最近点的距离
  size_t inliers = 0;
  double sq_sum = 0.0;
  const double inlier_sq = params_.inlier_dist * params_.inlier_dist;
  for (size_t i = 0; i < source->size(); ++i) {
    const Eigen::Vector4d p = T * source->point(i);
    size_t idx = 0;
    double sq = 0.0;
    if (impl_->target_tree->nearest_neighbor_search(p, &idx, &sq) && sq <= inlier_sq) {
      ++inliers;
      sq_sum += sq;
    }
  }
  out.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(source->size());
  out.rmse = inliers > 0 ? std::sqrt(sq_sum / static_cast<double>(inliers)) : 0.0;
  return out;
}

}  // namespace rc2027_navigation
