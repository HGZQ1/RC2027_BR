#include "rc2027_perception/block_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <queue>
#include <unordered_map>
#include <utility>

namespace rc2027_perception
{

namespace
{

using XY = std::pair<double, double>;

/// 一层切片里的点：水平位置 + 高度（高度用来判断方块哪一头看全了）
struct SlicePts
{
  std::vector<XY> xy;
  std::vector<double> z;
  size_t size() const {return xy.size();}
};

constexpr double kPi = 3.14159265358979323846;

/// v 的第 q 分位（0～1），会打乱 v
double quantile(std::vector<double> & v, double q)
{
  const size_t k = static_cast<size_t>(q * static_cast<double>(v.size() - 1));
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
  return v[k];
}

struct Box2
{
  double u_lo, u_hi, v_lo, v_hi;
};

/// 点转 -theta 以后的外接矩形（用 2% / 98% 分位，几个飞点不影响）
Box2 rotated_box(const std::vector<XY> & pts, double theta)
{
  const double c = std::cos(theta), s = std::sin(theta);
  std::vector<double> u, v;
  u.reserve(pts.size());
  v.reserve(pts.size());
  for (const auto & p : pts) {
    u.push_back(c * p.first + s * p.second);
    v.push_back(-s * p.first + c * p.second);
  }
  Box2 b;
  b.u_lo = quantile(u, 0.02);
  b.u_hi = quantile(u, 0.98);
  b.v_lo = quantile(v, 0.02);
  b.v_hi = quantile(v, 0.98);
  return b;
}

/// 近边那个面到底在哪。edge 是 2% 分位（带噪声时它在真实的面前面约 2 倍噪声：Mid-360 噪声 1 cm 时
/// 中心就偏 2 cm——2026-10-05 host 扫描查出来的，Gazebo 插件几乎没噪声所以仿真里没暴露）。
/// 在 edge 往里 6 cm 的范围里找点最密的地方（面上的点在这个轴上挤在一起，另一个面的点是均匀铺开的），
/// 取那附近的平均。dir = +1：面在 edge 的正方向那边；-1：反过来。找不到明显的峰（这个面只是擦边看到几个点）
/// 返回 false
bool face_position(const std::vector<double> & coords, double edge, int dir, double * face)
{
  constexpr double kBin = 0.01, kBand = 0.06;
  std::map<int, int> hist;
  int in_band = 0;
  for (double x : coords) {
    const double r = (x - edge) * dir;
    if (r > -0.02 && r < kBand) {
      ++hist[static_cast<int>(std::floor(r / kBin))];
      ++in_band;
    }
  }
  int best_k = 0, best = 0;
  for (const auto & kv : hist) {
    const int k = kv.first;
    const int sum = kv.second + (hist.count(k - 1) ? hist.at(k - 1) : 0) + (hist.count(k + 1) ? hist.at(k + 1) : 0);
    if (sum > best) {
      best = sum;
      best_k = k;
    }
  }
  // 均匀铺开时三格大约是 in_band * 3 / 8；要明显比这多才算面
  if (best < 2.0 * in_band * 3.0 / 8.0 || best < 10) {
    return false;
  }
  const double center = (best_k + 0.5) * kBin;
  double sum = 0.0;
  int n = 0;
  for (double x : coords) {
    const double r = (x - edge) * dir;
    if (std::abs(r - center) <= 0.015) {
      sum += r;
      ++n;
    }
  }
  *face = n > 0 ? edge + dir * (sum / n) : edge;
  return true;
}

struct SquareFit
{
  double x{0.0}, y{0.0}, yaw{0.0}, extent_a{0.0}, extent_b{0.0};
  bool reliable{false};
};

/// 投影集中度：点转 -theta 以后，u、v 各按 bin 分格，格子点数的平方和。
/// 转对了的时候，每个面的点都落在同一个 u（或 v）格里，平方和最大
double concentration(const std::vector<XY> & pts, double theta, double bin)
{
  const double c = std::cos(theta), s = std::sin(theta);
  std::unordered_map<int, int> hu, hv;
  for (const auto & p : pts) {
    ++hu[static_cast<int>(std::floor((c * p.first + s * p.second) / bin))];
    ++hv[static_cast<int>(std::floor((-s * p.first + c * p.second) / bin))];
  }
  double score = 0.0;
  for (const auto & kv : hu) {score += static_cast<double>(kv.second) * kv.second;}
  for (const auto & kv : hv) {score += static_cast<double>(kv.second) * kv.second;}
  return score;
}

/// 用看得到的一两个面拟合边长 side 的正方形（见头文件说明）
SquareFit fit_square(const SlicePts & slice_pts, double side, const Viewpoint & view)
{
  const std::vector<XY> & pts = slice_pts.xy;
  // 朝向：投影最集中的角度（不用"外接矩形面积最小"：两个面成 L 形时，转 45° 的外接矩形面积
  // 跟转对了一样大，分不出来）。先 1° 粗搜，再在最好的附近 0.1° 细搜。bin 2 cm ≈ 仿真 / 真雷达噪声
  constexpr double kBin = 0.02;
  double best_theta = 0.0, best_score = -1.0;
  for (int d = 0; d < 90; ++d) {
    const double sc = concentration(pts, d * kPi / 180.0, kBin);
    if (sc > best_score) {
      best_score = sc;
      best_theta = d * kPi / 180.0;
    }
  }
  const double coarse = best_theta;
  for (int k = -10; k <= 10; ++k) {
    const double t = coarse + k * 0.1 * kPi / 180.0;
    const double sc = concentration(pts, t, kBin);
    if (sc > best_score) {
      best_score = sc;
      best_theta = t;
    }
  }

  const Box2 b = rotated_box(pts, best_theta);
  const double c = std::cos(best_theta), s = std::sin(best_theta);
  const double su = c * view.x + s * view.y;
  const double sv = -s * view.x + c * view.y;

  std::vector<double> us, vs;
  us.reserve(pts.size());
  vs.reserve(pts.size());
  for (const auto & p : pts) {
    us.push_back(c * p.first + s * p.second);
    vs.push_back(-s * p.first + c * p.second);
  }

  // 每个轴：雷达在点的范围外 → 离雷达近的边（一定是看得到的面）往里推半个边长
  bool reliable = true;
  // 近边：那个面看到了（点挤在一起）就用面的位置；没看到（那条边只是另一个面的端头）就用分位数的边——
  // 端头的点是均匀铺开的，2% / 98% 分位只差几毫米。不要退回用"两头的中点"：雷达前倾时往车后方向的下视角
  // 越来越高，侧面看时方块靠车尾那一头只看得到顶上一小条，中点会偏向车头 2～3 cm（2026-10-05 host 扫描）
  auto axis_center = [&](const std::vector<double> & coords, double lo, double hi, double sensor) {
      double face = 0.0;
      if (sensor < lo) {
        return (face_position(coords, lo, +1, &face) ? face : lo) + side / 2.0;
      }
      if (sensor > hi) {
        return (face_position(coords, hi, -1, &face) ? face : hi) - side / 2.0;
      }
      if (hi - lo < 0.8 * side) {
        reliable = false;
      }
      // 雷达在两头之间：两头都看全了才取中点。前倾雷达往车后方向下视角越来越高，侧面看时靠车尾那一头
      // 只看得到顶上一小条、范围被截短，中点会偏 2～3 cm（2026-10-05 host 扫描）。哪头看到得更低（离台面近）
      // 就是看全的那头，用它往里推半个边长
      double z_lo = std::numeric_limits<double>::infinity(), z_hi = z_lo;
      for (size_t i = 0; i < coords.size(); ++i) {
        if (coords[i] <= lo + 0.03) {z_lo = std::min(z_lo, slice_pts.z[i]);}
        if (coords[i] >= hi - 0.03) {z_hi = std::min(z_hi, slice_pts.z[i]);}
      }
      if (z_hi - z_lo > 0.05) {
        return lo + side / 2.0;
      }
      if (z_lo - z_hi > 0.05) {
        return hi - side / 2.0;
      }
      return (lo + hi) / 2.0;
    };
  const double cu = axis_center(us, b.u_lo, b.u_hi, su);
  const double cv = axis_center(vs, b.v_lo, b.v_hi, sv);

  SquareFit f;
  f.x = c * cu - s * cv;
  f.y = s * cu + c * cv;
  double yaw = best_theta;
  while (yaw >= kPi / 4) {yaw -= kPi / 2;}
  while (yaw < -kPi / 4) {yaw += kPi / 2;}
  f.yaw = yaw;
  f.extent_a = b.u_hi - b.u_lo;
  f.extent_b = b.v_hi - b.v_lo;
  f.reliable = reliable;
  return f;
}

int64_t cell_key(int ix, int iy)
{
  return (static_cast<int64_t>(ix) << 32) ^ static_cast<int64_t>(static_cast<uint32_t>(iy));
}

}  // namespace

const char * to_string(BlockType type)
{
  switch (type) {
    case BlockType::Ground: return "GROUND";
    case BlockType::Sky: return "SKY";
    case BlockType::Unknown: return "UNKNOWN";
  }
  return "?";
}

bool decode_stack(double height, const DetectorParams & p, int * ground_layers, bool * has_sky)
{
  double best = std::numeric_limits<double>::infinity();
  for (int g = 0; g <= 3; ++g) {
    for (int s = 0; s <= 1; ++s) {
      if (g == 0 && s == 0) {
        continue;
      }
      const double err = std::abs(height - (g * p.ground_size + s * p.sky_size));
      if (err < best) {
        best = err;
        *ground_layers = g;
        *has_sky = (s == 1);
      }
    }
  }
  return best <= p.height_tolerance;
}

RegionResult detect_blocks(
  const std::vector<Point3> & points, const Region & region, const Viewpoint & view,
  const DetectorParams & p)
{
  RegionResult out;

  // ── 裁到区域 ──
  std::vector<Point3> inside;
  for (const auto & pt : points) {
    if (std::abs(pt.x - region.cx) <= region.half_x && std::abs(pt.y - region.cy) <= region.half_y &&
      pt.z > region.expected_surface_z - p.surface_band &&
      pt.z < region.expected_surface_z + p.max_height)
    {
      inside.push_back(pt);
    }
  }
  out.num_points = inside.size();

  // ── 1. 台面高度：1 cm 一格，点最多的那格 ──
  {
    std::map<int, std::vector<double>> bins;
    for (const auto & pt : inside) {
      if (std::abs(pt.z - region.expected_surface_z) <= p.surface_band) {
        bins[static_cast<int>(std::floor(pt.z / 0.01))].push_back(pt.z);
      }
    }
    size_t best = 0;
    int best_bin = 0;
    double z = region.expected_surface_z;
    for (const auto & kv : bins) {
      if (kv.second.size() > best) {
        best = kv.second.size();
        best_bin = kv.first;
        double sum = 0.0;
        for (double v : kv.second) {sum += v;}
        z = sum / static_cast<double>(kv.second.size());
      }
    }
    // 台面是一个"尖峰"：方块侧面的点在高度上是均匀铺开的，点数再多也不是尖峰
    // （从 L1 看 L2 时看不到台面，只有侧面，原来会把侧面上某一格当成台面）
    double neighbors = 0.0;
    int n_neighbors = 0;
    for (int d = 2; d <= 6; ++d) {
      for (int sign : {-1, 1}) {
        const auto it = bins.find(best_bin + sign * d);
        neighbors += it == bins.end() ? 0.0 : static_cast<double>(it->second.size());
        ++n_neighbors;
      }
    }
    const double neighbor_mean = neighbors / n_neighbors;
    out.surface_measured = best >= p.min_surface_points && best >= 3.0 * neighbor_mean;
    out.surface_z = out.surface_measured ? z : region.expected_surface_z;
  }

  // ── 覆盖率：区域里 5 cm 小格有点的比例 ──
  {
    const int nx = std::max(1, static_cast<int>(std::round(2 * region.half_x / p.coverage_cell)));
    const int ny = std::max(1, static_cast<int>(std::round(2 * region.half_y / p.coverage_cell)));
    std::vector<char> hit(static_cast<size_t>(nx * ny), 0);
    for (const auto & pt : inside) {
      if (pt.z < out.surface_z - 0.03) {
        continue;
      }
      const int ix = std::min(nx - 1, static_cast<int>((pt.x - region.cx + region.half_x) / p.coverage_cell));
      const int iy = std::min(ny - 1, static_cast<int>((pt.y - region.cy + region.half_y) / p.coverage_cell));
      if (ix >= 0 && iy >= 0) {
        hit[static_cast<size_t>(iy * nx + ix)] = 1;
      }
    }
    size_t n = 0;
    for (char h : hit) {n += h;}
    out.coverage = static_cast<double>(n) / static_cast<double>(hit.size());
  }

  // ── 2. 台面以上的点按 xy 栅格聚类 ──
  std::vector<Point3> above;
  for (const auto & pt : inside) {
    if (pt.z > out.surface_z + p.min_height && pt.z < out.surface_z + p.max_height) {
      above.push_back(pt);
    }
  }
  std::unordered_map<int64_t, std::vector<size_t>> cells;
  for (size_t i = 0; i < above.size(); ++i) {
    const int ix = static_cast<int>(std::floor(above[i].x / p.cluster_cell));
    const int iy = static_cast<int>(std::floor(above[i].y / p.cluster_cell));
    cells[cell_key(ix, iy)].push_back(i);
  }
  std::unordered_map<int64_t, int> label;
  std::vector<std::vector<size_t>> clusters;
  for (const auto & kv : cells) {
    if (label.count(kv.first)) {
      continue;
    }
    const int id = static_cast<int>(clusters.size());
    clusters.emplace_back();
    std::queue<int64_t> q;
    q.push(kv.first);
    label[kv.first] = id;
    while (!q.empty()) {
      const int64_t k = q.front();
      q.pop();
      const auto & members = cells[k];
      clusters[id].insert(clusters[id].end(), members.begin(), members.end());
      const int ix = static_cast<int>(k >> 32);
      const int iy = static_cast<int>(static_cast<int32_t>(k & 0xffffffff));
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          const int64_t nk = cell_key(ix + dx, iy + dy);
          if (cells.count(nk) && !label.count(nk)) {
            label[nk] = id;
            q.push(nk);
          }
        }
      }
    }
  }

  // ── 2b. 悬空的簇并到下面那一摞 ──
  // 天空块（200）比地块（350）小，居中放的时候它的侧面比下面地块的侧面往里缩 7.5 cm；从侧面看，
  // xy 投影是两条隔开的线，栅格聚类会分成两簇（2026-10-05 仿真：spot_03 的塔被拆成"2 层地块"+"悬空的
  // 天空块"）。东西都是放在台面上的，底部离台面很高的簇一定压在别的东西上 → 并到 xy 上最近的落地簇
  // 先把竖直方向有空档（≥ max_vertical_gap 没有点）的簇拆开：悬空的东西离一摞水平上很近时，xy 投影已经连成
  // 一簇了，不拆的话整簇被当成一个高度对不上的怪东西（2026-10-05 单测：举着的方块离地块 5 cm）。
  // 正常叠着的方块侧面是连着的，竖直方向没有空档
  {
    std::vector<std::vector<size_t>> split;
    for (auto & members : clusters) {
      std::sort(members.begin(), members.end(), [&](size_t a, size_t b) {return above[a].z < above[b].z;});
      size_t start = 0;
      for (size_t k = 1; k <= members.size(); ++k) {
        if (k == members.size() || above[members[k]].z - above[members[k - 1]].z > p.max_vertical_gap) {
          split.emplace_back(members.begin() + static_cast<std::ptrdiff_t>(start),
            members.begin() + static_cast<std::ptrdiff_t>(k));
          start = k;
        }
      }
    }
    clusters.swap(split);
  }

  auto pts_of = [&](const std::vector<size_t> & members, double z_min = -1e9) {
      SlicePts sp;
      for (size_t i : members) {
        if (above[i].z >= z_min) {
          sp.xy.emplace_back(above[i].x, above[i].y);
          sp.z.push_back(above[i].z);
        }
      }
      return sp;
    };

  // 比一块方块还大的簇，用更细的栅格（1.5 cm、4 邻接）再分一次：两样东西水平只隔 3～6 cm 时 3 cm 栅格会把它们
  // 连成一簇（2026-10-05 助教仿真：举着的方块贴着旁边那摞、只隔 5 cm，高度又跟那摞的顶重叠 5 cm，高度空档也拆不开）。
  // 拆出来至少两块各占 15% 以上才算数，不然（真的挨着、或者稀疏的面被拆碎）保持原样
  {
    std::vector<std::vector<size_t>> refined;
    for (auto & members : clusters) {
      // 主方向上的跨度（2% / 98% 分位）
      double mx = 0.0, my = 0.0;
      for (size_t i : members) {mx += above[i].x; my += above[i].y;}
      mx /= std::max<size_t>(members.size(), 1);
      my /= std::max<size_t>(members.size(), 1);
      double sxx = 0.0, sxy = 0.0, syy = 0.0;
      for (size_t i : members) {
        const double dx = above[i].x - mx, dy = above[i].y - my;
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
      }
      const double ang = 0.5 * std::atan2(2 * sxy, sxx - syy);
      std::vector<double> proj;
      for (size_t i : members) {
        proj.push_back(std::cos(ang) * (above[i].x - mx) + std::sin(ang) * (above[i].y - my));
      }
      const double span = proj.empty() ? 0.0 :
        quantile(proj, 0.98) - quantile(proj, 0.02);
      // 只对落地的簇再分：悬空的（举着的天空块）立面和顶面点之间本来就隔得开，细栅格会把一块拆成两块
      // （助教 010026c 复测情况 B）
      std::vector<double> zs;
      for (size_t i : members) {zs.push_back(above[i].z);}
      const bool grounded = !zs.empty() && quantile(zs, 0.05) <= out.surface_z + p.min_stack_height;
      if (members.size() < p.min_cluster_points || !grounded ||
        span <= p.ground_size * p.size_tolerance + 0.03)
      {
        refined.push_back(members);
        continue;
      }
      // 细栅格连通
      std::unordered_map<int64_t, std::vector<size_t>> fine;
      for (size_t i : members) {
        fine[cell_key(static_cast<int>(std::floor(above[i].x / p.fine_cell)),
          static_cast<int>(std::floor(above[i].y / p.fine_cell)))].push_back(i);
      }
      std::unordered_map<int64_t, int> fl;
      std::vector<std::vector<size_t>> parts;
      for (const auto & kv : fine) {
        if (fl.count(kv.first)) {continue;}
        const int id = static_cast<int>(parts.size());
        parts.emplace_back();
        std::queue<int64_t> q;
        q.push(kv.first);
        fl[kv.first] = id;
        while (!q.empty()) {
          const int64_t k = q.front();
          q.pop();
          const auto & mem = fine[k];
          parts[id].insert(parts[id].end(), mem.begin(), mem.end());
          const int ix = static_cast<int>(k >> 32);
          const int iy = static_cast<int>(static_cast<int32_t>(k & 0xffffffff));
          const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
          for (const auto & d : nb) {
            const int64_t nk = cell_key(ix + d[0], iy + d[1]);
            if (fine.count(nk) && !fl.count(nk)) {
              fl[nk] = id;
              q.push(nk);
            }
          }
        }
      }
      size_t big = 0;
      for (const auto & part : parts) {big += part.size() >= 0.15 * members.size();}
      if (big >= 2) {
        // 小碎片并给最近的大块（按点到点最近距离太贵，按重心）
        std::vector<std::pair<double, double>> centers(parts.size());
        for (size_t k = 0; k < parts.size(); ++k) {
          double cx = 0.0, cy = 0.0;
          for (size_t i : parts[k]) {cx += above[i].x; cy += above[i].y;}
          centers[k] = {cx / parts[k].size(), cy / parts[k].size()};
        }
        std::vector<std::vector<size_t>> kept;
        std::vector<size_t> kept_src;
        for (size_t k = 0; k < parts.size(); ++k) {
          if (parts[k].size() >= 0.15 * members.size()) {
            kept.push_back(parts[k]);
            kept_src.push_back(k);
          }
        }
        for (size_t k = 0; k < parts.size(); ++k) {
          if (parts[k].size() >= 0.15 * members.size()) {continue;}
          size_t best = 0;
          double best_d = std::numeric_limits<double>::infinity();
          for (size_t j = 0; j < kept.size(); ++j) {
            const auto & c2 = centers[kept_src[j]];
            const double d = std::hypot(centers[k].first - c2.first, centers[k].second - c2.second);
            if (d < best_d) {best_d = d; best = j;}
          }
          kept[best].insert(kept[best].end(), parts[k].begin(), parts[k].end());
        }
        refined.insert(refined.end(), kept.begin(), kept.end());
      } else {
        refined.push_back(members);
      }
    }
    clusters.swap(refined);
  }

  {
    std::vector<double> z_lo(clusters.size(), 0.0);
    std::vector<std::vector<XY>> cell_xy(clusters.size());
    for (size_t c = 0; c < clusters.size(); ++c) {
      std::unordered_map<int64_t, char> seen;
      for (size_t i : clusters[c]) {
        const int ix = static_cast<int>(std::floor(above[i].x / p.cluster_cell));
        const int iy = static_cast<int>(std::floor(above[i].y / p.cluster_cell));
        if (seen.emplace(cell_key(ix, iy), 1).second) {
          cell_xy[c].emplace_back((ix + 0.5) * p.cluster_cell, (iy + 0.5) * p.cluster_cell);
        }
      }
    }
    std::vector<double> z_top(clusters.size(), 0.0);
    for (size_t c = 0; c < clusters.size(); ++c) {
      std::vector<double> zs;
      for (size_t i : clusters[c]) {zs.push_back(above[i].z);}
      z_lo[c] = zs.empty() ? 0.0 : quantile(zs, 0.05);
      z_top[c] = zs.empty() ? 0.0 : quantile(zs, 0.99);
    }
    const double floating = out.surface_z + p.min_stack_height;
    for (size_t c = 0; c < clusters.size(); ++c) {
      if (clusters[c].size() < p.min_cluster_points || z_lo[c] <= floating) {
        continue;
      }
      int best = -1;
      double best_d = p.max_set_back;
      for (size_t b = 0; b < clusters.size(); ++b) {
        if (b == c || clusters[b].size() < p.min_cluster_points || z_lo[b] > floating) {
          continue;
        }
        // 竖直方向也要对得上：悬空簇的底 ≈ 那一摞的顶，才是"压在它上面"。否则可能是别的车夹爪举着的方块、
        // 我们自己的夹爪在视野里——只按水平距离并的话层数会多算（助教 10-05 提的边界情况）
        if (std::abs(z_lo[c] - z_top[b]) > p.height_tolerance) {
          continue;
        }
        // 水平方向要撑得住：上面那块的中心要落在下面那摞顶层的脚印里（偏 < 半个地块边长，再留 3 cm）。
        // 真叠着的方块偏多了放不住；举着从旁边过的方块中心离得远（助教 10-05 情况 A：偏 40 cm）
        {
          const auto top_pts = pts_of(clusters[b], z_top[b] - 0.10);
          const auto up_pts = pts_of(clusters[c]);
          if (top_pts.size() >= 8 && up_pts.size() >= 8) {
            const auto fb = fit_square(top_pts, p.ground_size, view);
            const auto fu0 = fit_square(up_pts, p.ground_size, view);
            const bool up_is_sky = std::max(fu0.extent_a, fu0.extent_b) < 0.5 * (p.sky_size + p.ground_size);
            const auto fu = up_is_sky ? fit_square(up_pts, p.sky_size, view) : fu0;
            if (std::hypot(fu.x - fb.x, fu.y - fb.y) > p.ground_size / 2 + 0.03) {
              continue;
            }
          }
        }
        for (const auto & u : cell_xy[c]) {
          for (const auto & v : cell_xy[b]) {
            const double d = std::hypot(u.first - v.first, u.second - v.second);
            if (d < best_d) {
              best_d = d;
              best = static_cast<int>(b);
            }
          }
        }
      }
      if (best >= 0) {
        auto & base = clusters[static_cast<size_t>(best)];
        base.insert(base.end(), clusters[c].begin(), clusters[c].end());
        clusters[c].clear();
      }
    }
  }

  // 判断"悬空"前先确认这一簇的底本来应该看得到：前倾雷达侧面往下只有 -7°，离近了方块下半截本来就看不到，
  // 底看起来也"离台面很高"（2026-10-05 host 扫描，侧面近处）。办法：沿**同一条视线方向**（方位角差 < 2°）、
  // 比这一簇最低的点**近一点**（0～0.5 m）的地方看到了台面 → 更陡的视线都打到地上了，这一簇的底如果真放在
  // 台面上也一定看得到。只看"旁边 12 cm 有没有台面"不行：车头方向往下看得更低，方块靠车头那一头旁边总有台面
  constexpr double kAzBin = 0.5 * kPi / 180;
  std::unordered_map<int, std::vector<double>> floor_by_az;   // 方位角格 → 台面点离雷达的水平距离
  for (const auto & pt : inside) {
    if (std::abs(pt.z - out.surface_z) < 0.02) {
      const double az = std::atan2(pt.y - view.y, pt.x - view.x);
      floor_by_az[static_cast<int>(std::floor(az / kAzBin))].push_back(
        std::hypot(pt.x - view.x, pt.y - view.y));
    }
  }
  auto bottom_should_be_visible = [&](const std::vector<size_t> & members, double bottom_z) {
      int supported = 0;
      for (size_t i : members) {
        if (above[i].z > bottom_z + 0.05) {
          continue;   // 只看最低的那一圈
        }
        const double d_pt = std::hypot(above[i].x - view.x, above[i].y - view.y);
        const int bin = static_cast<int>(std::floor(std::atan2(above[i].y - view.y, above[i].x - view.x) / kAzBin));
        bool found = false;
        for (int k = bin - 4; k <= bin + 4 && !found; ++k) {   // ±2°
          const auto it = floor_by_az.find(k);
          if (it == floor_by_az.end()) {
            continue;
          }
          for (double d : it->second) {
            if (d < d_pt - 0.02 && d > d_pt - 0.5) {
              found = true;
              break;
            }
          }
        }
        supported += found;
        if (supported >= 3) {
          return true;
        }
      }
      return false;
    };

  // ── 3 / 4. 每一摞：总高度 → 组合；每层切片拟合 ──
  struct Stack
  {
    std::vector<Block> blocks;
    double top;
    bool clipped;
    int ground;
    bool sky;
    bool known;
    bool floating;     // 悬空、没压在任何一摞上（排在最后，不当"区域中心那一摞"）
    double dist_to_center;
  };
  std::vector<Stack> stacks;
  std::vector<std::string> notes;
  for (const auto & members : clusters) {
    if (members.size() < p.min_cluster_points) {
      continue;
    }
    std::vector<double> zs;
    for (size_t i : members) {zs.push_back(above[i].z);}
    const double top_z = quantile(zs, 0.99);
    const double bottom_z = quantile(zs, 0.05);
    const double height = top_z - out.surface_z;
    if (height < p.min_stack_height) {
      continue;  // 围栏、台面边缘这种矮东西
    }
    // 没并到任何一摞上的悬空簇：不是放在台面上的方块（被举着的、夹爪……），不数层数。
    // 前提是它的底本来应该看得到（看不到的话底只是出了视场）
    const bool floating_alone =
      bottom_z - out.surface_z > p.min_stack_height && bottom_should_be_visible(members, bottom_z);

    Stack st{};
    st.top = height;
    // 最高点贴着雷达视场上沿 → 上面可能还有看不到的
    double nearest = std::numeric_limits<double>::infinity();
    for (size_t i : members) {
      nearest = std::min(nearest, std::hypot(above[i].x - view.x, above[i].y - view.y));
    }
    st.clipped = std::atan2(top_z - view.z, nearest) > view.max_elevation - 0.03;
    st.floating = floating_alone;
    st.known = !floating_alone && decode_stack(height, p, &st.ground, &st.sky);

    auto slice = [&](double z_lo, double z_hi) {
        SlicePts out_pts;
        for (size_t i : members) {
          if (above[i].z >= z_lo && above[i].z <= z_hi) {
            out_pts.xy.emplace_back(above[i].x, above[i].y);
            out_pts.z.push_back(above[i].z);
          }
        }
        return out_pts;
      };

    if (!st.known) {
      Block b;
      b.type = BlockType::Unknown;
      const auto xy = slice(out.surface_z, top_z + 0.01);
      const auto f = fit_square(xy, p.ground_size, view);
      b.x = f.x;
      b.y = f.y;
      b.yaw = f.yaw;
      b.extent_a = f.extent_a;
      b.extent_b = f.extent_b;
      b.z_bottom = floating_alone ? bottom_z : out.surface_z;
      b.num_points = xy.size();
      st.blocks.push_back(b);
      char buf[200];
      if (floating_alone) {
        std::snprintf(
          buf, sizeof(buf), "(%.2f, %.2f) 有悬空的东西（底离台面 %.2f m，没压在哪一摞上：被举着的方块？）",
          f.x, f.y, bottom_z - out.surface_z);
      } else {
        std::snprintf(buf, sizeof(buf), "(%.2f, %.2f) 有高 %.2f m 的东西对不上方块组合", f.x, f.y, height);
      }
      notes.emplace_back(buf);
    } else {
      const int total = st.ground + (st.sky ? 1 : 0);
      double bottom = out.surface_z;
      for (int k = 0; k < total; ++k) {
        const bool is_sky = st.sky && k == total - 1;
        const double side = is_sky ? p.sky_size : p.ground_size;
        // 上下各让 3 cm：不要底下那层的顶面 / 自己的顶面（顶面是实心的一片，会把朝向估歪）
        const auto xy = slice(bottom + 0.03, bottom + side - 0.03);
        Block b;
        b.type = is_sky ? BlockType::Sky : BlockType::Ground;
        b.layer = k + 1;
        b.z_bottom = bottom;
        b.num_points = xy.size();
        if (xy.size() >= 8) {
          const auto f = fit_square(xy, side, view);
          b.x = f.x;
          b.y = f.y;
          b.yaw = f.yaw;
          b.extent_a = f.extent_a;
          b.extent_b = f.extent_b;
          b.center_reliable = f.reliable;
          if (f.extent_a > side * p.size_tolerance + 0.03 || f.extent_b > side * p.size_tolerance + 0.03) {
            b.type = BlockType::Unknown;
            b.center_reliable = false;
            char buf[160];
            std::snprintf(
              buf, sizeof(buf), "(%.2f, %.2f) 第 %d 层看到 %.2f x %.2f m，比方块大（两块挨着？）",
              f.x, f.y, k + 1, f.extent_a, f.extent_b);
            notes.emplace_back(buf);
          }
        } else if (!st.blocks.empty()) {
          // 这一层几乎没看到点（被挡 / 视场外），位置先跟下面那层
          b.x = st.blocks.back().x;
          b.y = st.blocks.back().y;
          b.yaw = st.blocks.back().yaw;
        } else {
          // 最下面一层就没看到点（被前面的东西挡住）：用整摞的点估个位置，标不可靠
          const auto f = fit_square(slice(out.surface_z, top_z + 0.01), side, view);
          b.x = f.x;
          b.y = f.y;
          b.yaw = f.yaw;
        }
        st.blocks.push_back(b);
        bottom += side;
      }
    }
    st.dist_to_center = std::hypot(st.blocks.front().x - region.cx, st.blocks.front().y - region.cy);
    stacks.push_back(std::move(st));
  }

  std::sort(stacks.begin(), stacks.end(), [](const Stack & a, const Stack & b) {
      if (a.floating != b.floating) {
        return !a.floating;
      }
      return a.dist_to_center < b.dist_to_center;
    });
  for (const auto & st : stacks) {
    out.blocks.insert(out.blocks.end(), st.blocks.begin(), st.blocks.end());
  }
  // 摘要按离区域中心最近的那一摞（建造点上就是塔本身）
  if (!stacks.empty() && !stacks.front().floating) {
    const auto & main = stacks.front();
    out.ground_layers = main.known ? main.ground : 0;
    if (!main.known && main.clipped) {
      // 顶部出了视场、高度对不上组合：至少有看到的这么多层地块。报 0 的话决策会按空的去放，砸到塔上
      out.ground_layers = static_cast<int>((main.top + p.height_tolerance) / p.ground_size);
    }
    out.has_sky = main.known && main.sky;
    out.top_height = main.top;
    out.top_maybe_clipped = main.clipped;
    if (main.clipped) {
      notes.emplace_back("最高点贴着雷达视场上沿，上面可能还有看不到的（退远一点再看）");
    }
  }

  out.visible = out.coverage >= p.min_coverage || !stacks.empty();
  if (!out.visible) {
    notes.emplace_back("看不到这个区域（覆盖率太低）");
  } else if (!out.surface_measured) {
    notes.emplace_back("看不到台面，台面高度用的预期值");
  }
  for (size_t i = 0; i < notes.size(); ++i) {
    out.note += (i ? "；" : "") + notes[i];
  }
  return out;
}

}  // namespace rc2027_perception
