// 方块检测"最近多少距离能稳定认出来"的 host 扫描（2026-10-05，用户要求：正面看、侧面看分开测）。
//
// 不依赖 ROS / Gazebo：自己按 Mid-360 的视场打射线（-7°～52°，水平 360°，0.46° 一格 ≈ 5 帧 10 万点，
// 每次随机抖一下格子，模拟非重复扫描），场景只有平台面 + 方块（解析求交），雷达按 robot_params.yaml：
// 车头前 0.2 m、离台面 0.23 m、前下倾 30°。然后直接调 detect_blocks，看认得对不对。
//
// "认对"= 这一摞的层数 / 有没有天空块对，块数对，每块中心误差 < 2 cm、朝向误差 < 3°、中心可靠。
// 每个距离随机摆 trials 次（方块偏 ±3 cm、转 0～45°、点加 1 cm 噪声），报成功率（%），后面跟失败最多的原因：
//   L 层数不对  N 块数不对  U 中心不可靠（只看到一部分面）  C 中心误差 ≥ 2 cm  Y 朝向误差超（地块 3°、天空块 5°）
//
// 编译（host）：g++ -std=c++17 -O2 -Iinclude tools/view_distance_sweep.cpp src/block_detector.cpp -o sweep
// 用法：./sweep [lidar_height=0.23] [pitch_deg=30] [trials=20]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "rc2027_perception/block_detector.hpp"

using namespace rc2027_perception;

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Box3 { double cx, cy, z0, s, yaw; };

// 射线和绕 z 转了 yaw 的方块求交（转到方块系做 slab）
double hit_box(const double o[3], const double d[3], const Box3 & b)
{
  const double c = std::cos(-b.yaw), s = std::sin(-b.yaw);
  const double ox = c * (o[0] - b.cx) - s * (o[1] - b.cy), oy = s * (o[0] - b.cx) + c * (o[1] - b.cy);
  const double dx = c * d[0] - s * d[1], dy = s * d[0] + c * d[1];
  const double lo[3] = {-b.s / 2, -b.s / 2, b.z0}, hi[3] = {b.s / 2, b.s / 2, b.z0 + b.s};
  const double oo[3] = {ox, oy, o[2]}, dd[3] = {dx, dy, d[2]};
  double t0 = 0, t1 = 1e9;
  for (int k = 0; k < 3; ++k) {
    if (std::abs(dd[k]) < 1e-12) {
      if (oo[k] < lo[k] || oo[k] > hi[k]) {return -1;}
      continue;
    }
    double a = (lo[k] - oo[k]) / dd[k], bb = (hi[k] - oo[k]) / dd[k];
    if (a > bb) {std::swap(a, bb);}
    t0 = std::max(t0, a);
    t1 = std::min(t1, bb);
    if (t0 > t1) {return -1;}
  }
  return t0 > 1e-6 ? t0 : -1;
}

struct Case { const char * name; std::vector<bool> layers; };   // false = 地块，true = 天空块（只能最上面）

}  // namespace

int main(int argc, char ** argv)
{
  const double lidar_h = argc > 1 ? std::atof(argv[1]) : 0.23;
  const double pitch = (argc > 2 ? std::atof(argv[2]) : 30.0) * kPi / 180;
  const int trials = argc > 3 ? std::atoi(argv[3]) : 20;
  // 正前方往上能看到的角度 = 52° - 俯仰
  const double max_up_front = 52.0 * kPi / 180 - pitch;
  std::printf("雷达离台面 %.2f m、前倾 %.0f°（正前方往上看到 %.0f°、往下 %.0f°；正侧面 -7°～52°）\n",
    lidar_h, pitch * 180 / kPi, max_up_front * 180 / kPi, -(7 * kPi / 180 + pitch) * 180 / kPi);

  const std::vector<Case> cases = {
    {"单块地块 350", {false}}, {"天空块 200", {true}}, {"2 地一摞 700", {false, false}},
    {"地+天 550", {false, true}}, {"完整塔 900", {false, false, true}}};
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> off(-0.03, 0.03), yawd(0, kPi / 4), jit(0, 0.46);
  std::normal_distribution<double> noise(0, 0.01);

  for (const auto & view : {std::string("正面"), std::string("侧面")}) {
    std::printf("\n== %s看（距离 = 车体中心到方块中心）==\n%-14s", view.c_str(), "");
    std::vector<double> dists;
    for (double d = 0.45; d <= 2.51; d += (d < 1.0 ? 0.05 : 0.1)) {dists.push_back(d);}
    for (double d : dists) {std::printf("%5.2f ", d);}
    std::printf("\n");
    for (const auto & cs : cases) {
      std::printf("%-14s", cs.name);
      double stable_from = -1;
      for (double d : dists) {
        // 正面：方块在车头正前方；侧面：车体中心正左方
        const double bx = view == "正面" ? d : 0.0, by = view == "正面" ? 0.0 : d;
        // 放不下（雷达伸到 0.25 m，车体半宽 0.15）
        const double half = cs.layers[0] ? 0.10 : 0.175;
        if ((view == "正面" && d < 0.25 + half * std::sqrt(2.0) + 0.02) ||
          (view == "侧面" && d < 0.15 + half * std::sqrt(2.0) + 0.02))
        {
          std::printf("    - ");
          continue;
        }
        int ok = 0;
        std::map<char, int> reasons;
        for (int t = 0; t < trials; ++t) {
          std::vector<Box3> boxes;
          const double cx = bx + off(rng), cy = by + off(rng);
          double z = 0;
          std::vector<double> yaws;
          for (bool sky : cs.layers) {
            const double s = sky ? 0.2 : 0.35, yw = yawd(rng);
            boxes.push_back({cx + 0.3 * off(rng), cy + 0.3 * off(rng), z, s, yw});
            z += s;
          }
          // 打射线
          const double lx = 0.2, ly = 0, lz = lidar_h;
          const double o[3] = {lx, ly, lz};
          std::vector<Point3> pts;
          const double ja = jit(rng), je = jit(rng);
          for (double el = -7 + je; el <= 52; el += 0.46) {
            for (double az = ja; az < 360; az += 0.46) {
              const double e = el * kPi / 180, a = az * kPi / 180;
              double v[3] = {std::cos(e) * std::cos(a), std::cos(e) * std::sin(a), std::sin(e)};
              const double d0 = v[0] * std::cos(pitch) + v[2] * std::sin(pitch);
              const double d2 = -v[0] * std::sin(pitch) + v[2] * std::cos(pitch);
              v[0] = d0;
              v[2] = d2;
              double tbest = v[2] < 0 ? -lz / v[2] : 1e9;   // 台面 z = 0
              for (const auto & b : boxes) {
                const double tb = hit_box(o, v, b);
                if (tb > 0 && tb < tbest) {tbest = tb;}
              }
              if (tbest < 0.3 || tbest > 8.0) {continue;}   // block_detector 的 min_range 0.3
              pts.push_back({static_cast<float>(lx + tbest * v[0] + noise(rng)),
                static_cast<float>(ly + tbest * v[1] + noise(rng)), static_cast<float>(lz + tbest * v[2] + noise(rng))});
            }
          }
          Region r;
          r.cx = bx;
          r.cy = by;
          r.half_x = r.half_y = 0.40;
          r.expected_surface_z = 0.0;
          Viewpoint vp;
          vp.x = lx;
          vp.y = ly;
          vp.z = lz;
          vp.max_elevation = max_up_front;
          const auto res = detect_blocks(pts, r, vp);
          const int g = static_cast<int>(cs.layers.size()) - (cs.layers.back() ? 1 : 0);
          // 失败原因（只记第一个）：L 层数 / N 块数 / U 中心不可靠 / C 中心误差 ≥ 2 cm / Y 朝向误差超（地块 3°、天空块 5°）
          char why = 0;
          if (res.ground_layers != g || res.has_sky != cs.layers.back()) {
            why = 'L';
          } else if (res.blocks.size() != boxes.size()) {
            why = 'N';
          }
          for (size_t k = 0; !why && k < boxes.size(); ++k) {
            const auto & bk = res.blocks[k];
            double dy = std::fmod(bk.yaw - boxes[k].yaw, kPi / 2);
            if (dy > kPi / 4) {dy -= kPi / 2;}
            if (dy < -kPi / 4) {dy += kPi / 2;}
            const double yaw_tol = (cs.layers[k] ? 5.0 : 3.0) * kPi / 180;
            if (!bk.center_reliable) {
              why = 'U';
            } else if (std::hypot(bk.x - boxes[k].cx, bk.y - boxes[k].cy) >= 0.02) {
              why = 'C';
            } else if (std::abs(dy) >= yaw_tol) {
              why = 'Y';
            }
          }
          ok += !why;
          if (why) {++reasons[why];}
        }
        const double rate = static_cast<double>(ok) / trials;
        // 成功率 + 失败最多的原因（成功率 < 90% 时）
        char main_why = ' ';
        int most = 0;
        for (const auto & kv : reasons) {
          if (kv.second > most) {
            most = kv.second;
            main_why = kv.first;
          }
        }
        std::printf("%4.0f%c", rate * 100, rate < 0.9 ? main_why : ' ');
        // "稳定"：从这个距离起成功率 ≥ 90%，而且更远处不会掉到 80% 以下（随机摆 20～30 次，±10% 是抽样波动）
        if (rate >= 0.9 && stable_from < 0) {stable_from = d;}
        if (rate < 0.8) {stable_from = -1;}
      }
      if (stable_from > 0) {
        std::printf("   ← %.2f m 起稳定", stable_from);
      }
      std::printf("\n");
    }
  }
  return 0;
}
