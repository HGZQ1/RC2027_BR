// map_matcher 的单测——用仓库里真实的仿真先验地图（rc2027_sim_bridge/maps/prior_map_sim.pcd），
// 不用 ROS，host 上 g++ + Eigen + OpenMP 就能跑。用法：test_map_matcher <prior_map.pcd>
//
// "扫描"是从地图里截出来的（车周围 8 m、随机抽 50% 的点、加 1 cm 噪声），再用一个已知的
// map -> lidar_odom 变到 lidar_odom 系。给一个故意偏了的初值，看配准能不能拉回真值；
// 偏太多 / 放到镜像那边，验收必须拒绝。这证明的是配准 + 验收逻辑本身，不代表仿真/真车
// 点云（有遮挡、有动态物体）也一样好，那要在 Gazebo 里看。

#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <string>

#include "rc2027_navigation/map_matcher.hpp"

using namespace rc2027_navigation;

namespace
{

int g_failures = 0;

void check(bool condition, const std::string & what)
{
  if (!condition) {
    std::cerr << "FAIL: " << what << std::endl;
    ++g_failures;
  } else {
    std::cout << "ok: " << what << std::endl;
  }
}

Transform3D make(double x, double y, double z, double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  Transform3D t;
  t.x = x;
  t.y = y;
  t.z = z;
  t.qx = sr * cp * cy - cr * sp * sy;
  t.qy = cr * sp * cy + sr * cp * sy;
  t.qz = cr * cp * sy - sr * sp * cy;
  t.qw = cr * cp * cy + sr * sp * sy;
  return t;
}

Eigen::Vector3f apply(const Transform3D & t, const Eigen::Vector3f & p)
{
  const auto r = compose(t, make(p.x(), p.y(), p.z(), 0, 0, 0));
  return Eigen::Vector3f(r.x, r.y, r.z);
}

// 车在 map 里 (cx, cy)，截出周围 radius 内的地图点当"扫描"，变到 lidar_odom 系
Points fake_scan(
  const Points & map, double cx, double cy, double radius, const Transform3D & true_map_to_odom)
{
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> keep(0.0f, 1.0f);
  std::normal_distribution<float> noise(0.0f, 0.01f);
  const auto odom_from_map = inverse(true_map_to_odom);
  Points scan;
  for (const auto & p : map) {
    if (std::hypot(p.x() - cx, p.y() - cy) > radius || keep(rng) > 0.5f) {
      continue;
    }
    const Eigen::Vector3f n(noise(rng), noise(rng), noise(rng));
    scan.push_back(apply(odom_from_map, p + n));
  }
  return scan;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "用法: test_map_matcher <prior_map.pcd>" << std::endl;
    return 2;
  }

  // ── PCD 读取 ──
  const auto map = load_pcd_xyz(argv[1]);
  check(map.size() == 35744, "读 PCD: " + std::to_string(map.size()) + " 点");

  const auto t0 = std::chrono::steady_clock::now();
  MapMatcher matcher(map);
  const auto t1 = std::chrono::steady_clock::now();
  std::cout << "   地图预处理 " << std::chrono::duration<double>(t1 - t0).count() * 1000
            << " ms，降采样后 " << matcher.map_size() << " 点" << std::endl;

  // 车在红方 BR 起点附近；Point-LIO 的 lidar_odom 原点在开机时 IMU 处（离地 0.23 m），
  // 朝向跟场地差了任意一个角度
  const double cx = -4.2, cy = -4.9;
  const Transform3D true_map_to_odom = make(-4.35, -5.15, 0.23, 0.0, 0.0, 1.62);
  const Transform3D odom_to_base = compose(inverse(true_map_to_odom), make(cx, cy, 0, 0, 0, 1.5));
  const auto scan = fake_scan(map, cx, cy, 8.0, true_map_to_odom);
  std::cout << "   模拟扫描 " << scan.size() << " 点" << std::endl;

  AcceptParams accept;

  // ── 1. 小偏差：初值差 12 cm、3°，应该拉回真值并接受 ──
  {
    // 在车体处施加偏差：init = 真值 · (车体处偏一点)
    const auto base_true = compose(true_map_to_odom, odom_to_base);
    const auto base_off = compose(base_true, make(0.10, -0.07, 0.0, 0, 0, 3.0 * M_PI / 180));
    const auto init = compose(base_off, inverse(odom_to_base));

    const auto s = std::chrono::steady_clock::now();
    const auto r = matcher.match(scan, init);
    const auto e = std::chrono::steady_clock::now();
    const auto c_init = correction_between(true_map_to_odom, init, odom_to_base);
    const auto c_err = correction_between(true_map_to_odom, r.map_to_odom, odom_to_base);
    const auto c = correction_between(init, r.map_to_odom, odom_to_base);
    std::cout << "   配准 " << std::chrono::duration<double>(e - s).count() * 1000 << " ms，"
              << r.iterations << " 次迭代，" << r.num_points << " 点，内点 " << r.inlier_ratio
              << "，rmse " << r.rmse << " m" << std::endl;
    std::cout << "   初值误差 " << c_init.xy() * 100 << " cm / " << c_init.dyaw * 180 / M_PI
              << "°，配准后误差 " << c_err.xy() * 100 << " cm / " << c_err.dyaw * 180 / M_PI
              << "°" << std::endl;
    check(r.converged, "小偏差: 收敛");
    check(c_err.xy() < 0.01, "小偏差: 位置误差 < 1 cm");
    check(std::abs(c_err.dyaw) < 0.2 * M_PI / 180, "小偏差: 朝向误差 < 0.2°");
    check(r.inlier_ratio > 0.9, "小偏差: 内点比例 > 0.9");
    const auto d = decide(r, c, accept);
    check(d.accepted, "小偏差: 验收通过 " + d.reason);
  }

  // ── 2. 大偏差 0.8 m（比如 start_box 给错了）：不管配到哪，修正量都超 0.3 m，必须拒绝 ──
  {
    const auto base_true = compose(true_map_to_odom, odom_to_base);
    const auto init = compose(
      compose(base_true, make(0.8, 0.0, 0.0, 0, 0, 0)), inverse(odom_to_base));
    const auto r = matcher.match(scan, init);
    const auto d = decide(r, correction_between(init, r.map_to_odom, odom_to_base), accept);
    std::cout << "   大偏差: " << (d.accepted ? "接受" : "拒绝：" + d.reason) << std::endl;
    check(!d.accepted, "大偏差 0.8 m: 拒绝");
  }

  // ── 3. 镜像：初值放到蓝方对称位置，必须拒绝 ──
  {
    const auto mirrored = make(4.35, -5.15, 0.23, 0.0, 0.0, M_PI - 1.62);
    const auto r = matcher.match(scan, mirrored);
    const auto d = decide(r, correction_between(mirrored, r.map_to_odom, odom_to_base), accept);
    std::cout << "   镜像: " << (d.accepted ? "接受" : "拒绝：" + d.reason) << std::endl;
    // 镜像那边如果"配得很好"、修正量又小，就是真的会误判——验收会接受，这一条会失败
    const auto c_err = correction_between(true_map_to_odom, r.map_to_odom, odom_to_base);
    check(!d.accepted || c_err.xy() < 0.05, "镜像: 拒绝（或者自己配回了真值）");
  }

  // ── 3b. 车头摆反（初值差 180°）：必须拒绝；转 180° 再配就能对上——field_localizer 拿这个提示操作员 ──
  {
    const auto flipped_init = flip_base_yaw(true_map_to_odom, odom_to_base);
    const auto c0 = correction_between(true_map_to_odom, flipped_init, odom_to_base);
    check(std::abs(std::abs(c0.dyaw) - M_PI) < 1e-6 && c0.xy() < 1e-6, "flip_base_yaw: 车体原地转 180°");
    const auto r = matcher.match(scan, flipped_init);
    const auto d = decide(r, correction_between(flipped_init, r.map_to_odom, odom_to_base), accept);
    std::cout << "   摆反: " << (d.accepted ? "接受" : "拒绝：" + d.reason) << std::endl;
    check(!d.accepted, "车头摆反: 拒绝");
    const auto retry_init = flip_base_yaw(flipped_init, odom_to_base);
    const auto r2 = matcher.match(scan, retry_init);
    const auto d2 = decide(r2, correction_between(retry_init, r2.map_to_odom, odom_to_base), accept);
    check(d2.accepted, "车头摆反: 转回 180° 再配能对上（用来提示操作员）");
  }

  // ── 3c. 场外的点（场馆墙）：不裁的话内点比例掉下去被拒；裁到场地范围就正常 ──
  {
    auto with_walls = scan;
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-9.0f, 9.0f), h(0.0f, 4.0f);
    const auto odom_from_map = inverse(true_map_to_odom);
    for (int i = 0; i < 15000; ++i) {   // 场地外 9 m 处四面墙上的点
      const float a = u(rng);
      const Eigen::Vector3f w[4] = {{9.0f, a, h(rng)}, {-9.0f, a, h(rng)}, {a, 9.0f, h(rng)}, {a, -9.0f, h(rng)}};
      with_walls.push_back(apply(odom_from_map, w[i % 4]));
    }
    const auto base_true = compose(true_map_to_odom, odom_to_base);
    const auto init = compose(compose(base_true, make(0.10, -0.07, 0.0, 0, 0, 3.0 * M_PI / 180)),
        inverse(odom_to_base));
    const auto r_nocrop = matcher.match(with_walls, init);
    MatchParams cp;
    cp.crop_half_size = 5.6;
    MapMatcher cropped_matcher(map, cp);
    const auto r = cropped_matcher.match(with_walls, init);
    const auto err = correction_between(true_map_to_odom, r.map_to_odom, odom_to_base);
    std::cout << "   场馆墙: 不裁内点 " << r_nocrop.inlier_ratio << "，裁到 ±5.6 m 内点 " << r.inlier_ratio
              << "，误差 " << err.xy() * 100 << " cm" << std::endl;
    check(r_nocrop.inlier_ratio < accept.min_inlier_ratio, "场馆墙: 不裁会被内点比例拒掉");
    check(r.inlier_ratio > 0.9 && err.xy() < 0.01, "场馆墙: 裁到场地范围后配准正常");
  }

  // ── 3d. 方向退化：起点附近信息充足；"长走廊"（地面 + 两面平行于 y 的墙）沿 y 退化 ──
  {
    const auto base_true = compose(true_map_to_odom, odom_to_base);
    const auto init = compose(compose(base_true, make(0.02, -0.02, 0.0, 0, 0, 0)),
        inverse(odom_to_base));
    const auto r = matcher.match(scan, init);
    std::cout << "   起点附近: 平面约束强度（每点）弱 " << r.info_xy_min << " / 强 " << r.info_xy_max << std::endl;
    check(r.info_xy_min > 1.5, "退化: 起点附近两个方向都约束得住（每点 > 1.5）");

    Points corridor;
    for (float y = -6.0f; y <= 6.0f; y += 0.05f) {
      for (float x = -1.0f; x <= 1.0f; x += 0.05f) {corridor.emplace_back(x, y, 0.0f);}
      for (float z = 0.0f; z <= 1.0f; z += 0.05f) {
        corridor.emplace_back(-1.0f, y, z);
        corridor.emplace_back(1.0f, y, z);
      }
    }
    MapMatcher corridor_matcher(corridor);
    const auto rc = corridor_matcher.match(corridor, make(0.03, 0.03, 0, 0, 0, 0));
    std::cout << "   走廊: 弱 " << rc.info_xy_min << " / 强 " << rc.info_xy_max << "，弱方向 ("
              << rc.weak_dir_x << ", " << rc.weak_dir_y << ")" << std::endl;
    check(rc.info_xy_min < 1.5 && rc.info_xy_min < 0.01 * rc.info_xy_max,
      "退化: 走廊沿长度方向几乎没有约束（每点 < 1.5）");
    check(std::abs(rc.weak_dir_y) > 0.99, "退化: 最弱方向是走廊方向 y");
    check(std::abs(rc.map_to_odom.x) < 0.005, "退化: 横向（x）照样配准回来");

    // 只修强方向：车体 x、朝向、z 照修，y 不动
    const auto before = make(1, 2, 0, 0, 0, 0.3);
    const auto obase = make(3, -1, 0.2, 0, 0, -0.5);
    const auto b0 = compose(before, obase);
    // after：车体 x +4 cm、y −5 cm、z +1 cm、朝向 +0.01 rad
    const auto after = compose(
      make(b0.x + 0.04, b0.y - 0.05, b0.z + 0.01, 0, 0, yaw_of(b0) + 0.01), inverse(obase));
    const auto kept = drop_weak_direction(before, after, obase, 0.0, 1.0);
    const auto ck = correction_between(before, kept, obase);
    check(std::abs(ck.dx - 0.04) < 1e-9 && std::abs(ck.dy) < 1e-9, "只修强方向: x 修 4 cm，y 不修");
    check(std::abs(ck.dz - 0.01) < 1e-9 && std::abs(ck.dyaw - 0.01) < 1e-9, "只修强方向: z、朝向照修");

    // 死区里只修高度
    const auto zonly = keep_only_z(before, after, obase);
    const auto cz = correction_between(before, zonly, obase);
    check(cz.xy() < 1e-9 && std::abs(cz.dyaw) < 1e-9 && std::abs(cz.dz - 0.01) < 1e-9,
      "只修高度: x、y、朝向不动，z 修 1 cm");
  }

  // ── 4. 验收的单项分支 ──
  {
    MatchResult r;
    r.converged = true;
    r.num_points = 1000;
    r.inlier_ratio = 0.9;
    Correction c;
    check(decide(r, c, accept).accepted, "验收: 全部正常时接受");
    r.num_points = 10;
    check(!decide(r, c, accept).accepted, "验收: 点太少拒绝");
    r.num_points = 1000;
    r.inlier_ratio = 0.3;
    check(!decide(r, c, accept).accepted, "验收: 内点太少拒绝");
    r.inlier_ratio = 0.9;
    c.dyaw = 10 * M_PI / 180;
    check(!decide(r, c, accept).accepted, "验收: 朝向修正太大拒绝");
  }

  // ── 4b. "只有 z 错"自愈（2026-10-06：Point-LIO z 掉了 0.6 m，ICP 算对了却被 0.1 m 上限拒）──
  {
    MatchResult r;
    r.converged = true;
    r.num_points = 670;
    r.inlier_ratio = 0.88;                 // 助教那次的数
    Correction c;
    c.dx = 0.003;
    c.dy = -0.004;
    c.dz = 0.613;
    c.dyaw = -0.11 * M_PI / 180;
    const auto d = decide(r, c, accept);
    check(d.accepted && d.large_z, "z-only: 水平 / 朝向几乎不动、内点 0.88 → 放行大幅高度修正");
    check(!decide(r, Correction{0.0, 0.0, 0.05, 0.0, 0.0}, accept).large_z, "z-only: 小修正不算大幅");

    auto bad = c;
    bad.dx = 0.08;   // 水平也被拉了 8 cm：像"换了一层"的错误解
    check(!decide(r, bad, accept).accepted, "z-only: 水平被拉歪 → 仍拒绝");
    auto yawed = c;
    yawed.dyaw = 3 * M_PI / 180;
    check(!decide(r, yawed, accept).accepted, "z-only: 朝向被拉歪 → 仍拒绝");
    auto low = r;
    low.inlier_ratio = 0.7;                // 错误解的内点 0.5～0.7
    const auto dl = decide(low, c, accept);
    check(!dl.accepted && dl.reason.find("建议 Retry") != std::string::npos,
      "z-only: 内点只有 0.7 → 拒绝，并提示建议 Retry");
    auto huge = c;
    huge.dz = 1.5;
    check(!decide(r, huge, accept).accepted, "z-only: 超过 z_only_max_dz → 拒绝");
    auto off = accept;
    off.z_only_max_dz = 0.0;
    check(!decide(r, c, off).accepted, "z-only: z_only_max_dz = 0 关掉放行");
  }

  // ── 5. 修正量按车体算，不按 map->lidar_odom 本身 ──
  {
    // 车离 lidar_odom 原点 10 m，map->lidar_odom 绕原点转 1°：车体只转 1°、平移约 17 cm
    const auto before = make(0, 0, 0, 0, 0, 0);
    const auto after = make(0, 0, 0, 0, 0, M_PI / 180);
    const auto c = correction_between(before, after, make(10, 0, 0, 0, 0, 0));
    check(std::abs(c.dyaw - M_PI / 180) < 1e-9, "修正量: yaw 1°");
    check(std::abs(c.xy() - 10 * M_PI / 180) < 1e-3, "修正量: 杠杆臂带来的平移 ≈ 17 cm");
  }

  if (g_failures != 0) {
    std::cerr << g_failures << " 个检查失败" << std::endl;
    return 1;
  }
  std::cout << "全部通过" << std::endl;
  return 0;
}
