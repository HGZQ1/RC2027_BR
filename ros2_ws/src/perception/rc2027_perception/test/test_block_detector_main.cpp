// block_detector 的单测——assert 风格 + main()，ctest 注册，host 上 g++ 就能跑。
//
// 合成点云：方块的侧面 / 顶面里朝着雷达的那几面 + 台面，加 5 mm 噪声。不做遮挡（方块之间不互相挡），
// 所以只能证明几何算法本身对；真实遮挡、雷达扫描图案要在 Gazebo 里摆方块测（sim_blocks）。

#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "rc2027_perception/block_detector.hpp"

using rc2027_perception::Block;
using rc2027_perception::BlockType;
using rc2027_perception::decode_stack;
using rc2027_perception::DetectorParams;
using rc2027_perception::detect_blocks;
using rc2027_perception::Point3;
using rc2027_perception::Region;
using rc2027_perception::to_string;
using rc2027_perception::Viewpoint;

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

constexpr double kPi = 3.14159265358979323846;
constexpr double kSurface = 0.65;   // 仿真里 L1 台面

struct Box
{
  double x, y, z_bottom, size_xy, size_z, yaw;
};

struct Scene
{
  std::vector<Box> boxes;
  std::mt19937 rng{42};

  void block(double x, double y, int layer_bottom_ground, bool sky, double yaw_deg = 0.0)
  {
    const double s = sky ? 0.2 : 0.35;
    boxes.push_back({x, y, kSurface + 0.35 * layer_bottom_ground, s, s, yaw_deg * kPi / 180});
  }

  // 朝着雷达的面 + 台面（区域里不被方块压住的地方）
  std::vector<Point3> render(const Viewpoint & v, double cx, double cy, double half, double step = 0.02)
  {
    std::normal_distribution<double> noise(0.0, 0.005);
    std::vector<Point3> out;
    auto add = [&](double x, double y, double z) {
        out.push_back({static_cast<float>(x + noise(rng)), static_cast<float>(y + noise(rng)),
            static_cast<float>(z + noise(rng))});
      };
    for (double x = cx - half; x <= cx + half; x += step) {
      for (double y = cy - half; y <= cy + half; y += step) {
        bool covered = false;
        for (const auto & b : boxes) {
          const double c = std::cos(-b.yaw), s = std::sin(-b.yaw);
          const double u = c * (x - b.x) - s * (y - b.y), w = s * (x - b.x) + c * (y - b.y);
          covered |= (std::abs(u) < b.size_xy / 2 && std::abs(w) < b.size_xy / 2 &&
            std::abs(b.z_bottom - kSurface) < 0.01);
        }
        if (!covered) {add(x, y, kSurface);}
      }
    }
    for (const auto & b : boxes) {
      const double c = std::cos(b.yaw), s = std::sin(b.yaw);
      const double h = b.size_xy / 2;
      // 4 个侧面：法向 (±1,0)、(0,±1)（方块系）
      const double normals[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
      for (const auto & n : normals) {
        const double nx = c * n[0] - s * n[1], ny = s * n[0] + c * n[1];
        const double fx = b.x + nx * h, fy = b.y + ny * h;
        if (nx * (v.x - fx) + ny * (v.y - fy) <= 0) {
          continue;  // 背对雷达
        }
        const double tx = -ny, ty = nx;  // 面内水平方向
        for (double t = -h; t <= h; t += step) {
          for (double z = b.z_bottom; z <= b.z_bottom + b.size_z; z += step) {
            const double px = fx + tx * t, py = fy + ty * t;
            const double elev = std::atan2(z - v.z, std::hypot(px - v.x, py - v.y));
            if (elev <= v.max_elevation) {add(px, py, z);}
          }
        }
      }
      // 顶面：雷达比它高才看得到
      if (v.z > b.z_bottom + b.size_z) {
        for (double a = -h; a <= h; a += step) {
          for (double d = -h; d <= h; d += step) {
            add(b.x + c * a - s * d, b.y + s * a + c * d, b.z_bottom + b.size_z);
          }
        }
      }
    }
    return out;
  }
};

Region spot(double cx, double cy)
{
  Region r;
  r.name = "spot";
  r.cx = cx;
  r.cy = cy;
  r.half_x = r.half_y = 0.4;
  r.expected_surface_z = kSurface;
  return r;
}

Viewpoint from(double x, double y, double z = kSurface + 0.23)
{
  Viewpoint v;
  v.x = x;
  v.y = y;
  v.z = z;
  v.max_elevation = 22.0 * kPi / 180;
  return v;
}

double yaw_err_deg(double a, double b)
{
  double d = std::fmod(a - b, kPi / 2);
  if (d > kPi / 4) {d -= kPi / 2;}
  if (d < -kPi / 4) {d += kPi / 2;}
  return std::abs(d) * 180 / kPi;
}

void report(const Block & b)
{
  std::cout << "   " << to_string(b.type) << " 第" << b.layer << "层 (" << b.x << ", " << b.y
            << ") yaw " << b.yaw * 180 / kPi << "° 看到 " << b.extent_a << " x " << b.extent_b
            << "，" << b.num_points << " 点" << (b.center_reliable ? "" : "（中心不可靠）") << std::endl;
}

void test_decode()
{
  DetectorParams p;
  int g = -1;
  bool s = false;
  check(decode_stack(0.35, p, &g, &s) && g == 1 && !s, "组合: 0.35 = 1 地");
  check(decode_stack(0.71, p, &g, &s) && g == 2 && !s, "组合: 0.71 = 2 地");
  check(decode_stack(0.88, p, &g, &s) && g == 2 && s, "组合: 0.88 = 2 地 + 天");
  check(decode_stack(0.56, p, &g, &s) && g == 1 && s, "组合: 0.56 = 1 地 + 天");
  check(decode_stack(0.19, p, &g, &s) && g == 0 && s, "组合: 0.19 = 天空块");
  check(!decode_stack(0.45, p, &g, &s), "组合: 0.45 对不上");
}

void test_empty_spot()
{
  Scene sc;
  const auto r = detect_blocks(sc.render(from(-2.7, 1.5), -2.7, 2.7, 0.45), spot(-2.7, 2.7), from(-2.7, 1.5));
  check(r.visible && r.blocks.empty() && r.ground_layers == 0, "空建造点: 看得到、0 层");
  check(r.surface_measured && std::abs(r.surface_z - kSurface) < 0.005, "空建造点: 台面高度测得准");
}

void test_single_ground_block()
{
  Scene sc;
  sc.block(-2.67, 2.68, 0, false, 7.0);
  const auto v = from(-3.6, 1.6);   // 斜着看，两个面都看得到
  const auto r = detect_blocks(sc.render(v, -2.7, 2.7, 0.45), spot(-2.7, 2.7), v);
  check(r.blocks.size() == 1 && r.ground_layers == 1 && !r.has_sky, "一块地块: 1 层");
  if (!r.blocks.empty()) {
    const auto & b = r.blocks[0];
    report(b);
    check(b.type == BlockType::Ground, "一块地块: 类型 GROUND");
    check(std::hypot(b.x + 2.67, b.y - 2.68) < 0.015, "一块地块: 中心误差 < 1.5 cm");
    check(yaw_err_deg(b.yaw, 7.0 * kPi / 180) < 1.5, "一块地块: 朝向误差 < 1.5°");
    check(b.center_reliable, "一块地块: 两面都看到，中心可靠");
  }
}

void test_one_face_only()
{
  // 正对一个面（朝向 0°，雷达在正南）：只看得到南面，中心靠"近边往里推半个边长"
  Scene sc;
  sc.block(-2.7, 2.72, 0, false, 0.0);
  const auto v = from(-2.7, 1.3);
  const auto r = detect_blocks(sc.render(v, -2.7, 2.7, 0.45), spot(-2.7, 2.7), v);
  check(r.blocks.size() == 1, "只看到一面: 找到 1 块");
  if (!r.blocks.empty()) {
    report(r.blocks[0]);
    check(std::hypot(r.blocks[0].x + 2.7, r.blocks[0].y - 2.72) < 0.015, "只看到一面: 中心误差 < 1.5 cm");
    check(r.blocks[0].center_reliable, "只看到一面: 面看全了，中心可靠");
  }
}

void test_full_tower()
{
  // 两层地块 + 天空块，每层有点错位；从 2.2 m 外看（近了顶部出视场）
  Scene sc;
  sc.block(-2.70, 2.70, 0, false, 3.0);
  sc.block(-2.68, 2.71, 1, false, -4.0);
  sc.block(-2.69, 2.69, 2, true, 10.0);
  const auto v = from(-4.2, 1.1);
  const auto r = detect_blocks(sc.render(v, -2.7, 2.7, 0.45), spot(-2.7, 2.7), v);
  for (const auto & b : r.blocks) {report(b);}
  check(r.ground_layers == 2 && r.has_sky, "完整塔: 2 层地块 + 天空块");
  check(!r.top_maybe_clipped, "完整塔: 2.2 m 外顶部在视场里");
  if (r.blocks.size() == 3) {
    check(std::hypot(r.blocks[1].x + 2.68, r.blocks[1].y - 2.71) < 0.015, "完整塔: 第 2 层中心误差 < 1.5 cm");
    check(r.blocks[2].type == BlockType::Sky && yaw_err_deg(r.blocks[2].yaw, 10 * kPi / 180) < 2.0,
      "完整塔: 天空块朝向误差 < 2°");
    check(std::hypot(r.blocks[2].x + 2.69, r.blocks[2].y - 2.69) < 0.02, "完整塔: 天空块中心误差 < 2 cm");
  } else {
    check(false, "完整塔: 应该是 3 块，实际 " + std::to_string(r.blocks.size()));
  }
}

void test_centered_tower_seen_from_one_side()
{
  // 2026-10-05 仿真踩的坑：天空块居中放、从正侧面看，它的面比地块的面往里缩 7.5 cm，投影隔开，
  // 原来被拆成"2 层地块"+"悬空的天空块（还补出两层不存在的地块）"
  Scene sc;
  sc.block(-2.7, 2.7, 0, false);
  sc.block(-2.7, 2.7, 1, false);
  sc.block(-2.7, 2.7, 2, true);
  const auto v = from(-0.6, 2.7);   // 正东 2.1 m，只看得到东面
  const auto r = detect_blocks(sc.render(v, -2.7, 2.7, 0.45), spot(-2.7, 2.7), v);
  for (const auto & b : r.blocks) {report(b);}
  check(r.ground_layers == 2 && r.has_sky && r.blocks.size() == 3,
    "居中的塔从正侧面看: 一摞 2 地 + 天，3 块（实际 " + std::to_string(r.blocks.size()) + " 块）");
  if (r.blocks.size() == 3) {
    check(std::hypot(r.blocks[2].x + 2.7, r.blocks[2].y - 2.7) < 0.02, "居中的塔: 天空块中心误差 < 2 cm");
  }
}

void test_floating_block_not_merged()
{
  // 一块地块旁边 5 cm 处、离台面 0.5 m 悬空着一块（别的车夹爪举着）：水平上离得近，但底（0.5）跟地块顶（0.35）
  // 对不上，不能并进去把层数算成 2；单独标成悬空的 UNKNOWN（助教 10-05 提的边界情况）
  Scene sc;
  sc.block(-2.7, 2.7, 0, false);
  sc.boxes.push_back({-2.30, 2.7, kSurface + 0.50, 0.35, 0.35, 0.0});
  const auto v = from(-2.5, 0.9);
  const auto r = detect_blocks(sc.render(v, -2.6, 2.7, 0.5), spot(-2.7, 2.7), v);
  for (const auto & b : r.blocks) {report(b);}
  check(r.ground_layers == 1 && !r.has_sky, "悬空的方块: 层数还是 1（没被并进去）");
  bool floating_unknown = false;
  for (const auto & b : r.blocks) {
    floating_unknown |= b.type == BlockType::Unknown && b.z_bottom > kSurface + 0.4;
  }
  check(floating_unknown && r.note.find("悬空") != std::string::npos,
    "悬空的方块: 单独标 UNKNOWN、说明悬空（" + r.note + "）");
}

void test_held_block_beside_overlapping_height()
{
  // 助教 10-05 仿真情况 A：地块旁边 5 cm、底离台面 0.30 m 举着一块（比那块地块的顶还低 5 cm，高度上重叠）。
  // 高度空档拆不开、3 cm 栅格又把两块连成一簇；原来被当成第 2 层（或者"高 0.63 对不上"）
  Scene sc;
  sc.block(-2.7, 2.7, 0, false);
  sc.boxes.push_back({-2.7, 3.10, kSurface + 0.30, 0.35, 0.35, 0.0});
  const auto v = from(-4.3, 2.6);   // 正西 1.6 m 看
  const auto r = detect_blocks(sc.render(v, -2.7, 2.9, 0.6), spot(-2.7, 2.7), v);
  for (const auto & b : r.blocks) {report(b);}
  check(r.ground_layers == 1 && !r.has_sky, "旁边举着、高度重叠: 层数还是 1（实际 " +
    std::to_string(r.ground_layers) + "）");
  check(!r.blocks.empty() && std::hypot(r.blocks[0].x + 2.7, r.blocks[0].y - 2.7) < 0.02,
    "旁边举着、高度重叠: 地块位置没被带偏");
}

void test_two_blocks_5cm_apart_separated()
{
  // 细栅格再分一次以后，台面上隔 5 cm 的两块也分得开了（原来要 ≥ 6 cm）
  Scene sc;
  sc.block(-2.7, 2.7, 0, false);
  sc.block(-2.7, 3.10, 0, false);
  const auto v = from(-4.3, 2.6);
  Region r0 = spot(-2.7, 2.9);
  r0.half_y = 0.5;
  const auto r = detect_blocks(sc.render(v, -2.7, 2.9, 0.6), r0, v);
  int ground = 0;
  for (const auto & b : r.blocks) {ground += b.type == BlockType::Ground;}
  check(ground == 2 && r.blocks.size() == 2, "隔 5 cm 的两块: 分成 2 块地块（实际 " +
    std::to_string(r.blocks.size()) + " 块）");
}

void test_held_sky_above_reported_once()
{
  // 助教情况 B：地块正上方空 15 cm 举着一块天空块——层数对，悬空的只报一个（细栅格再分曾经把它拆成两个）
  Scene sc;
  sc.block(-2.7, 2.7, 0, false);
  sc.boxes.push_back({-2.7, 2.7, kSurface + 0.50, 0.2, 0.2, 0.0});
  const auto v = from(-4.3, 2.6, kSurface + 0.6);   // 雷达比天空块底高，看得到它的顶面
  const auto r = detect_blocks(sc.render(v, -2.7, 2.7, 0.45), spot(-2.7, 2.7), v);
  int floating = 0;
  for (const auto & b : r.blocks) {floating += b.type == BlockType::Unknown;}
  check(r.ground_layers == 1 && !r.has_sky && floating == 1,
    "正上方举着天空块: 1 层、悬空的只报 1 个（实际 " + std::to_string(floating) + " 个）");
}

void test_tower_too_close_is_clipped()
{
  Scene sc;
  sc.block(-2.7, 2.7, 0, false);
  sc.block(-2.7, 2.7, 1, false);
  sc.block(-2.7, 2.7, 2, true);
  const auto v = from(-2.7, 1.9);   // 0.8 m，往上只看得到 22°
  const auto r = detect_blocks(sc.render(v, -2.7, 2.7, 0.45), spot(-2.7, 2.7), v);
  std::cout << "   近处看: " << r.ground_layers << " 层，最高 " << r.top_height << " m，" << r.note << std::endl;
  check(r.top_maybe_clipped, "太近看塔: 标出顶部可能出了视场");
  check(r.ground_layers >= 1, "太近看塔: 层数报\"至少 1 层\"，不报 0（报 0 会按空的去放）");
}

void test_transfer_area_stack_and_single()
{
  // 交接区"2 块叠 + 1 块单放"，中间留约 10 cm 缝（转了角以后角上最近处 ~8 cm）；另外一块天空块单放
  Region t;
  t.name = "transfer";
  t.cx = -3.5;
  t.cy = -1.3;
  t.half_x = t.half_y = 0.48;
  t.expected_surface_z = kSurface;
  Scene sc;
  sc.block(-3.70, -1.10, 0, false, 5.0);
  sc.block(-3.70, -1.10, 1, false, 5.0);
  sc.block(-3.20, -1.12, 0, false, -10.0);
  sc.block(-3.55, -1.55, 0, true, 20.0);
  const auto v = from(-2.4, -2.5);
  const auto r = detect_blocks(sc.render(v, -3.5, -1.3, 0.5), t, v);
  for (const auto & b : r.blocks) {report(b);}
  int ground = 0, sky = 0, stacked = 0;
  for (const auto & b : r.blocks) {
    ground += b.type == BlockType::Ground;
    sky += b.type == BlockType::Sky;
    stacked += b.layer == 2;
  }
  check(ground == 3 && sky == 1 && stacked == 1, "交接区: 3 地（其中 1 块在第 2 层）+ 1 天");
}

void test_touching_blocks_marked_unknown()
{
  Scene sc;
  sc.block(-2.7, 2.7, 0, false);
  sc.block(-2.35, 2.7, 0, false);   // 紧挨着
  const auto v = from(-3.5, 1.5);
  Region r0 = spot(-2.55, 2.7);
  r0.half_x = 0.5;
  const auto r = detect_blocks(sc.render(v, -2.55, 2.7, 0.55), r0, v);
  bool unknown = false;
  for (const auto & b : r.blocks) {unknown |= b.type == BlockType::Unknown;}
  check(unknown && !r.note.empty(), "两块挨着: 标 UNKNOWN 并说明（" + r.note + "）");
}

void test_robot_is_not_a_block()
{
  Scene sc;
  sc.boxes.push_back({-2.7, 2.7, kSurface, 0.3, 0.45, 0.0});   // 0.45 高，对不上任何组合
  const auto v = from(-3.5, 1.5);
  const auto r = detect_blocks(sc.render(v, -2.7, 2.7, 0.45), spot(-2.7, 2.7), v);
  check(r.blocks.size() == 1 && r.blocks[0].type == BlockType::Unknown && r.ground_layers == 0,
    "高 0.45 的东西: UNKNOWN，不算层数");
}

void test_low_fence_ignored()
{
  Scene sc;
  sc.boxes.push_back({-2.98, 2.7, kSurface, 0.05, 0.10, 0.0});   // L1 边上 10 cm 高的围栏
  const auto v = from(-2.4, 1.6);
  const auto r = detect_blocks(sc.render(v, -2.7, 2.7, 0.45), spot(-2.7, 2.7), v);
  check(r.blocks.empty(), "10 cm 高的围栏: 不当方块");
}

void test_surface_not_visible_uses_expected()
{
  // 从 L1 看 L2 建造点：台面比雷达高，只看得到方块侧面
  Scene sc;
  sc.block(-1.25, 1.25, 0, false);
  auto pts = sc.render(from(-2.4, 0.2), -1.25, 1.25, 0.45);
  std::vector<Point3> sides;
  for (const auto & p : pts) {
    if (p.z > kSurface + 0.02) {sides.push_back(p);}
  }
  const auto r = detect_blocks(sides, spot(-1.25, 1.25), from(-2.4, 0.2));
  check(!r.surface_measured && r.ground_layers == 1, "看不到台面: 用预期高度，照样数出 1 层");
}

void test_nothing_seen()
{
  const auto r = detect_blocks({}, spot(-2.7, 2.7), from(-2.7, 1.5));
  check(!r.visible && !r.note.empty(), "没有点: 报看不到");
}

}  // namespace

int main()
{
  test_decode();
  test_empty_spot();
  test_single_ground_block();
  test_one_face_only();
  test_full_tower();
  test_centered_tower_seen_from_one_side();
  test_floating_block_not_merged();
  test_held_block_beside_overlapping_height();
  test_two_blocks_5cm_apart_separated();
  test_held_sky_above_reported_once();
  test_tower_too_close_is_clipped();
  test_transfer_area_stack_and_single();
  test_touching_blocks_marked_unknown();
  test_robot_is_not_a_block();
  test_low_fence_ignored();
  test_surface_not_visible_uses_expected();
  test_nothing_seen();

  if (g_failures != 0) {
    std::cerr << g_failures << " 个检查失败" << std::endl;
    return 1;
  }
  std::cout << "全部通过" << std::endl;
  return 0;
}
