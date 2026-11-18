// field_geometry 的单测——不用 gtest（镜像里不确定有没有），用普通的
// assert + main()，CMakeLists 里用 add_test() 注册成 ctest 用例。
// 跟 Python 版 test/test_field_geometry.py 的测试用例一一对应，方便交叉
// 对照两边行为是否一致。

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "rc2027_navigation/field_geometry.hpp"

using rc2027_navigation::load_geometry;
using rc2027_navigation::yaw_to_quaternion;
using rc2027_navigation::zone_pose;

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

void test_start_zone_red_box_a(const YAML::Node & geometry)
{
  const auto pose = zone_pose(geometry, "red", "start", "box_a");
  check(std::abs(pose.x - (-5.15)) < 1e-9, "start_zone_red_box_a.x");
  check(std::abs(pose.y - (-5.15)) < 1e-9, "start_zone_red_box_a.y");
  check(std::abs(pose.yaw - M_PI / 2.0) < 1e-3, "start_zone_red_box_a.yaw");
}

void test_start_zone_mirrors_in_x(const YAML::Node & geometry)
{
  const auto red = zone_pose(geometry, "red", "start", "box_a");
  const auto blue = zone_pose(geometry, "blue", "start", "box_a");
  check(red.x == -blue.x, "start_zone_mirrors_in_x.x");
  check(red.y == blue.y, "start_zone_mirrors_in_x.y");
  check(red.yaw == blue.yaw, "start_zone_mirrors_in_x.yaw");
}

void test_l1_retry_zone_on_platform(const YAML::Node & geometry)
{
  // 2026-10-04：L1 Retry Zone 在 0.6 m 高的平台上，z 必须带出来（锁定 map 系要用）；
  // yaw 朝南 -90°（用户定；原来是 null，这个测试原来检查的是"没定就抛异常"）
  const auto red = zone_pose(geometry, "red", "l1");
  check(std::abs(red.x - (-1.8)) < 1e-9 && std::abs(red.y - 2.6) < 1e-9, "l1_retry_zone.xy");
  check(std::abs(red.z - 0.6) < 1e-9, "l1_retry_zone.z = 0.6（在 L1 平台上）");
  check(std::abs(red.yaw + M_PI / 2.0) < 1e-3, "l1_retry_zone.yaw 朝南 -90°");
  const auto blue = zone_pose(geometry, "blue", "l1");
  check(std::abs(blue.x - 1.8) < 1e-9 && blue.yaw == red.yaw, "l1_retry_zone 蓝方镜像：朝南不变");
  const auto start = zone_pose(geometry, "red", "start", "box_b");
  check(start.z == 0.0, "start_zone.z 没写就是 0（地面）");
}

void test_unknown_zone_rejected(const YAML::Node & geometry)
{
  bool threw = false;
  try {
    zone_pose(geometry, "red", "l2");
  } catch (const std::invalid_argument & e) {
    threw = std::string(e.what()).find("zone") != std::string::npos;
  }
  check(threw, "unknown_zone_rejected");
}

void test_unknown_side_rejected(const YAML::Node & geometry)
{
  bool threw = false;
  try {
    zone_pose(geometry, "green", "start");
  } catch (const std::invalid_argument & e) {
    threw = std::string(e.what()).find("side") != std::string::npos;
  }
  check(threw, "unknown_side_rejected");
}

void test_yaw_to_quaternion_zero()
{
  const auto q = yaw_to_quaternion(0.0);
  check(q[0] == 0.0 && q[1] == 0.0 && q[2] == 0.0 && q[3] == 1.0, "yaw_to_quaternion_zero");
}

void test_yaw_to_quaternion_90deg()
{
  const auto q = yaw_to_quaternion(M_PI / 2.0);
  check(std::abs(q[2] - std::sin(M_PI / 4.0)) < 1e-9, "yaw_to_quaternion_90deg.z");
  check(std::abs(q[3] - std::cos(M_PI / 4.0)) < 1e-9, "yaw_to_quaternion_90deg.w");
  check(q[0] == 0.0 && q[1] == 0.0, "yaw_to_quaternion_90deg.xy");
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "用法: " << argv[0] << " <field_geometry.yaml 路径>" << std::endl;
    return 2;
  }

  const YAML::Node geometry = load_geometry(argv[1]);

  test_start_zone_red_box_a(geometry);
  test_start_zone_mirrors_in_x(geometry);
  test_l1_retry_zone_on_platform(geometry);
  test_unknown_zone_rejected(geometry);
  test_unknown_side_rejected(geometry);
  test_yaw_to_quaternion_zero();
  test_yaw_to_quaternion_90deg();

  if (g_failures > 0) {
    std::cerr << g_failures << " 个测试失败" << std::endl;
    return 1;
  }
  std::cout << "全部测试通过" << std::endl;
  return 0;
}
