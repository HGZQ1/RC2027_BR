// pose_math 的单测——跟 test_field_geometry_main.cpp 一样不用 gtest，
// assert 风格 + main()，ctest 注册。不读任何文件，host 上直接 g++ 就能跑。
//
// 最重要的是 end_to_end_*：模拟"雷达装歪了、Point-LIO 的 lidar_odom 原点
// 是开机时 IMU 所在位置且重力对齐"的真实情况，车开走以后，用两个节点的
// 换算公式算出的 map -> base_footprint 必须等于车在场地上的真值。

#include <cmath>
#include <iostream>
#include <string>

#include "rc2027_navigation/pose_math.hpp"

using rc2027_navigation::compose;
using rc2027_navigation::inverse;
using rc2027_navigation::map_to_odom;
using rc2027_navigation::odom_to_base;
using rc2027_navigation::Pose2D;
using rc2027_navigation::Transform3D;
using rc2027_navigation::yaw_of;

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
  // 跟 tf2 Quaternion::setRPY 同一个公式（固定轴 X-Y-Z）
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

bool near(const Transform3D & a, const Transform3D & b, double tol = 1e-9)
{
  // q 和 -q 是同一个旋转
  const double dot = a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw;
  return std::abs(a.x - b.x) < tol && std::abs(a.y - b.y) < tol && std::abs(a.z - b.z) < tol &&
         std::abs(std::abs(dot) - 1.0) < tol;
}

void test_inverse_composes_to_identity()
{
  const auto t = make(1.0, -2.0, 0.3, 0.1, -0.4, 2.5);
  check(near(compose(t, inverse(t)), Transform3D{}), "t * t^-1 == I");
  check(near(compose(inverse(t), t), Transform3D{}), "t^-1 * t == I");
}

void test_compose_matches_hand_computed()
{
  // 父系里在 (1,0,0) 处、转了 90° 的子系；子系里 (1,0,0) 的点在父系里是 (1,1,0)
  const auto a = make(1.0, 0.0, 0.0, 0.0, 0.0, M_PI / 2);
  const auto b = make(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  check(near(compose(a, b), make(1.0, 1.0, 0.0, 0.0, 0.0, M_PI / 2)), "compose hand case");
}

void test_map_to_odom_when_lidar_odom_is_base_itself()
{
  // 雷达外参为 0、Point-LIO 原点就是车体：map->lidar_odom 就是配置位姿本身
  // （这是 10-03 之前 field_localizer 的隐含假设）
  const Pose2D start{-4.35, -5.15, M_PI / 2};
  const auto m = map_to_odom(start, Transform3D{});
  check(near(m, make(-4.35, -5.15, 0.0, 0.0, 0.0, M_PI / 2)), "zero-extrinsic degenerates to old");
  check(std::abs(yaw_of(m) - M_PI / 2) < 1e-9, "yaw_of");
}

void run_end_to_end(const Transform3D & base_to_body, const std::string & name)
{
  // 车在场地上的起点（Start Zone 配置值）和开走以后的真值
  const Pose2D start{-4.35, -5.15, M_PI / 2};
  const Transform3D field_to_base_t0 = make(-4.35, -5.15, 0.0, 0.0, 0.0, M_PI / 2);
  const Transform3D field_to_base_t1 = make(1.2, 3.4, 0.0, 0.0, 0.0, -2.0);

  // Point-LIO 的 lidar_odom：原点在开机时 IMU 的位置，只把重力对齐（roll/pitch
  // 去掉），yaw 是任意的——代码完全不知道它，这里随便取一个跟实际一致的
  const Transform3D field_to_body_t0 = compose(field_to_base_t0, base_to_body);
  const Transform3D field_to_odom = make(
    field_to_body_t0.x, field_to_body_t0.y, field_to_body_t0.z, 0.0, 0.0, 0.7);

  // Point-LIO 输出的 lidar_odom -> body（两个时刻）
  const auto odom_to_body_t0 = compose(inverse(field_to_odom), field_to_body_t0);
  const auto odom_to_body_t1 =
    compose(inverse(field_to_odom), compose(field_to_base_t1, base_to_body));

  // lio_to_base 节点做的事
  const auto odom_to_base_t0 = odom_to_base(odom_to_body_t0, base_to_body);
  const auto odom_to_base_t1 = odom_to_base(odom_to_body_t1, base_to_body);

  // field_localizer 在 t0 锁定 map -> lidar_odom
  const auto m = map_to_odom(start, odom_to_base_t0);

  // 别的节点查 TF 得到的 map -> base_footprint
  check(near(compose(m, odom_to_base_t0), field_to_base_t0), name + ": t0 回到起点");
  check(near(compose(m, odom_to_base_t1), field_to_base_t1), name + ": t1 等于真值");
}

}  // namespace

void test_latch_on_elevated_zone()
{
  // 车在 L1 平台（z = 0.6）上 Retry：Point-LIO 重置后 lidar_odom 原点就在车上方，
  // 锁定后 map -> base_footprint 的 z 必须是 0.6，不能是 0（2026-10-04 修的）
  const Pose2D l1{-1.8, 2.6, M_PI / 2, 0.6};
  const auto odom_to_base = make(0.0, 0.0, -0.23, 0.0, 0.0, 0.3);  // IMU 在车体上方 0.23 m
  const auto m = map_to_odom(l1, odom_to_base);
  const auto base = compose(m, odom_to_base);
  check(near(base, make(-1.8, 2.6, 0.6, 0.0, 0.0, M_PI / 2)), "L1 平台上锁定: 车体 z = 0.6");
}

int main()
{
  test_latch_on_elevated_zone();
  test_inverse_composes_to_identity();
  test_compose_matches_hand_computed();
  test_map_to_odom_when_lidar_odom_is_base_itself();
  // 仿真里的情况：雷达+IMU 在车体正上方，平装
  run_end_to_end(make(0.0, 0.0, 0.255, 0.0, 0.0, 0.0), "flat_on_top");
  // 偏心 + 转了个角（10-04 助教建议的仿真验证：故意给雷达一个前向偏移）
  run_end_to_end(make(0.2, 0.05, 0.255, 0.0, 0.0, 0.3), "offset_and_yawed");
  // 真车大概率的情况：30° 俯仰安装
  run_end_to_end(make(0.1, 0.0, 0.3, 0.0, 0.5236, 0.0), "pitched_30deg");

  if (g_failures != 0) {
    std::cerr << g_failures << " 个检查失败" << std::endl;
    return 1;
  }
  std::cout << "全部通过" << std::endl;
  return 0;
}
