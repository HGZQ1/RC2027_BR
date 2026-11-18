#include "rc2027_navigation/pose_math.hpp"

#include <cmath>

namespace rc2027_navigation
{

namespace
{

struct Vec3
{
  double x;
  double y;
  double z;
};

// 用四元数 q 旋转向量 v：v' = v + 2w(u×v) + 2u×(u×v)，u = (qx, qy, qz)
Vec3 rotate(const Transform3D & q, const Vec3 & v)
{
  const double tx = 2.0 * (q.qy * v.z - q.qz * v.y);
  const double ty = 2.0 * (q.qz * v.x - q.qx * v.z);
  const double tz = 2.0 * (q.qx * v.y - q.qy * v.x);
  return {
    v.x + q.qw * tx + (q.qy * tz - q.qz * ty),
    v.y + q.qw * ty + (q.qz * tx - q.qx * tz),
    v.z + q.qw * tz + (q.qx * ty - q.qy * tx),
  };
}

}  // namespace

Transform3D compose(const Transform3D & a, const Transform3D & b)
{
  Transform3D out;
  const Vec3 t = rotate(a, {b.x, b.y, b.z});
  out.x = a.x + t.x;
  out.y = a.y + t.y;
  out.z = a.z + t.z;
  // 四元数乘法 a.q * b.q
  out.qw = a.qw * b.qw - a.qx * b.qx - a.qy * b.qy - a.qz * b.qz;
  out.qx = a.qw * b.qx + a.qx * b.qw + a.qy * b.qz - a.qz * b.qy;
  out.qy = a.qw * b.qy - a.qx * b.qz + a.qy * b.qw + a.qz * b.qx;
  out.qz = a.qw * b.qz + a.qx * b.qy - a.qy * b.qx + a.qz * b.qw;
  // 连乘多次会有浮点漂移，顺手归一化
  const double n = std::sqrt(out.qw * out.qw + out.qx * out.qx + out.qy * out.qy + out.qz * out.qz);
  out.qw /= n;
  out.qx /= n;
  out.qy /= n;
  out.qz /= n;
  return out;
}

Transform3D inverse(const Transform3D & t)
{
  Transform3D out;
  out.qw = t.qw;
  out.qx = -t.qx;
  out.qy = -t.qy;
  out.qz = -t.qz;
  const Vec3 p = rotate(out, {-t.x, -t.y, -t.z});
  out.x = p.x;
  out.y = p.y;
  out.z = p.z;
  return out;
}

Transform3D from_pose2d(const Pose2D & pose)
{
  Transform3D out;
  out.x = pose.x;
  out.y = pose.y;
  out.z = pose.z;
  const auto q = yaw_to_quaternion(pose.yaw);
  out.qx = q[0];
  out.qy = q[1];
  out.qz = q[2];
  out.qw = q[3];
  return out;
}

double yaw_of(const Transform3D & t)
{
  return std::atan2(
    2.0 * (t.qw * t.qz + t.qx * t.qy), 1.0 - 2.0 * (t.qy * t.qy + t.qz * t.qz));
}

Transform3D odom_to_base(const Transform3D & odom_to_body, const Transform3D & base_to_body)
{
  return compose(odom_to_body, inverse(base_to_body));
}

Transform3D map_to_odom(const Pose2D & base_in_map, const Transform3D & odom_to_base_now)
{
  return compose(from_pose2d(base_in_map), inverse(odom_to_base_now));
}

}  // namespace rc2027_navigation
