#include "rc2027_navigation/field_geometry.hpp"

#include <cmath>

namespace rc2027_navigation
{

YAML::Node load_geometry(const std::string & path)
{
  return YAML::LoadFile(path);
}

Pose2D zone_pose(
  const YAML::Node & geometry, const std::string & side, const std::string & zone,
  const std::string & start_box)
{
  if (side != "red" && side != "blue") {
    throw std::invalid_argument("side 必须是 'red' 或 'blue'，收到 '" + side + "'");
  }

  YAML::Node cfg;
  if (zone == "start") {
    cfg = geometry["start_zone"][side][start_box];
    if (!cfg || cfg.IsNull()) {
      throw std::invalid_argument(
        "field_geometry.yaml 里找不到 start_zone." + side + "." + start_box);
    }
  } else if (zone == "l1") {
    cfg = geometry["l1_retry_zone"][side];
  } else {
    throw std::invalid_argument("zone 必须是 'start' 或 'l1'，收到 '" + zone + "'");
  }

  const YAML::Node yaw_node = cfg["yaw"];
  if (!yaw_node || yaw_node.IsNull()) {
    throw std::invalid_argument(
      "side='" + side + "' zone='" + zone +
      "' 的 yaw 在 field_geometry.yaml 里还是 null（没定），不能用来初始化定位");
  }

  const YAML::Node center = cfg["center"];
  Pose2D pose;
  pose.x = center[0].as<double>();
  pose.y = center[1].as<double>();
  pose.yaw = yaw_node.as<double>();
  const YAML::Node z_node = cfg["z"];
  pose.z = (z_node && !z_node.IsNull()) ? z_node.as<double>() : 0.0;
  return pose;
}

std::array<double, 4> yaw_to_quaternion(double yaw)
{
  const double half = yaw / 2.0;
  return {0.0, 0.0, std::sin(half), std::cos(half)};
}

}  // namespace rc2027_navigation
