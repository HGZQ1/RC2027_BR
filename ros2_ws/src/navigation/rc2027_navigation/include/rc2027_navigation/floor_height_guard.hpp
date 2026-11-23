#pragma once

#include <cmath>

namespace rc2027_navigation
{
constexpr double kFloorDriftLimit = 0.08;

// 只约束已知的台面；真实换层期间不把车辆拉回旧楼层。
inline double floor_height_correction(double z, double floor, bool & hold, bool transitioning)
{
  if (transitioning || !std::isfinite(z) || !std::isfinite(floor)) {return 0.0;}
  if (!hold && std::abs(z - floor) <= kFloorDriftLimit) {return 0.0;}
  hold = true;
  return floor - z;
}
}  // namespace rc2027_navigation
