#include "rc2027_perception/crop_box.hpp"

#include <stdexcept>
#include <string>

namespace rc2027_perception
{

std::vector<AxisBox> boxes_from_flat(const std::vector<double> & flat)
{
  if (flat.size() % 6 != 0) {
    throw std::invalid_argument(
      "裁剪框参数要是 6 的倍数个数（x_min x_max y_min y_max z_min z_max），现在是 " +
      std::to_string(flat.size()) + " 个");
  }
  std::vector<AxisBox> out;
  for (size_t i = 0; i < flat.size(); i += 6) {
    const AxisBox b{flat[i], flat[i + 1], flat[i + 2], flat[i + 3], flat[i + 4], flat[i + 5]};
    if (b.x_min > b.x_max || b.y_min > b.y_max || b.z_min > b.z_max) {
      throw std::invalid_argument("第 " + std::to_string(i / 6 + 1) + " 个裁剪框 min > max");
    }
    out.push_back(b);
  }
  return out;
}

CropFilter::CropFilter(const std::vector<AxisBox> & boxes, const Transform34 & base_from_lidar)
: boxes_(boxes), t_(base_from_lidar)
{
}

bool CropFilter::keep(float x, float y, float z) const
{
  const double bx = t_[0] * x + t_[1] * y + t_[2] * z + t_[3];
  const double by = t_[4] * x + t_[5] * y + t_[6] * z + t_[7];
  const double bz = t_[8] * x + t_[9] * y + t_[10] * z + t_[11];
  for (const auto & b : boxes_) {
    if (b.contains(bx, by, bz)) {
      return false;
    }
  }
  return true;
}

}  // namespace rc2027_perception
