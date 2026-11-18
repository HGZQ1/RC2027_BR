// "车体 + 货仓"裁剪框——纯 C++，不依赖 ROS，host 上单测。
//
// 为什么要（0002 2026-10-05 助教 + 用户）：拿上方块以后存在车后面（机械说比底盘高 15 cm 放），一摞 700 mm 高，
// 顶部会进前倾雷达往后上方的视场（+23°～82°），而且在 Point-LIO 的 blind（0.5 m 球）外面。这些点**跟着车走**，
// 会把 Point-LIO 往"车没动"拉，方块检测也会把它当成方块。Point-LIO 只有 blind 一种过滤，所以在它前面加一道：
// 原始点云 → 去掉 base_link 系里几个盒子里的点 → 再给 Point-LIO 和方块检测。
#pragma once

#include <array>
#include <vector>

namespace rc2027_perception
{

/// base_link 系里的轴对齐盒子
struct AxisBox
{
  double x_min, x_max, y_min, y_max, z_min, z_max;
  bool contains(double x, double y, double z) const
  {
    return x >= x_min && x <= x_max && y >= y_min && y <= y_max && z >= z_min && z <= z_max;
  }
};

/// 参数里的扁平数组 [x_min, x_max, y_min, y_max, z_min, z_max, ...] → 盒子。长度不是 6 的倍数、
/// 或者 min > max 时抛 std::invalid_argument
std::vector<AxisBox> boxes_from_flat(const std::vector<double> & flat);

/// 雷达系 → base_link 的刚体变换（行优先 3x4：R | t）
using Transform34 = std::array<double, 12>;

class CropFilter
{
public:
  CropFilter(const std::vector<AxisBox> & boxes, const Transform34 & base_from_lidar);

  /// 雷达系里的这个点要不要留（不在任何一个盒子里就留）
  bool keep(float x, float y, float z) const;

private:
  std::vector<AxisBox> boxes_;
  Transform34 t_;
};

}  // namespace rc2027_perception
