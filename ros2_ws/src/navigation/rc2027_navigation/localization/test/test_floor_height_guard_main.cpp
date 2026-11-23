#include <cassert>
#include <cmath>
#include <initializer_list>
#include "rc2027_navigation/floor_height_guard.hpp"

int main()
{
  using rc2027_navigation::floor_height_correction;
  // 复现：下楼规则高度 0.60 + 仿真偏移 0.05，估计掉至 0.49。
  bool hold = false;
  assert(std::abs(floor_height_correction(0.49, 0.65, hold, false) - 0.16) < 1e-9);
  assert(hold);
  // 换位第一拍已开启保护，小于旧阈值的连续漂移也逐拍抵消。
  hold = true;
  for (double z : {0.64, 0.61, 0.49, 0.10}) {
    assert(std::abs(z + floor_height_correction(z, 0.65, hold, false) - 0.65) < 1e-9);
  }
  // 真正换层必须释放旧台面约束。
  assert(floor_height_correction(0.95, 0.65, hold, true) == 0.0);
  // 无换位、正常小幅变化不自动进入保持状态。
  hold = false;
  assert(floor_height_correction(0.62, 0.65, hold, false) == 0.0);
  assert(!hold);
}
