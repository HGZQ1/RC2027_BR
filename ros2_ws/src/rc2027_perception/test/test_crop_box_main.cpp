// crop_box 的单测（assert 风格，host 上 g++ 就能跑）
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "rc2027_perception/crop_box.hpp"

using rc2027_perception::boxes_from_flat;
using rc2027_perception::CropFilter;
using rc2027_perception::Transform34;

namespace
{
int g_failures = 0;
void check(bool c, const std::string & what)
{
  std::cout << (c ? "ok: " : "FAIL: ") << what << std::endl;
  g_failures += !c;
}
}  // namespace

int main()
{
  // 雷达在 base_link (0.2, 0, 0.18)，前下倾 30°（绕 y 转 +30°）：雷达系 → base_link
  const double p = 30.0 * M_PI / 180.0, c = std::cos(p), s = std::sin(p);
  const Transform34 t{c, 0, s, 0.2, 0, 1, 0, 0, -s, 0, c, 0.18};
  // 货仓：底盘顶（0.15）上方 15 cm 起、850 mm 高，在车后
  const auto boxes = boxes_from_flat({-0.40, 0.05, -0.22, 0.22, 0.28, 1.17});
  CropFilter f(boxes, t);

  // base_link 里 (-0.2, 0, 0.8) 的点（货仓里）→ 换到雷达系再喂进去
  auto to_lidar = [&](double bx, double by, double bz, float * out) {
      const double dx = bx - 0.2, dy = by, dz = bz - 0.18;   // R^T * (b - t)
      out[0] = static_cast<float>(c * dx - s * dz);
      out[1] = static_cast<float>(dy);
      out[2] = static_cast<float>(s * dx + c * dz);
    };
  float q[3];
  to_lidar(-0.2, 0.0, 0.8, q);
  check(!f.keep(q[0], q[1], q[2]), "货仓里的点去掉");
  to_lidar(1.5, 0.3, 0.1, q);
  check(f.keep(q[0], q[1], q[2]), "车前 1.5 m 的点留着");
  to_lidar(-0.2, 0.0, 1.3, q);
  check(f.keep(q[0], q[1], q[2]), "货仓上方（框外）的点留着");

  bool threw = false;
  try {
    boxes_from_flat({0, 1, 0, 1, 0});
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "参数个数不是 6 的倍数报错");
  threw = false;
  try {
    boxes_from_flat({1, 0, 0, 1, 0, 1});
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "min > max 报错");
  check(boxes_from_flat({}).empty(), "空参数 = 不裁");

  if (g_failures) {
    std::cerr << g_failures << " 个检查失败" << std::endl;
    return 1;
  }
  std::cout << "全部通过" << std::endl;
  return 0;
}
