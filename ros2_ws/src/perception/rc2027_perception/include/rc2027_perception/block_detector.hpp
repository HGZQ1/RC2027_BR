// 雷达方块检测——纯 C++，不依赖 ROS，host 上用合成点云单测。
//
// 一个模块管三件事（0002 10-04 定的）：建造点上叠了几层、交接区里找方块、放方块前的局部对齐。
// 输入：map 系点云（Point-LIO 的 /cloud_registered 累几帧，用 TF 转到 map）+ 一个区域
// （建造点 / 交接区，轴对齐矩形）+ 雷达在 map 里的位置。输出：区域里每块方块的
// {中心、朝向（模 90°）、地块 350 / 天空块 200、第几层}，以及"这个建造点有几层、有没有封顶"。
//
// 怎么做的（都是几何，不用训练）：
//   1. 台面高度：区域里 [预期 ± 0.15 m] 的点按 1 cm 分格，点最多的那格就是台面（地面 / L1 / L2）。
//      看不到台面（比如从 L1 看 L2，台面比雷达高）就用预期值
//   2. 台面以上的点按 xy 投影到 3 cm 栅格、8 邻接连通 → 一簇 = 一摞（叠着的方块投影在一起）。
//      高度上有空档（≥ 8 cm）的簇先拆开；底部离台面很高的簇（天空块侧面比地块往里缩 7.5 cm，投影分开了）
//      并到 xy 上最近、顶正好接得上它的底、而且中心落在它脚印里的落地簇；接不上的是悬空的东西（被举着的方块），
//      标 UNKNOWN。比一块方块还大的簇先用 1.5 cm 细栅格再分一次（隔 3～6 cm 的两样东西）
//   3. 一摞的总高度（最高点 - 台面）对上 "n × 0.35 + (0 或 0.2)" 里最近的组合 → 几块地块、有没有天空块
//      （天空块只能在最上面）。差太多 = 不是方块（对方机器人、杂物），标 UNKNOWN
//   4. 每一层单独切片（按高度），拟合正方形：
//      - 朝向：把点转 0～90°，外接矩形面积最小的角度（看到一个面或者两个面都对）
//      - 中心：雷达多半只看得到朝向自己的一两个面。每个轴上，雷达在点的范围外面 → 用离雷达近的
//        那条边往里推半个边长（这条边一定是看得到的面）；雷达在范围里面 → 用范围中点，
//        范围不到 0.8 个边长时标"中心不可靠"
//
// 局限（v1）：两块挨着放（缝 < 3 cm）会连成一簇，尺寸超了标 UNKNOWN（让 TR 留缝）；
// 方块顶部可能出了雷达视场（前倾 30° 时正前方最多往上看 22°），这时层数偏少——会标 top_maybe_clipped。
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace rc2027_perception
{

struct Point3
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

/// 要看的区域：map 系里轴对齐的矩形
struct Region
{
  std::string name;
  double cx{0.0};
  double cy{0.0};
  double half_x{0.4};
  double half_y{0.4};
  double expected_surface_z{0.0};   // 台面高度的预期值（map 系），实际先用点云找
};

/// 雷达（看的人）在 map 里的位置——用来判断看到的是方块哪一面
struct Viewpoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double max_elevation{0.38};       // rad，雷达往上最多看多少（前倾 30° 的 Mid-360 正前方约 22°）
};

enum class BlockType
{
  Ground,   // 地块 350 mm
  Sky,      // 天空块 200 mm
  Unknown,  // 尺寸 / 高度对不上
};

const char * to_string(BlockType type);

struct Block
{
  BlockType type{BlockType::Unknown};
  double x{0.0};                    // map 系中心
  double y{0.0};
  double z_bottom{0.0};             // 底面高度（map 系）
  double yaw{0.0};                  // 模 90°，[-45°, 45°)
  double extent_a{0.0};             // 沿朝向 / 垂直朝向看到的长度（m），检查用
  double extent_b{0.0};
  int layer{1};                     // 从台面数第几层（1 起）
  size_t num_points{0};
  bool center_reliable{false};
};

struct RegionResult
{
  bool visible{false};              // 有没有看到这个区域（台面或者东西）
  double coverage{0.0};             // 区域里 5 cm 小格有点的比例
  double surface_z{0.0};
  bool surface_measured{false};     // 台面高度是测出来的（false = 用的预期值）
  int ground_layers{0};             // 区域中心那一摞：几块地块（顶部出视场时是"至少"）
  bool has_sky{false};              //               ：有没有天空块
  double top_height{0.0};           //               ：最高点离台面多高（m）
  bool top_maybe_clipped{false};    // 最高点贴着雷达视场上沿，可能还有更高的看不到
  size_t num_points{0};             // 区域里的点数
  std::vector<Block> blocks;        // 区域里所有方块（每摞从下往上）
  std::string note;                 // 看不全 / 有不认识的东西
};

struct DetectorParams
{
  double ground_size{0.35};
  double sky_size{0.20};
  double surface_band{0.15};        // m，找台面时在预期值上下多少找
  double min_surface_points{20};    // 台面那一格至少这么多点才算测到
  double min_height{0.03};          // m，离台面这么高以上才算"东西"
  double max_height{1.6};
  double min_stack_height{0.15};    // m，矮于这个的（围栏、台面边缘）不当方块
  double height_tolerance{0.06};    // m，总高度跟组合差多少以内算对上
  double cluster_cell{0.03};        // m，聚类栅格
  size_t min_cluster_points{15};
  double fine_cell{0.015};          // m，比方块大的簇用这个细栅格再分一次
  double max_vertical_gap{0.08};    // m，一簇点在高度上空出这么多就拆开（悬空的东西）
  double max_set_back{0.25};        // m，悬空的簇（上层方块侧面往里缩）离落地簇多远以内算压在它上面
  double size_tolerance{1.3};       // 看到的边长超过 边长 × 这个 + 3 cm 就标 UNKNOWN
  double coverage_cell{0.05};
  double min_coverage{0.2};
};

/// 在一个区域里找方块。points 是 map 系点云（可以是整帧，函数自己裁）
RegionResult detect_blocks(
  const std::vector<Point3> & points, const Region & region, const Viewpoint & view,
  const DetectorParams & params = DetectorParams{});

/// 一摞的总高度 → (地块数, 有没有天空块)；对不上任何组合返回 false
bool decode_stack(
  double height, const DetectorParams & params, int * ground_layers, bool * has_sky);

}  // namespace rc2027_perception
