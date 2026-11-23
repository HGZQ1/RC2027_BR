# small_gicp — vendor 说明

- 上游：https://github.com/koide3/small_gicp
- 版本：`fa0cfc983417c044b6036f71dfa448038bb9c820`（2026-09-29，`master`），2026-10-04 拉取
- 许可证：MIT（同目录 `LICENSE`）
- 用途：`rc2027_navigation` 的先验地图配准（`src/map_matcher.cpp`，0003 决定 9）。组长建议用它

## 为什么 vendor 而不是 apt / 编进镜像

- ROS Humble 的 rosdistro 里**没有** small_gicp（查过 `humble/distribution.yaml`，只有它的前身
  fast_gicp），apt 装不了
- 它是 **header-only** 的（README 原话），只要 Eigen（镜像里 `libeigen3-dev`）+ OpenMP（GCC 自带），
  直接放进包里就能用，不用改 Dockerfile、不用重 build 镜像
- header-only 还有个好处：host 上下一份 Eigen 头文件就能编译单测，用真实先验地图跑配准
  （`test/test_map_matcher_main.cpp`）

## 拿了什么、删了什么

只拿了 `include/small_gicp/` + `LICENSE`，**头文件一个字没改**。删掉了用不上的：
- `include/small_gicp/pcl/`（PCL 接口，要 PCL）、`ros/`、`benchmark/`
- 上游的 `src/`（`registration_helper.cpp` 里的 `align()` 便捷函数——我们用模板接口
  `Registration<GICPFactor, ParallelReductionOMP>`，不需要它）、`python/`、例子、测试
- `*_tbb.hpp` 留着没删，但不会被 include（不需要 TBB）

头文件里有不少有/无符号比较的警告，CMake 里用 `SYSTEM` include 屏蔽了。
