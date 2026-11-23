cmake_minimum_required(VERSION 3.8)
project(rc2027_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rc2027_navigation_msgs REQUIRED)
# apt 包是 libyaml-cpp-dev，CMake 包名是 yaml-cpp；0.7.x（镜像里会装到的
# 版本）导出的目标名就是裸的 yaml-cpp，不是 yaml-cpp::yaml-cpp（新版本
# upstream 才有命名空间前缀）——这个在 host 上本地编译 yaml-cpp 0.7.0
# 验证过，不是猜的
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# 先验地图配准（0003 决定 9）：small_gicp 是 header-only，vendor 在 localization/registration/third_party/，
# 只需要 Eigen（镜像里 libeigen3-dev，point_lio 也要它）和 OpenMP（GCC 自带 libgomp）
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

# 纯逻辑（不依赖 ROS）：读场地配置、定位链路上的位姿换算、导航控制律，host 上能单测
add_library(localization_core
  localization/src/field_geometry.cpp localization/src/pose_math.cpp navigation/src/point_nav_controller.cpp)
target_include_directories(localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(localization_core yaml-cpp)

# 配准单独一个库：只有 field_localizer 用，别的目标不用跟着依赖 Eigen/OpenMP。
# small_gicp 用 SYSTEM include，它头文件里大量的有/无符号比较警告不刷屏
add_library(map_matching localization/registration/src/map_matcher.cpp)
target_include_directories(map_matching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/localization/registration/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(map_matching SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/localization/registration/third_party/small_gicp/include)
target_link_libraries(map_matching localization_core Eigen3::Eigen OpenMP::OpenMP_CXX)
target_compile_options(map_matching PRIVATE -O2)

# 可执行文件名特意叫 field_localizer（不是源码文件名 field_localizer_node），
# 跟 Python 版的 console_script 名字保持一致——README/0002/0003 里到处
# 写的是 `ros2 run rc2027_navigation field_localizer`，不想因为换语言让
# 这些文档和操作员习惯的命令全部失效
add_executable(field_localizer localization/src/field_localizer_node.cpp)
target_link_libraries(field_localizer localization_core map_matching)
ament_target_dependencies(field_localizer
  rclcpp tf2 tf2_ros geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs rc2027_navigation_msgs)

# 点到点导航（0005）。nav2_msgs 只用它的 NavigateToPose action 定义，不用 Nav2 本体
add_executable(point_navigator navigation/src/point_navigator_node.cpp)
target_link_libraries(point_navigator localization_core)
ament_target_dependencies(point_navigator
  rclcpp rclcpp_action tf2 tf2_ros geometry_msgs nav_msgs std_msgs nav2_msgs rc2027_navigation_msgs)

install(TARGETS field_localizer point_navigator
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS localization_core map_matching
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_floor_height_guard localization/test/test_floor_height_guard_main.cpp)
  target_include_directories(test_floor_height_guard PRIVATE include)
  add_test(NAME test_floor_height_guard COMMAND test_floor_height_guard)
  add_executable(test_field_geometry localization/test/test_field_geometry_main.cpp)
  target_link_libraries(test_field_geometry localization_core)
  # 测试要传真实的 field_geometry.yaml 路径。docs/ 在 ros2_ws 外面，不在
  # 这个包的安装范围内，不能用相对路径（容器里 colcon build 的源码树是
  # /opt/ros2_ws/src/..，跟 docs/ 所在的 /workspace 是两个不同的挂载，
  # 相对路径算出来的是错的，host 上验证这个模块时就是这么踩出来的）。
  # 直接用运行时节点同样的绝对路径约定（见 field_localizer_node.cpp 里
  # field_geometry_path 参数的默认值），这个 ctest 要在挂了 /workspace 的
  # 容器里跑才能找到文件——跟这整个项目"本地开发用 bind mount"的约定一致
  add_test(
    NAME test_field_geometry
    COMMAND test_field_geometry "/workspace/docs/rules/field_geometry.yaml")

  # 位姿换算，不读文件
  add_executable(test_pose_math localization/test/test_pose_math_main.cpp)
  target_link_libraries(test_pose_math localization_core)
  add_test(NAME test_pose_math COMMAND test_pose_math)

  # 导航控制律，理想全向车闭环
  add_executable(test_point_nav_controller navigation/test/test_point_nav_controller_main.cpp)
  target_link_libraries(test_point_nav_controller localization_core)
  add_test(NAME test_point_nav_controller COMMAND test_point_nav_controller)

  # 先验地图配准：用仓库里真实的仿真先验地图（绝对路径，跟 test_field_geometry 同一个约定）
  add_executable(test_map_matcher localization/registration/test/test_map_matcher_main.cpp)
  target_link_libraries(test_map_matcher map_matching)
  add_test(
    NAME test_map_matcher
    COMMAND test_map_matcher "/workspace/sim_ws/src/rc2027_sim_bridge/maps/prior_map_sim.pcd")
endif()

ament_package()
