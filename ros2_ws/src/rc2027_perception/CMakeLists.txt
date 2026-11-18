cmake_minimum_required(VERSION 3.8)
project(rc2027_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rc2027_perception_msgs REQUIRED)
# 雷达原始点云的消息类型（external_ws 里编的，容器里已经 source 了）
find_package(livox_ros_driver2 REQUIRED)
# 跟 rc2027_navigation 一样：0.7.x 导出的目标名是裸的 yaml-cpp
find_package(yaml-cpp REQUIRED)

# 纯算法（不依赖 ROS），host 上能单测
add_library(block_detection src/block_detector.cpp src/crop_box.cpp)
target_include_directories(block_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(block_detection PRIVATE -O2)

add_executable(block_detector src/block_detector_node.cpp)
target_link_libraries(block_detector block_detection yaml-cpp)
ament_target_dependencies(block_detector
  rclcpp sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros rc2027_perception_msgs)

add_executable(cloud_crop src/cloud_crop_node.cpp)
target_link_libraries(cloud_crop block_detection)
ament_target_dependencies(cloud_crop rclcpp sensor_msgs tf2 tf2_ros livox_ros_driver2)

install(TARGETS block_detection block_detector cloud_crop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_block_detector test/test_block_detector_main.cpp)
  target_link_libraries(test_block_detector block_detection)
  add_test(NAME test_block_detector COMMAND test_block_detector)
  add_executable(test_crop_box test/test_crop_box_main.cpp)
  target_link_libraries(test_crop_box block_detection)
  add_test(NAME test_crop_box COMMAND test_crop_box)
endif()

ament_package()
