cmake_minimum_required(VERSION 3.8)
project(rc2027_sim_bridge)

find_package(ament_cmake REQUIRED)

find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(rc2027_planar_drive SHARED src/planar_drive.cpp)
target_compile_features(rc2027_planar_drive PRIVATE cxx_std_17)
ament_target_dependencies(rc2027_planar_drive gazebo_dev gazebo_ros geometry_msgs nav_msgs rclcpp)
install(TARGETS rc2027_planar_drive LIBRARY DESTINATION lib)

install(DIRECTORY
  config
  launch
  maps
  rviz
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
