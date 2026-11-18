cmake_minimum_required(VERSION 3.8)
project(rc2027_sim_bridge)

find_package(ament_cmake REQUIRED)

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
