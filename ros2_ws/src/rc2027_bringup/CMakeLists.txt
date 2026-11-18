cmake_minimum_required(VERSION 3.8)
project(rc2027_bringup)

find_package(ament_cmake REQUIRED)

# rviz/ 还没加——没有一个经过真正 rviz2 验证过的配置，不想塞一个没验证过的
# 文件进仓库（见 README.md）。以后加了 .rviz 配置，这里要把 rviz 加回来
install(DIRECTORY
  urdf
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
