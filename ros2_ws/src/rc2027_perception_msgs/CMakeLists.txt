cmake_minimum_required(VERSION 3.8)
project(rc2027_perception_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Block.msg"
  "msg/RegionState.msg"
  "msg/RegionStateArray.msg"
  "msg/SpotState.msg"
  "msg/FieldState.msg"
  "srv/DetectBlocks.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
