cmake_minimum_required(VERSION 3.8)
project(rc2027_navigation_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Retry.srv"
  "srv/Relatch.srv"
  "action/NavigateToPoint.action"
  DEPENDENCIES geometry_msgs
)

ament_package()
