cmake_minimum_required(VERSION 3.8)
project(rc2027_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionCommand.msg"
  "msg/ActionState.msg"
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
