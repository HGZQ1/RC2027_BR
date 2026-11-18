#!/bin/bash
# 容器启动入口：依次 source 两层工作空间，再执行传进来的命令。
# 顺序很重要：外部工作空间(驱动+算法)先 source，项目工作空间后 source，
# 保证项目工作空间的包路径优先于外部工作空间被找到（ROS2 的标准约定）。
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f /opt/external_ws/install/setup.bash ]; then
    source /opt/external_ws/install/setup.bash
fi

if [ -f /opt/ros2_ws/install/setup.bash ]; then
    source /opt/ros2_ws/install/setup.bash
fi

exec "$@"
