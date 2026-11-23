"""雷达方块检测 block_detector。

前提：定位链路在跑（TF 里有 map -> lidar_odom -> ... -> lidar_link），Point-LIO 在发 /cloud_registered。

用法：
    ros2 launch rc2027_perception block_detector.launch.py                    # 真车
    ros2 launch rc2027_perception block_detector.launch.py sim:=true    # 仿真（台面高 5 cm、sim 时间）
    ros2 service call /block_detector/detect rc2027_perception_msgs/srv/DetectBlocks \\
      "{region: spot_03}"
    ros2 topic echo /block_detector/regions
RViz 加 MarkerArray /block_detector/markers。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(context):
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    params = [os.path.join(get_package_share_directory('rc2027_perception'),
                           'config', 'block_detector.yaml')]
    if sim:
        params.append({'surface_z_offset': 0.05, 'use_sim_time': True})
    return [Node(
        package='rc2027_perception',
        executable='block_detector',
        name='block_detector',
        output='screen',
        parameters=params,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true = 仿真（台面整体高 5 cm、用 sim 时间）'),
        OpaqueFunction(function=_node),
    ])
