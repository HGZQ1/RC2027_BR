"""点到点导航 point_navigator，见 docs/decisions/0005-navigation.md。

前提：定位链路已经在跑（localization.launch.py），TF 里能查到 map -> base_link。

用法：
    ros2 launch rc2027_navigation navigation.launch.py
    ros2 launch rc2027_navigation navigation.launch.py use_sim_time:=true

发目标：
    RViz 工具栏 "2D Goal Pose"（发 /goal_pose，固定帧要设成 map）
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
      "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}" --feedback
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _navigator(context):
    params = [LaunchConfiguration('params_file').perform(context)]
    # extra_params_file：只写要覆盖的几项（比如仿真降速），后加载的覆盖前面的
    extra = LaunchConfiguration('extra_params_file').perform(context)
    if extra:
        params.append(extra)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    params.append({'use_sim_time': use_sim_time})
    return [Node(
        package='rc2027_navigation',
        executable='point_navigator',
        name='point_navigator',
        output='screen',
        parameters=params,
    )]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('rc2027_navigation'), 'config', 'point_navigator.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'extra_params_file', default_value='',
            description='可选：覆盖 params_file 里部分参数的 yaml（仿真降速用）'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_navigator),
    ])
