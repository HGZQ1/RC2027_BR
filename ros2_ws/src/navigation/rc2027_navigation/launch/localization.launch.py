"""场地定位。Point-LIO 发布 lidar_odom -> imu_link，URDF 接车体静态外参。

field_localizer 查询 lidar_odom -> base_footprint 来锁定场地高度，发布 map -> lidar_odom。
prior_map_path 指向先验地图时开启 ICP 精修。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'side', default_value='red',
            description="红方/蓝方：'red' 或 'blue'"),
        DeclareLaunchArgument(
            'start_box', default_value='box_b',
            description="Start Zone 两个 700mm 子格子选哪个：'box_a'(TR/靠边) 或 "
                        "'box_b'(BR/靠里，本仓库默认)"),
        DeclareLaunchArgument(
            'start_zone', default_value='start',
            description="开机锁定到哪个 zone：'start'（比赛开局）或 'l1'（仿真直接从 L1 Retry Zone 开始）"),
        DeclareLaunchArgument(
            'field_geometry_path',
            default_value='/workspace/docs/rules/field_geometry.yaml',
            description='field_geometry.yaml 的绝对路径（容器里挂载点固定，见 docker-compose.yml）'),
        DeclareLaunchArgument('motion_icp_probe', default_value='false'),
        DeclareLaunchArgument('z_check_offset', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'prior_map_path', default_value='',
            description='先验地图 PCD 的绝对路径，空 = 不做 ICP 精修'),

        Node(
            package='rc2027_navigation',
            executable='field_localizer',
            name='field_localizer',
            output='screen',
            parameters=[{
                'side': LaunchConfiguration('side'),
                'start_box': LaunchConfiguration('start_box'),
                'start_zone': LaunchConfiguration('start_zone'),
                'field_geometry_path': LaunchConfiguration('field_geometry_path'),
                'motion_icp_probe': ParameterValue(LaunchConfiguration('motion_icp_probe'), value_type=bool),
                'prior_map_path': LaunchConfiguration('prior_map_path'),
                'z_check_offset': ParameterValue(LaunchConfiguration('z_check_offset'), value_type=float),
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
