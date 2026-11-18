"""定位链路：lio_to_base + field_localizer，见 docs/decisions/0003-field-localization.md 决定 8。

TF 链：
    map --(field_localizer)--> lidar_odom --(lio_to_base)--> base_footprint --(URDF)--> base_link ...

前提（这个 launch 不负责起）：
  - Point-LIO 在跑，并且 **publish.tf_send_en 必须是 false**（它自己发的
    lidar_odom -> lidar_link 会跟 URDF 抢 lidar_link 的父节点）
  - robot_state_publisher 在跑（lio_to_base 要从 TF 查雷达外参）

prior_map_path：先验地图 PCD（map 系）。给了就开 ICP 精修（锁定后自动做一次 +
/field_localizer/relocalize 服务，0003 决定 9）；空 = 只靠锁定。仿真用
rc2027_sim_bridge/maps/prior_map_sim.pcd（sim_mapping.launch.py 已经传了）。

用法：
    ros2 launch rc2027_navigation localization.launch.py side:=red
    ros2 launch rc2027_navigation localization.launch.py side:=blue start_box:=box_a

start_box 默认 box_b：本仓库这台车是 BR，2026-10-03 暂定"TR 用靠边的 box_a、
BR 用靠里的 box_b"，见 0003 和 field_geometry.yaml 的 start_zone 注释。

body_frame：URDF 里跟 Point-LIO 的 "body"（它的 IMU 系）重合的坐标系。
真车默认 lidar_imu_link（Mid-360 内置 IMU）；仿真传 livox_sim（雷达和 IMU
挂在同一个 link 上，仿真配置外参是 0），见 rc2027_sim_bridge 的 launch。
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
        DeclareLaunchArgument(
            'body_frame', default_value='lidar_imu_link',
            description='URDF 里跟 Point-LIO 的 body（IMU 系）重合的坐标系'),
        DeclareLaunchArgument('z_check_offset', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'prior_map_path', default_value='',
            description='先验地图 PCD 的绝对路径，空 = 不做 ICP 精修'),

        Node(
            package='rc2027_navigation',
            executable='lio_to_base',
            name='lio_to_base',
            output='screen',
            parameters=[{
                'body_frame': LaunchConfiguration('body_frame'),
                'use_sim_time': use_sim_time,
            }],
        ),
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
                'prior_map_path': LaunchConfiguration('prior_map_path'),
                'z_check_offset': ParameterValue(LaunchConfiguration('z_check_offset'), value_type=float),
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
