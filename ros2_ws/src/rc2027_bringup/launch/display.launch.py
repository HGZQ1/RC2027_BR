"""没有车也能跑的 URDF 可视化：robot_state_publisher + joint_state_publisher + rviz2。

读 config/robot_params.yaml，把每个尺寸/安装位置转成 xacro:arg 传给
urdf/rc2027_robot.urdf.xacro（跟去年 GAFA rc2026_bringup 的约定一样，
改尺寸只改 robot_params.yaml，不用碰这个文件或 urdf.xacro）。

用法：
    ros2 launch rc2027_bringup display.launch.py

打开后手动在 rviz2 里加 RobotModel + TF 两个 display（没有现成的 .rviz
配置——还没法在真正的 rviz2 里跑一遍确认配置对不对，不想塞一个没验证过的
文件进仓库）。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc2027_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'robot_params.yaml')
    urdf_file = os.path.join(bringup_dir, 'urdf', 'rc2027_robot.urdf.xacro')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)

    chassis = params['chassis']
    wheel = chassis['wheel']
    lidar = params['lidar']
    camera = params['camera']

    def _a(key, value):
        return f' {key}:={value}'

    xacro_args = (
        _a('chassis_width', chassis['width']) +
        _a('chassis_depth', chassis['depth']) +
        _a('chassis_height', chassis['height']) +
        _a('wheel_radius', wheel['radius']) +
        _a('wheel_width', wheel['width']) +
        _a('module_offset_x', chassis['module_offset_x']) +
        _a('module_offset_y', chassis['module_offset_y']) +
        _a('lidar_x', lidar['offset_x']) +
        _a('lidar_y', lidar['offset_y']) +
        _a('lidar_z', lidar['offset_z']) +
        _a('lidar_roll', lidar['roll']) +
        _a('lidar_pitch', lidar['pitch']) +
        _a('lidar_yaw', lidar['yaw']) +
        _a('camera_x', camera['offset_x']) +
        _a('camera_y', camera['offset_y']) +
        _a('camera_z', camera['offset_z']) +
        _a('camera_roll', camera['roll']) +
        _a('camera_pitch', camera['pitch']) +
        _a('camera_yaw', camera['yaw'])
    )

    # urdf_file / use_jsp / use_sim_time 三个参数是给仿真复用的
    # （rc2027_sim_bridge 的 sim_mapping.launch.py 会 include 这个文件，换成
    # 带 Gazebo 插件的 urdf、关掉 joint_state_publisher、打开 sim time），
    # 这样 robot_params.yaml -> xacro 参数这段逻辑只有一份，不在仿真里再抄一遍
    # xacro_overrides 拼在最后：xacro 同名参数后面的覆盖前面的（host 上用 xacro 2.1.1
    # 验证过），所以可以临时改某个尺寸而不动 robot_params.yaml，比如仿真里故意给
    # 雷达一个偏移，验证定位链路的外参换算（0003 决定 8）
    robot_description = ParameterValue(
        Command([
            'xacro ', LaunchConfiguration('urdf_file'), xacro_args, ' ',
            LaunchConfiguration('xacro_overrides'),
        ]), value_type=str)

    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否同时启动 rviz2'),
        DeclareLaunchArgument(
            'urdf_file', default_value=urdf_file,
            description='要展开的 xacro 文件，默认是真车的 rc2027_robot.urdf.xacro'),
        DeclareLaunchArgument(
            'use_jsp', default_value='true',
            description='是否启动 joint_state_publisher（URDF 现在全是固定关节，开不开都'
                        '一样；以后舵轮模块改成 continuous 关节、由串口桥/仿真发 joint_states 时'
                        '要关掉，否则两边抢着发）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='仿真里设成 true'),
        DeclareLaunchArgument(
            'xacro_overrides', default_value='',
            description='追加的 xacro 参数，覆盖 robot_params.yaml 的值，'
                        '例如 "lidar_x:=0.2 lidar_yaw:=0.3"'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_jsp')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
