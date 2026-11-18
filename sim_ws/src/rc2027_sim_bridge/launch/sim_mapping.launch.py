"""Gazebo 仿真里放车 + 仿真 Mid-360 + Point-LIO 建图。

起这些东西：
  1. Gazebo Classic 11，world 用本包 worlds/rc2027_sim.world（场地模型还是
     robocon_2027_gazebo 的，只改了物理 500 Hz、关阴影，为了性能）
  2. robot_state_publisher：include rc2027_bringup/display.launch.py，换成
     本包的 rc2027_robot_sim.urdf.xacro（真车 URDF + Gazebo 插件），关掉
     joint_state_publisher（URDF 现在全是固定关节，用不上），打开 sim time
  3. spawn_entity：把车放在 docs/rules/field_geometry.yaml 的 start_zone
     （默认红方 box_b，即 BR 的位置，朝 +y）。Gazebo 世界坐标和 field.xml
     坐标系一致（核对过储存区网格的位置），所以直接用 yaml 里的数
  4. Point-LIO（pointlio_mapping 节点，用本包 config/point_lio_mid360_sim.yaml，
     TF 关掉）——**spawn 完成后再等 3 s 才起**，让 IMU 初始化用的是车停稳以后的数据
  5. 定位链路（rc2027_navigation/localization.launch.py：lio_to_base +
     field_localizer，body_frame=livox_sim），补上 map -> lidar_odom ->
     base_footprint，teach_point/导航查 map -> base_link 才有值（0003 决定 8）
  6. 点到点导航（rc2027_navigation/navigation.launch.py：point_navigator，0005），
     仿真降速（config/point_navigator_sim.yaml）。
     RViz 里把固定帧改成 map，用工具栏 "2D Goal Pose" 点目标；或者
     ros2 action send_goal /navigate_to_pose ...（见 navigation.launch.py）
  7. RViz（本包 rviz/sim.rviz，固定帧 map）
  定位链路带先验地图 ICP 精修（maps/prior_map_sim.pcd，0003 决定 9）

开车（另开一个终端）：
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

用法：
    ros2 launch rc2027_sim_bridge sim_mapping.launch.py
    ros2 launch rc2027_sim_bridge sim_mapping.launch.py side:=blue gui:=false
    # 故意放偏 10 cm / 3°，看 ICP 开局精修能不能拉回来（field_localizer 仍按 zone 中心锁定）
    ros2 launch rc2027_sim_bridge sim_mapping.launch.py spawn_dx:=0.1 spawn_dyaw_deg:=3
    # 故意让雷达偏心+转个角，验证外参换算（对照 ground_truth_check 的误差）
    ros2 launch rc2027_sim_bridge sim_mapping.launch.py xacro_overrides:="lidar_x:=0.2 lidar_yaw:=0.3"
    # 爬楼梯回放（测上楼后的定位、L1 Retry），起来以后另开终端 ros2 run rc2027_utils sim_climb_replay
    ros2 launch rc2027_sim_bridge sim_mapping.launch.py replay:=true
    # 电脑卡：不开画面 + 雷达点数再减半
    ros2 launch rc2027_sim_bridge sim_mapping.launch.py gui:=false xacro_overrides:="sim_lidar_samples:=10000"

2026-10-04 已在容器里跑通建图；之后的改动（性能、延迟启动）没跑过，见本包 README。
"""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
    SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 车 spawn 完（spawn_entity.py 进程退出）之后再等这么久才起 Point-LIO（墙钟秒）。
# 2026-10-04 助教从日志看到：原来 Point-LIO 和 spawn 同时起，IMU 初始化（估重力方向和
# 零偏，前提是车完全静止）只比 spawn 晚 0.6 s，正好用上了车落地弹跳的数据，重力方向
# 估歪了，后面一直抖。RTF 不到 1 时墙钟 3 s 对应的仿真时间更短，但落 5 cm 足够停稳
POINT_LIO_DELAY_AFTER_SPAWN = 3.0


def _spawn_robot_then_point_lio(context):
    side = LaunchConfiguration('side').perform(context)
    start_box = LaunchConfiguration('start_box').perform(context)
    geometry_path = LaunchConfiguration('field_geometry_path').perform(context)

    with open(geometry_path, encoding='utf-8') as f:
        geometry = yaml.safe_load(f)
    start_zone = LaunchConfiguration('start_zone').perform(context)
    if start_zone == 'l1':
        cfg = geometry['l1_retry_zone'][side]
    else:
        cfg = geometry['start_zone'][side][start_box]
    x, y = cfg['center']
    yaw = cfg['yaw']
    # Gazebo 场地整体比规则高 5 cm（地面 0.05、L1 顶 0.65）；地面上原来就是 z = 0.05
    surface_z = float(cfg.get('z', 0.0)) + 0.05
    # 故意放偏（测 ICP 开局精修能不能拉回来）：field_localizer 仍然按 zone 中心锁定
    replay = LaunchConfiguration('replay').perform(context).lower() == 'true'
    x += float(LaunchConfiguration('spawn_dx').perform(context))
    y += float(LaunchConfiguration('spawn_dy').perform(context))
    yaw += math.radians(float(LaunchConfiguration('spawn_dyaw_deg').perform(context)))

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rc2027_br',
            '-x', str(x), '-y', str(y),
            # Start Zone 网格在 z≈0.025，轮子底面在 base_footprint 的 z=0。
            # 原来从 0.1 放下去，落地冲击混进了 Point-LIO 的 IMU 初始化；改成 0.05，
            # 只落两三厘米
            # 回放模式车没有重力，放在地面上方 1 cm（跟回放轨迹的 clearance 一致），不碰地
            '-z', str(surface_z + 0.01) if replay else str(surface_z),
            '-Y', str(yaw),
        ],
    )

    point_lio_params = [
        os.path.join(
            get_package_share_directory('rc2027_sim_bridge'),
            'config', 'point_lio_mid360_sim.yaml'),
        {'use_sim_time': True},
    ]
    # 裁剪框开着时 Point-LIO 吃裁过的点云（cloud_crop 去掉车体 + 货仓里的点，2026-10-05）
    if LaunchConfiguration('cloud_crop').perform(context).lower() == 'true':
        point_lio_params.append({'common.lid_topic': '/cloud_crop/custom'})
    point_lio = Node(
        package='point_lio',
        executable='pointlio_mapping',
        output='screen',
        parameters=point_lio_params,
    )

    return [
        spawn,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=[TimerAction(period=POINT_LIO_DELAY_AFTER_SPAWN, actions=[point_lio])],
        )),
    ]


def generate_launch_description():
    sim_share = get_package_share_directory('rc2027_sim_bridge')
    bringup_share = get_package_share_directory('rc2027_bringup')
    field_share = get_package_share_directory('robocon_2027_gazebo')
    nav_share = get_package_share_directory('rc2027_navigation')

    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    sim_urdf = os.path.join(sim_share, 'urdf', 'rc2027_robot_sim.urdf.xacro')

    return LaunchDescription([
        # 死进程留在 /dev/shm 里的 fastrtps 锁会让每个节点刷 open_and_lock_file failed
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(sim_share, 'config', 'fastdds_no_shm.xml')),
        DeclareLaunchArgument('side', default_value='red', description="'red' 或 'blue'"),
        DeclareLaunchArgument(
            'start_box', default_value='box_b',
            description="Start Zone 子格子，BR 默认 box_b（见 field_geometry.yaml）"),
        DeclareLaunchArgument(
            'start_zone', default_value='start',
            description="'start'（地面 Start Zone）或 'l1'（直接放在 L1 Retry Zone 开机，"
                        "测 L1 上的方块检测 / 定位用，不用先回放爬楼梯）"),
        DeclareLaunchArgument(
            'field_geometry_path',
            default_value='/workspace/docs/rules/field_geometry.yaml'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='是否开 gzclient（不看画面就关掉，省 CPU/显卡）'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(sim_share, 'worlds', 'rc2027_sim.world'),
            description='默认本包的 rc2027_sim.world（物理 500 Hz、无阴影）；想用原版传 '
                        'robocon_2027_gazebo 的 worlds/robocon_2027_classic.world'),
        DeclareLaunchArgument('rviz', default_value='true', description='是否开 RViz'),
        DeclareLaunchArgument(
            'localization', default_value='true',
            description='是否起定位链路（lio_to_base + field_localizer）'),
        DeclareLaunchArgument(
            'navigation', default_value='true',
            description='是否起点到点导航 point_navigator（需要定位链路）'),
        DeclareLaunchArgument(
            'replay', default_value='false',
            description='爬楼梯回放模式：车关重力、去掉 planar_move，由 rc2027_utils 的 '
                        'sim_climb_replay 设置位姿（测高度变化后的定位）'),
        DeclareLaunchArgument('spawn_dx', default_value='0.0', description='放车 x 偏移（m）'),
        DeclareLaunchArgument('spawn_dy', default_value='0.0', description='放车 y 偏移（m）'),
        DeclareLaunchArgument(
            'spawn_dyaw_deg', default_value='0.0', description='放车朝向偏移（度）'),
        DeclareLaunchArgument(
            'prior_map_path',
            default_value=os.path.join(sim_share, 'maps', 'prior_map_sim.pcd'),
            description='先验地图（ICP 精修用），传空字符串关掉 ICP'),
        DeclareLaunchArgument(
            'cloud_crop', default_value='true',
            description='true = 起 cloud_crop（车体 + 货仓裁剪框），Point-LIO 和方块检测都吃裁过的点云'),
        DeclareLaunchArgument(
            'sim_cargo', default_value='false',
            description='true = 车后挂一个 850 mm 高的"货仓方块"盒子（测裁剪框）'),
        DeclareLaunchArgument(
            'xacro_overrides', default_value='',
            description='覆盖 robot_params.yaml 的 xacro 参数，如 "lidar_x:=0.2 lidar_yaw:=0.3"'),

        # 跟 robocon_2027_field.launch.py 做的一样（让 model://robocon_2027_field 找得到），
        # 只是 world 换成本包的
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', field_share),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gui'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'display.launch.py')),
            launch_arguments={
                'urdf_file': sim_urdf,
                'use_jsp': 'false',
                'use_rviz': 'false',
                'use_sim_time': 'true',
                'xacro_overrides': [
                    LaunchConfiguration('xacro_overrides'), ' sim_replay:=',
                    LaunchConfiguration('replay'), ' sim_cargo:=', LaunchConfiguration('sim_cargo')],
            }.items(),
        ),

        OpaqueFunction(function=_spawn_robot_then_point_lio),

        Node(
            package='rc2027_perception',
            executable='cloud_crop',
            name='cloud_crop',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('rc2027_perception'),
                             'config', 'cloud_crop.yaml'),
                {'use_sim_time': True},
            ],
            condition=IfCondition(LaunchConfiguration('cloud_crop')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'localization.launch.py')),
            launch_arguments={
                'side': LaunchConfiguration('side'),
                'start_box': LaunchConfiguration('start_box'),
                'start_zone': LaunchConfiguration('start_zone'),
                'field_geometry_path': LaunchConfiguration('field_geometry_path'),
                # 仿真里雷达和 IMU 挂在同一个 link（livox_sim）上
                'body_frame': 'livox_sim',
                'z_check_offset': '0.05',
                'use_sim_time': 'true',
                'prior_map_path': LaunchConfiguration('prior_map_path'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('localization')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                # 仿真里先降速：雷达插件逐点时间戳是假的（NOTES 坑 3），转得快点云就畸变，
                # Point-LIO 容易发散。稳定了再往上调
                'extra_params_file': os.path.join(sim_share, 'config', 'point_navigator_sim.yaml'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('navigation')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            # 本包自己的 RViz 配置（2026-10-04）：固定帧 map、点云只留 5 s、显示先验地图和车模型。
            # Point-LIO 自带的 loam_livox.rviz 点云留 30 s（约 600 万点）+ 无限长的 /path，
            # 跑久了 RViz 卡死（助教 0002）
            arguments=['-d', os.path.join(sim_share, 'rviz', 'sim.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
