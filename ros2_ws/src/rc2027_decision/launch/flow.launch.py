"""Flow 引擎：flow_runner + 世界模型 field_state + 假电控 mock_action_executor。

前提：定位（localization.launch.py）和导航（navigation.launch.py）已经在跑——仿真里
sim_mapping.launch.py 默认都带上了。

用法（仿真）：
    ros2 launch rc2027_decision flow.launch.py use_sim_time:=true
    ros2 service call /flow_runner/start std_srvs/srv/Trigger
    ros2 topic echo /flow_runner/status

    换流程文件 / 点文件：flows:=<yaml> points:=<yaml>
    只测流程逻辑、动作不等：time_scale:=0
    测动作失败分支：fail_actions:="['PLACE']"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    use_sim_time = arg('use_sim_time').lower() == 'true'
    nodes = [Node(
        package='rc2027_decision',
        executable='flow_runner',
        name='flow_runner',
        output='screen',
        parameters=[{
            'flows_path': arg('flows'),
            'points_path': arg('points'),
            'autostart': arg('autostart').lower() == 'true',
            'planner': arg('planner').lower() == 'true',
            'use_sim_time': use_sim_time,
        }],
    )]
    if arg('field_state').lower() == 'true':
        nodes.append(Node(
            package='rc2027_decision',
            executable='field_state',
            name='field_state',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ))
    if arg('mock_actions').lower() == 'true':
        fail = [a for a in arg('fail_actions').strip('[]').replace("'", '').split(',') if a]
        nodes.append(Node(
            package='rc2027_decision',
            executable='mock_action_executor',
            name='mock_action_executor',
            output='screen',
            parameters=[{
                'time_scale': float(arg('time_scale')),
                'fail_actions': [a.strip() for a in fail] or [''],
                'use_sim_time': use_sim_time,
            }],
        ))
    return nodes


def generate_launch_description():
    flows = os.path.join(get_package_share_directory('rc2027_decision'),
                         'config', 'flows', 'route_demo.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('flows', default_value=flows, description='流程文件'),
        DeclareLaunchArgument(
            'points', default_value='/workspace/docs/field_data/sim_red.yaml',
            description='teach_point 录的点文件'),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='true = 启动 1 s 后自动跑主流程'),
        DeclareLaunchArgument('planner', default_value='false',
                              description='true = 规划器选下一件再压栈，不跑写死的主流程'),
        DeclareLaunchArgument('mock_actions', default_value='true',
                              description='true = 起假电控（串口桥接好以前）'),
        DeclareLaunchArgument('time_scale', default_value='1.0',
                              description='假电控动作时长系数，0 = 立刻完成'),
        DeclareLaunchArgument('fail_actions', default_value='[]',
                              description="假电控回 FAILED 的动作，如 \"['PLACE']\""),
        DeclareLaunchArgument('field_state', default_value='true',
                              description='true = 起世界模型 /field_state（要 block_detector 在跑）'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
