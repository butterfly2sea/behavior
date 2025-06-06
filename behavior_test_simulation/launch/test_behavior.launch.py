#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    tree_directory_arg = DeclareLaunchArgument(
        'tree_directory',
        default_value='test_trees',
        description='Directory containing behavior tree XML files'
    )

    # 获取包路径
    behavior_test_pkg = FindPackageShare('behavior_test_simulation')
    behavior_node_pkg = FindPackageShare('behavior_node')

    # 仿真节点
    simulation_node = Node(
        package='behavior_test_simulation',
        executable='simulation_node',
        name='simulation_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # 行为树节点
    behavior_control_node = Node(
        package='behavior_node',
        executable='behavior_node',
        name='behavior_control',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'tree_directory': PathJoinSubstitution([
                behavior_test_pkg,
                LaunchConfiguration('tree_directory')
            ]),
            'tick_interval_ms': 50,
            'status_interval_ms': 1000,
            'message_queue_size': 100
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        tree_directory_arg,
        simulation_node,
        behavior_control_node
    ])

if __name__ == '__main__':
    generate_launch_description()