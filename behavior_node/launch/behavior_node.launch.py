from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='behavior_config.yaml',
        description='Configuration file name'
    )

    tree_dir_arg = DeclareLaunchArgument(
        'tree_dir',
        default_value='trees',
        description='Behavior tree directory'
    )

    # 获取包路径
    package_share = FindPackageShare('behavior_node')

    # 配置文件路径
    config_file = PathJoinSubstitution([
        package_share, 'config', LaunchConfiguration('config_file')
    ])

    # 行为树目录路径
    tree_directory = PathJoinSubstitution([
        package_share, LaunchConfiguration('tree_dir')
    ])

    # 创建behavior_node节点
    behavior_node = Node(
        package='behavior_node',
        executable='behavior_node',
        name='behavior_node',
        parameters=[
            config_file,
            {
                'tree_directory': tree_directory,
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        tree_dir_arg,
        behavior_node
    ])