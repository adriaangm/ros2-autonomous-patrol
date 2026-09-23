"""Launch the patrol mission manager (requires the simulation / Nav2 to be running)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('patrol_bringup')
    default_params = os.path.join(pkg_share, 'config', 'patrol_mission.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the patrol mission parameters file'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock'),
        Node(
            package='patrol_mission',
            executable='patrol_manager_node',
            name='patrol_manager',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
