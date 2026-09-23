"""Simulation bringup: TurtleBot4 + Nav2 in the patrol warehouse world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('patrol_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_world = os.path.join(pkg_share, 'worlds', 'patrol_warehouse.sdf.xacro')
    default_map = os.path.join(pkg_share, 'maps', 'patrol_warehouse.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    declared_arguments = [
        DeclareLaunchArgument(
            'slam', default_value='False',
            description='Run SLAM Toolbox to build a new map instead of AMCL localization'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Full path to the map YAML used for localization'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the Nav2 parameters file'),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Full path to the Gazebo world (SDF or SDF xacro)'),
        DeclareLaunchArgument(
            'headless', default_value='False',
            description='Run Gazebo without GUI'),
        DeclareLaunchArgument(
            'use_rviz', default_value='True',
            description='Start RViz'),
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='Robot spawn X in the world frame [m]'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Robot spawn Y in the world frame [m]'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='Robot spawn yaw in the world frame [rad]'),
    ]

    tb4_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'tb4_simulation_launch.py')),
        launch_arguments={
            'slam': slam,
            'map': map_yaml,
            'params_file': params_file,
            'world': world,
            'headless': headless,
            'use_rviz': use_rviz,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
        }.items(),
    )

    return LaunchDescription([*declared_arguments, tb4_simulation])
