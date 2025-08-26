#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('spot_nav2')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')
    # map_yaml     = LaunchConfiguration('map_yaml')
    params_file  = LaunchConfiguration('params_file')
    log_level    = LaunchConfiguration('log_level')


    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')
    # declare_map = DeclareLaunchArgument(
    #     'map_yaml', default_value=None)
    declare_log = DeclareLaunchArgument(
        'log_level', default_value='info')
    declare_params = DeclareLaunchArgument('params_file', default_value=default_params)


    nav2_bringup_share = FindPackageShare('nav2_bringup')

    # 1) Start map_server as a lifecycle node (no AMCL)
    # map_server = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time,
    #                  'yaml_filename': map_yaml}],
    #     arguments=['--ros-args', '--log-level', log_level]
    # )

    # Lifecycle manager to configure/activate map_server
    # lm_map = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_map',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time,
    #                  'autostart': autostart,
    #                  'node_names': ['map_server']}],
    #     arguments=['--ros-args', '--log-level', log_level]
    # )

    # 2) Start the rest of Nav2 (planner, controller, BT navigator, etc.)
    nav2_bringup_share = FindPackageShare('nav2_bringup')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'log_level': log_level
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time, 
        declare_autostart, 
        # declare_map, 
        declare_params, 
        declare_log,
        # map_server, 
        # lm_map, 
        navigation
    ])
