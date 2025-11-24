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
    default_params = os.path.join(pkg_share, 'config', 'sim_nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')
    params_file  = LaunchConfiguration('params_file')
    log_level    = LaunchConfiguration('log_level')


    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')
    declare_log = DeclareLaunchArgument(
        'log_level', default_value='info')
    declare_params = DeclareLaunchArgument('params_file', default_value=default_params)


    nav2_bringup_share = FindPackageShare('nav2_bringup')

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
        declare_params, 
        declare_log,
        navigation
    ])
