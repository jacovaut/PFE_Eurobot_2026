#!/usr/bin/env python3
"""
Nav2 Launch File for Eurobot 2026

This launch file starts the complete Nav2 navigation stack using the
nav2_bringup package's navigation_launch.py with custom configuration.

It includes:
- Planner Server (path planning)
- Controller Server (velocity control)
- BT Navigator (behavior tree execution)
- Behavior Server (recovery behaviors)
- Velocity Smoother (smooth velocity commands)
- Global Costmap (long-range obstacle avoidance)
- Local Costmap (short-range obstacle avoidance)
- Lifecycle Manager (coordinates node startup/shutdown)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    nav2_pkg_dir = get_package_share_directory('nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch configurations - these can be overridden from command line
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    # Declare launch arguments with default values and descriptions
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_pkg_dir, 'config', 'nav2_config.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # Include the navigation launch file from nav2_bringup
    # This launches all Nav2 components with the specified parameters
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )

    # Create the launch description and add all components
    ld = LaunchDescription()

    # Add the launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    # Add the main navigation launch
    ld.add_action(navigation_launch)

    return ld