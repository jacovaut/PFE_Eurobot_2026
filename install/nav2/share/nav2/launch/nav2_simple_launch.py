#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    # Get package directory
    nav2_pkg_dir = get_package_share_directory('nav2')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_pkg_dir, 'config', 'nav2_config.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )

    # Load parameters
    param_substitutions = {}
    configured_params = ParameterFile(
        ParameterFile(
            source_file=params_file,
            allow_substs=True,
        ),
        allow_substs=True,
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add Nav2 nodes
    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
    ))

    ld.add_action(Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
    ))

    ld.add_action(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
    ))

    ld.add_action(Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
    ))

    ld.add_action(Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
    ))

    ld.add_action(Node(
        package='nav2_costmap_2d',
        executable='costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[configured_params],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    ))

    ld.add_action(Node(
        package='nav2_costmap_2d',
        executable='costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[configured_params],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    ))

    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator',
                'behavior_server',
                'velocity_smoother',
                'global_costmap',
                'local_costmap'
            ]
        }],
    ))

    return ld