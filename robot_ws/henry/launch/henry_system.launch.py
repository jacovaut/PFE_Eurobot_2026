import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths to other launch files
    local_camera_launch = os.path.join(
        get_package_share_directory('local_camera'), 'launch', 'local_camera_system.launch.py')
    manip_action_launch = os.path.join(
        get_package_share_directory('manip_action_node'), 'launch', 'manip_actions.launch.py')
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable YDLidar nodes.'
    )
    team_color_arg = DeclareLaunchArgument(
        'team_color',
        default_value='blue',
        description='Team color for local pickup scoring (blue|yellow, bleu|jaune).'
    )
    start_gpio_pin_arg = DeclareLaunchArgument(
        'start_gpio_pin',
        default_value='22',
        description='BCM GPIO pin used as the match start signal.'
    )
    start_gpio_active_high_arg = DeclareLaunchArgument(
        'start_gpio_active_high',
        default_value='true',
        description='Publish match/start when the configured GPIO is high.'
    )
    start_gpio_poll_sec_arg = DeclareLaunchArgument(
        'start_gpio_poll_sec',
        default_value='0.05',
        description='GPIO polling interval in seconds while waiting for match start.'
    )

    return LaunchDescription([
        enable_lidar_arg,
        team_color_arg,
        start_gpio_pin_arg,
        start_gpio_active_high_arg,
        start_gpio_poll_sec_arg,
        Node(
            package='match',
            executable='match_node',
            name='match_node',
            output='screen',
            parameters=[{
                'autostart': False,
                'duration_s': 98.0,
                'force_running': False,
                'team_color': LaunchConfiguration('team_color'),
            }],
        ),
        Node(
            package='henry',
            executable='gpio_match_start',
            name='gpio_match_start',
            output='screen',
            parameters=[{
                'pin': LaunchConfiguration('start_gpio_pin'),
                'active_high': LaunchConfiguration('start_gpio_active_high'),
                'poll_sec': LaunchConfiguration('start_gpio_poll_sec'),
                'start_topic': '/match/start',
                'latch_start': True,
            }],
        ),
        Node(
            package='match',
            executable='cmd_vel_match_gate',
            name='cmd_vel_match_gate',
            output='screen',
            parameters=[{
                'input_topic': '/cmd_vel_match_input',
                'output_topic': '/cmd_vel_smoothed',
                'match_running_topic': '/match/running',
                'zero_publish_hz': 20.0,
                'lidar_stop_enabled': True,
                'scan_topic': '/scan',
                'lidar_stop_distance_m': 0.40,
                'lidar_clear_distance_m': 0.45,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(local_camera_launch),
            launch_arguments={
                'enable_lidar': LaunchConfiguration('enable_lidar'),
                'team_color': LaunchConfiguration('team_color'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manip_action_launch),
        ),
    ])
