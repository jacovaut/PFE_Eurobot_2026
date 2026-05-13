from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paths to other launch files
    local_camera_launch = os.path.join(
        get_package_share_directory('local_camera'), 'launch', 'local_camera_system.launch.py')
    manip_action_launch = os.path.join(
        get_package_share_directory('manip_action_node'), 'launch', 'manip_actions.launch.py')
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='false',
        description='Enable YDLidar nodes.'
    )
    team_color_arg = DeclareLaunchArgument(
        'team_color',
        default_value='blue',
        description='Team color for local pickup scoring (blue|yellow, bleu|jaune).'
    )

    return LaunchDescription([
        enable_lidar_arg,
        team_color_arg,
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
