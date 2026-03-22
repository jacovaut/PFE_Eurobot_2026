from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('nav2_all_bringups')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_full.yaml'),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'full_navigation_launch.launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'map': '',
            }.items(),
        ),
    ])
