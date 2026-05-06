from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('nav2')
    camera_localization_share = get_package_share_directory('camera_localization')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    launch_camera_obstacles = LaunchConfiguration('launch_camera_obstacles')
    camera_global_map_config = LaunchConfiguration('camera_global_map_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_full.yaml'),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'config', 'map_config.yaml'),
        ),
        DeclareLaunchArgument(
            'launch_camera_obstacles',
            default_value='true',
            description='Launch overhead camera obstacle publisher for /camera/block_obstacles',
        ),
        DeclareLaunchArgument(
            'camera_global_map_config',
            default_value=os.path.join(
                camera_localization_share, 'config', 'camera_global_map.yaml'
            ),
            description='Camera localization config used for obstacle publishing',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(camera_localization_share, 'launch', 'global_station.launch.py')
            ),
            condition=IfCondition(launch_camera_obstacles),
            launch_arguments={
                'camera_global_map_config': camera_global_map_config,
                'use_cluster_pipeline': 'false',
                'launch_map_visualizer': 'true',
                'publish_block_obstacles': 'true',
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'full_navigation_launch.launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'map': map_yaml,
            }.items(),
        ),
    ])
