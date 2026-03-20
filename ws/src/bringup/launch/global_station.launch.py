from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')
    default_camera_map = os.path.join(pkg_share, 'config', 'camera_global_map.yaml')

    camera_map_config = LaunchConfiguration('camera_global_map_config')
    global_camera_index = LaunchConfiguration('global_camera_index')
    use_cluster_analyze = LaunchConfiguration('use_cluster_analyze')
    cluster_team_color = LaunchConfiguration('cluster_team_color')
    cluster_show_debug_window = LaunchConfiguration('cluster_show_debug_window')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_global_map_config',
            default_value=default_camera_map,
            description='World map + camera config YAML for global_localization_node'
        ),
        DeclareLaunchArgument(
            'global_camera_index',
            default_value='0',
            description='Camera index for the overhead global camera on this machine'
        ),
        DeclareLaunchArgument(
            'use_cluster_analyze',
            default_value='true',
            description='Launch strategy cluster analysis node on global station'
        ),
        DeclareLaunchArgument(
            'cluster_team_color',
            default_value='jaune',
            description='Team color for cluster scoring (jaune|bleu)'
        ),
        DeclareLaunchArgument(
            'cluster_show_debug_window',
            default_value='true',
            description='Show OpenCV debug window for cluster analysis'
        ),

        # Overhead camera: detects ArUco markers, publishes /camera/global_pose (map frame).
        Node(
            package='camera_localization',
            executable='global_localization_node',
            name='global_localization_node',                                                                                                                                  
            output='screen',
            parameters=[camera_map_config, {
                'camera_index': global_camera_index,
            }],
        ),

        # Cluster analysis node: consumes /detected_blocks and publishes /cluster_info
        Node(
            package='pfe',
            executable='cluster_analyze_node',
            name='cluster_analyze_node',
            output='screen',
            condition=IfCondition(use_cluster_analyze),
            parameters=[{
                'team_color': cluster_team_color,
                'show_debug_window': cluster_show_debug_window,
            }],
        ),
    ])
