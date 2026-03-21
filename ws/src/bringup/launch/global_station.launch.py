from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _read_cluster_pipeline_default(camera_map_path: str) -> str:
    try:
        with open(camera_map_path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        enabled = data.get('global_localization_node', {}).get('ros__parameters', {}).get('use_cluster_pipeline', True)
        return 'true' if bool(enabled) else 'false'
    except Exception:
        return 'true'

def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')
    default_camera_map = os.path.join(pkg_share, 'config', 'camera_global_map.yaml')
    default_use_cluster_pipeline = _read_cluster_pipeline_default(default_camera_map)

    camera_map_config = LaunchConfiguration('camera_global_map_config')
    use_cluster_pipeline = LaunchConfiguration('use_cluster_pipeline')
    cluster_team_color = LaunchConfiguration('cluster_team_color')
    cluster_show_debug_window = LaunchConfiguration('cluster_show_debug_window')
    cluster_goal_min_score = LaunchConfiguration('cluster_goal_min_score')
    cluster_goal_offset_m = LaunchConfiguration('cluster_goal_offset_m')
    cluster_goal_update_period_s = LaunchConfiguration('cluster_goal_update_period_s')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_global_map_config',
            default_value=default_camera_map,
            description='World map + camera config YAML for global_localization_node'
        ),
        DeclareLaunchArgument(
            'use_cluster_pipeline',
            default_value=default_use_cluster_pipeline,
            description='Master switch for cluster nodes (default read from camera_global_map.yaml use_cluster_pipeline)'
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
        DeclareLaunchArgument(
            'cluster_goal_min_score',
            default_value='0.0',
            description='Minimum best-cluster score required to send goal'
        ),
        DeclareLaunchArgument(
            'cluster_goal_offset_m',
            default_value='0.18',
            description='Offset before cluster center to avoid driving into block cluster'
        ),
        DeclareLaunchArgument(
            'cluster_goal_update_period_s',
            default_value='0.7',
            description='Minimum interval between goal updates to Nav2'
        ),

        # Overhead camera: detects ArUco markers, publishes /camera/global_pose (map frame).
        Node(
            package='camera_localization',
            executable='global_localization_node',
            name='global_localization_node',                                                                                                                                  
            output='screen',
            parameters=[camera_map_config],
        ),

        # Cluster analysis node: consumes /detected_blocks and publishes /cluster_info
        Node(
            package='pfe',
            executable='cluster_analyze_node',
            name='cluster_analyze_node',
            output='screen',
            condition=IfCondition(use_cluster_pipeline),
            parameters=[{
                'team_color': cluster_team_color,
                'show_debug_window': cluster_show_debug_window,
            }],
        ),

        # Bridge best cluster -> Nav2 navigate_to_pose goal
        Node(
            package='bringup',
            executable='cluster_goal_bridge_node',
            name='cluster_goal_bridge_node',
            output='screen',
            condition=IfCondition(use_cluster_pipeline),
            parameters=[{
                'cluster_topic': '/cluster_info',
                'action_name': 'navigate_to_pose',
                'goal_frame': 'map',
                'enabled': True,
                'min_score': cluster_goal_min_score,
                'approach_offset_m': cluster_goal_offset_m,
                'min_goal_update_period_s': cluster_goal_update_period_s,
            }],
        ),
    ])
