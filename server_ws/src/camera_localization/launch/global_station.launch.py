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

def _read_team_color_default(camera_map_path: str) -> str:
    try:
        with open(camera_map_path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        color = data.get('global_localization_node', {}).get('ros__parameters', {}).get('team_color', 'blue')
        return str(color)
    except Exception:
        return 'blue'

def generate_launch_description():
    pkg_share = get_package_share_directory('camera_localization')
    default_camera_map = os.path.join(pkg_share, 'config', 'camera_global_map.yaml')
    default_use_cluster_pipeline = _read_cluster_pipeline_default(default_camera_map)
    default_team_color = _read_team_color_default(default_camera_map)

    camera_map_config = LaunchConfiguration('camera_global_map_config')
    use_cluster_pipeline = LaunchConfiguration('use_cluster_pipeline')
    launch_map_visualizer = LaunchConfiguration('launch_map_visualizer')
    publish_block_obstacles = LaunchConfiguration('publish_block_obstacles')
    team_color = LaunchConfiguration('team_color')
    cluster_robot_marker_id = LaunchConfiguration('cluster_robot_marker_id')
    cluster_show_debug_window = LaunchConfiguration('cluster_show_debug_window')

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
            'launch_map_visualizer',
            default_value='true',
            description='Launch the camera map visualizer for RViz markers and static obstacle cloud'
        ),
        DeclareLaunchArgument(
            'publish_block_obstacles',
            default_value='true',
            description='Publish detected blocks and forbidden zones as PointCloud2 obstacle points'
        ),
        DeclareLaunchArgument(
            'team_color',
            default_value=default_team_color,
            description='Team color for robot IDs and cluster scoring (blue|yellow, bleu|jaune)'
        ),
        DeclareLaunchArgument(
            'cluster_robot_marker_id',
            default_value='1',
            description='Robot ArUco marker id used in detected_blocks for cluster scoring'
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
            output='log',
            parameters=[camera_map_config, {'team_color': team_color}],
        ),

        Node(
            package='camera_localization',
            executable='camera_map_visualizer_node.py',
            name='camera_map_visualizer_node',
            output='log',
            emulate_tty=True,
            prefix='python3 -u -X faulthandler',
            condition=IfCondition(launch_map_visualizer),
            parameters=[camera_map_config, {
                'publish_block_obstacles': publish_block_obstacles,
                'map_frame': 'map',
                'robot_pose_topic': '/camera/global_pose',
                'detected_blocks_topic': '/detected_blocks',
                'marker_topic': '/camera_map/markers',
                'block_pointcloud_topic': '/camera/block_obstacles',
                'team_color': team_color,
            }],
        ),

        # Cluster analysis node: consumes /detected_blocks and publishes /cluster_info
        Node(
            package='strategy',
            executable='cluster_analyze_node',
            name='cluster_analyze_node',
            output='log',
            condition=IfCondition(use_cluster_pipeline),
            parameters=[{
                'team_color': team_color,
                'robot_marker_id': cluster_robot_marker_id,
                'show_debug_window': cluster_show_debug_window,
            }],
        ),

    ])
