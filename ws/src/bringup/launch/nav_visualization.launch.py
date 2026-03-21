import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')

    default_camera_map = os.path.join(pkg_share, 'config', 'camera_global_map.yaml')
    default_rviz_config = os.path.join(pkg_share, 'config', 'camera_global_map.rviz')

    camera_global_map_config = LaunchConfiguration('camera_global_map_config')
    use_camera_global_localization = LaunchConfiguration('use_camera_global_localization')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    publish_block_obstacles = LaunchConfiguration('publish_block_obstacles')
    use_cluster_analyze = LaunchConfiguration('use_cluster_analyze')
    cluster_team_color = LaunchConfiguration('cluster_team_color')
    cluster_show_debug_window = LaunchConfiguration('cluster_show_debug_window')
    use_cluster_goal_bridge = LaunchConfiguration('use_cluster_goal_bridge')
    cluster_goal_min_score = LaunchConfiguration('cluster_goal_min_score')
    cluster_goal_offset_m = LaunchConfiguration('cluster_goal_offset_m')
    cluster_goal_update_period_s = LaunchConfiguration('cluster_goal_update_period_s')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_global_map_config',
            default_value=default_camera_map,
            description='World map + camera config YAML for global_localization_node and visualizer'
        ),
        DeclareLaunchArgument(
            'use_camera_global_localization',
            default_value='true',
            description='Launch overhead camera global localization node'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz with map visualization preset'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='RViz config file path'
        ),
        DeclareLaunchArgument(
            'publish_block_obstacles',
            default_value='true',
            description='Publish detected blocks as PointCloud2 obstacle points'
        ),
        DeclareLaunchArgument(
            'use_cluster_analyze',
            default_value='true',
            description='Launch strategy cluster analysis node'
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
            'use_cluster_goal_bridge',
            default_value='true',
            description='Send Nav2 goals from /cluster_info best cluster'
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

        Node(
            package='camera_localization',
            executable='global_localization_node',
            name='global_localization_node',
            output='screen',
            condition=IfCondition(use_camera_global_localization),
            parameters=[camera_global_map_config],
        ),

        Node(
            package='bringup',
            executable='camera_map_visualizer_node',
            name='camera_map_visualizer_node',
            output='screen',
            parameters=[camera_global_map_config, {
                'publish_block_obstacles': publish_block_obstacles,
                'map_frame': 'map',
                'robot_pose_topic': '/camera/global_pose',
                'detected_blocks_topic': '/detected_blocks',
                'marker_topic': '/camera_map/markers',
                'block_pointcloud_topic': '/camera/block_obstacles',
            }],
        ),

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

        Node(
            package='bringup',
            executable='cluster_goal_bridge_node',
            name='cluster_goal_bridge_node',
            output='screen',
            condition=IfCondition(use_cluster_goal_bridge),
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

        Node(
            package='rviz2',
            executable='rviz2',
            name='camera_map_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
        ),
    ])
