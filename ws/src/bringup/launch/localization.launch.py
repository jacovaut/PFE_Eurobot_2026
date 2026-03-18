from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')

    ekf1_config = os.path.join(pkg_share, 'config', 'ekf1.yaml')
    ekf2_config = os.path.join(pkg_share, 'config', 'ekf2.yaml')
    camera_map_config = os.path.join(pkg_share, 'config', 'camera_global_map.yaml')

    use_camera_global_localization = LaunchConfiguration('use_camera_global_localization')
    camera_global_map_config = LaunchConfiguration('camera_global_map_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera_global_localization',
            default_value='true',
            description='Launch overhead camera global localization node'
        ),
        DeclareLaunchArgument(
            'camera_global_map_config',
            default_value=camera_map_config,
            description='YAML config for global_localization_node world map and camera settings'
        ),

        # Deadwheel odometry node
        Node(
            package='deadwheel_odometry',
            executable='ticks_listener',
            name='ticks_listener',
            output='screen'
        ),

        # EKF1 : Local filter (odom -> base_link)
	Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[ekf1_config],
        ),
 	# EKF2: global filter (map -> odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            parameters=[ekf2_config],
            remappings=[('/odometry/filtered', '/odometry/global')]
        ),

        # Overhead camera: publishes /camera/global_pose in map frame
        Node(
            package='camera_localization',
            executable='global_localization_node',
            name='global_localization_node',
            output='screen',
            parameters=[camera_global_map_config],
            condition=IfCondition(use_camera_global_localization),
        )
    ])
