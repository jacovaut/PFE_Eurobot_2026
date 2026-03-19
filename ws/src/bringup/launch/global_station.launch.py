from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')
    default_camera_map = os.path.join(pkg_share, 'config', 'camera_global_map.yaml')

    camera_map_config = LaunchConfiguration('camera_global_map_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_global_map_config',
            default_value=default_camera_map,
            description='World map + camera config YAML for global_localization_node'
        ),

        # Overhead camera: detects ArUco markers, publishes /camera/global_pose (map frame).
        # ROS2 DDS automatically routes this topic to the RPi over the network
        # as long as both machines share the same ROS_DOMAIN_ID.
        Node(
            package='camera_localization',
            executable='global_localization_node',
            name='global_localization_node',
            output='screen',
            parameters=[camera_map_config],
        ),
    ])
