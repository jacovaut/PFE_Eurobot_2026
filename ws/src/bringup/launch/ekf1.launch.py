from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')
    config_file = os.path.join(pkg_share, 'config', 'ekf1.yaml')

    return LaunchDescription([
        # Deadwheel odometry node
        Node(
            package='deadwheel_odometry',
            executable='ticks_listener',
            name='ticks_listener',
            output='screen'
        ),

        # EKF
	Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file],
        )
    ])
