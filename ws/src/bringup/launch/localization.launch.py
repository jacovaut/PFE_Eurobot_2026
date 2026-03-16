from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('bringup')

    ekf1_config = os.path.join(pkg_share, 'config', 'ekf1.yaml')
    ekf2_config = os.path.join(pkg_share, 'config', 'ekf2.yaml')

    return LaunchDescription([
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
        )
    ])
