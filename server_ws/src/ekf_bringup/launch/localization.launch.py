from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ekf_bringup')

    ekf1_config = os.path.join(pkg_share, 'config', 'ekf1.yaml')
    ekf2_config = os.path.join(pkg_share, 'config', 'ekf2.yaml')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw_deg = LaunchConfiguration('start_yaw_deg')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_x',
            default_value='0.0',
            description='Initial robot X position in table/odom coordinates, in metres'
        ),
        DeclareLaunchArgument(
            'start_y',
            default_value='0.0',
            description='Initial robot Y position in table/odom coordinates, in metres'
        ),
        DeclareLaunchArgument(
            'start_yaw_deg',
            default_value='0.0',
            description='Initial robot heading in table/odom coordinates, in degrees'
        ),
        # Deadwheel odometry node
        Node(
            package='deadwheel_odometry',
            executable='ticks_listener',
            name='ticks_listener',
            output='screen',
            parameters=[{
                'initial_x': start_x,
                'initial_y': start_y,
                'initial_yaw_deg': start_yaw_deg,
            }]
        ),

        # EKF1 : Local filter (odom -> base_link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[ekf1_config],
            remappings=[
                ('/odometry/filtered', '/odometry/local'),
                ('set_pose', '/ekf_local_node/set_pose'),
            ],
        ),
        # EKF2: global filter (map -> odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            parameters=[ekf2_config],
            remappings=[
                ('/odometry/filtered', '/odometry/global'),
                ('set_pose', '/ekf_global_node/set_pose'),
            ],
        ),
        Node(
            package='ekf_bringup',
            executable='initial_pose_bridge',
            name='initial_pose_bridge',
            output='screen',
            parameters=[{
                'input_topic': '/initialpose',
                'set_pose_topic': '/ekf_global_node/set_pose',
                'set_pose_service': '/ekf_global_node/set_pose',
                'map_frame': 'map',
            }],
        )
    ])
