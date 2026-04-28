
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('pfe')
    urdf_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro', ' ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }],
            output='screen'
        ),
        Node(
            package='local_camera',
            executable='merged_local_pickup_node',
            name='merged_local_pickup_node',
            output='screen',
            parameters=[{
                'camera_mode': 'stream',
                'stream_url': 'tcp://127.0.0.1:8888',
                'camera_device': 0,
                'show_debug_window': False,
                'debug_save_image': True,
            }],
        ),
    ])
