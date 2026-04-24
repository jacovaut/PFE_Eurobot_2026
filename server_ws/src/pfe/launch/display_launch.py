from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('pfe')
    urdf_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    available_packages = get_packages_with_prefixes()

    # Use GUI publisher when available, otherwise fallback to headless publisher.
    jsp_pkg = 'joint_state_publisher_gui'
    jsp_exec = 'joint_state_publisher_gui'
    if jsp_pkg not in available_packages:
        jsp_pkg = 'joint_state_publisher'
        jsp_exec = 'joint_state_publisher'

    robot_description = ParameterValue(
        Command(['ros2 run xacro xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package=jsp_pkg,
            executable=jsp_exec,
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])