from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    local_camera_device = LaunchConfiguration('local_camera_device')
    local_camera_path = LaunchConfiguration('local_camera_path')
    local_show_debug_window = LaunchConfiguration('local_show_debug_window')

    return LaunchDescription([
        DeclareLaunchArgument(
            'local_camera_device',
            default_value='0',
            description='Camera index for local_camera_perception_node on this machine'
        ),
        DeclareLaunchArgument(
            'local_camera_path',
            default_value='',
            description='Optional camera path for local_camera_perception_node, e.g. /dev/v4l/by-id/...'
        ),
        DeclareLaunchArgument(
            'local_show_debug_window',
            default_value='false',
            description='Show debug window for local_camera_perception_node'
        ),

        Node(
            package='pfe',
            executable='local_camera_perception_node',
            name='local_camera_perception_node',
            output='screen',
            parameters=[{
                'camera_device': local_camera_device,
                'camera_path': local_camera_path,
                'show_debug_window': local_show_debug_window,
            }],
        ),
    ])
