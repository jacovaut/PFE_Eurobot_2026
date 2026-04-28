from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
