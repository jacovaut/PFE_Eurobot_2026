from launch.exit_handler import ignore_exit_handler, restart_exit_handler
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    package = 'ydlidar_ros2_driver'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='ydlidar_ros2_driver_node')],
        name='ydlidar_ros2_driver_node',
        exit_handler=restart_exit_handler,
    )
    return ld
