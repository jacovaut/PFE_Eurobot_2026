from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'pfe'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # Install urdf files
        (os.path.join('share', package_name, 'urdf'),
            glob('description/*.urdf*')),

        # Install meshes
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*.stl')),

        # Install camera calibration files
        (os.path.join('share', package_name, 'camera_calibration'),
            glob('camera_calibration/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pfe',
    maintainer_email='pfe@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csi_camera_publisher_node = ros2_opencv.csi_camera_publisher:main',
            'local_camera_perception_node = ros2_opencv.local_camera_perception:main',
            'local_pickup_solver_node = ros2_opencv.local_pickup_solver:main',
        ],
    },
)