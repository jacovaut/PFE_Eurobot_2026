from setuptools import find_packages, setup

package_name = 'scan_filter'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/scan_filter_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PFE Eurobot',
    maintainer_email='todo@todo.com',
    description='Filters lidar PointCloud2 to map boundaries.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_boundary_filter = scan_filter.scan_boundary_filter_node:main',
        ],
    },
)
