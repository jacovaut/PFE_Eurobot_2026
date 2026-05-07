import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'manip_action_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacov',
    maintainer_email='jacovaut@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dispense_action = manip_action_node.dispense_action:main',
            'pick_action = manip_action_node.pick_action:main',
            'pami_keyboard = manip_action_node.pami_keyboard:main',
            'therm_action = manip_action_node.therm_action:main',
        ],
    },
)
