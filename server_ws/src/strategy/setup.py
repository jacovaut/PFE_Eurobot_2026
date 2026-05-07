from setuptools import find_packages, setup

package_name = 'strategy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pfe',
    maintainer_email='pfe@todo.todo',
    description='Strategy nodes: cluster analysis and match timer',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cluster_analyze_node = strategy.clusterAnalyze:main',
        ],
    },
)
