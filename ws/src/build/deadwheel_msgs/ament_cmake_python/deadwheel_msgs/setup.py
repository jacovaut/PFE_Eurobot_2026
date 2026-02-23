from setuptools import find_packages
from setuptools import setup

setup(
    name='deadwheel_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('deadwheel_msgs', 'deadwheel_msgs.*')),
)
