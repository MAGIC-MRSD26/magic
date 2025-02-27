from setuptools import find_packages
from setuptools import setup

setup(
    name='ros_ign_interfaces',
    version='0.244.16',
    packages=find_packages(
        include=('ros_ign_interfaces', 'ros_ign_interfaces.*')),
)
