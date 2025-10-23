from setuptools import find_packages
from setuptools import setup

setup(
    name='controller_manager',
    version='2.49.0',
    packages=find_packages(
        include=('controller_manager', 'controller_manager.*')),
)
