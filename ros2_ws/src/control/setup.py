from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add URDF, XML, and other model files
        (os.path.join('share', package_name, 'kinova_gen3_mujoco'), glob('kinova_gen3_mujoco/*.xml')),
        # Add all STL files in the assets directory
        (os.path.join('share', package_name, 'kinova_gen3_mujoco/assets'), glob('kinova_gen3_mujoco/assets/*.stl')),
        (os.path.join('share', package_name, 'kinova_gen3_mujoco/assets'), glob('kinova_gen3_mujoco/assets/*.STL')),
        # Add any PNG or other image files
        (os.path.join('share', package_name, 'kinova_gen3_mujoco'), glob('kinova_gen3_mujoco/*.png'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Megan Lee',
    maintainer_email='meganlee@andrew.cmu.edu',
    description='Robot control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = control.control:main',
            'kinova_gen3_mujoco = control.kinova_gen3_mujoco:main',
            # Add other Python scripts that have main functions here
        ],
    },
)