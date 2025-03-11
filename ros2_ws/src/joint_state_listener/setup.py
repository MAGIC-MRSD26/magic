from setuptools import setup

package_name = 'joint_state_listener'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A ROS 2 node that listens to /joint_states and prints values',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_subscriber = joint_state_listener.joint_state_subscriber:main',
        ],
    },
    options={'build_scripts': {'executable': '/home/jarvis/miniconda3/envs/magic/bin/python'}}
)
