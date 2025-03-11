# Multi Agent Geometric Inspection and Classification (MAGIC)

## Overview
MAGIC is an system designed for automated geometric inspection and classification using 2 robot arms. The project is part of the Master of Robot Systems Development (MRSD'26) curriculum at Carnegie Mellon University. 

## Features
- Multi-agent coordination and control
- 3D Reconstruction
- Object Pose Estimation

## System Requirements
- Python 3.8+
- ROS2 Humble
- MuJoCo

## remarks:
To run this node properly, make the followingg changes to your code:
- the interpreter path in shebang for the ros nodes needs to be updated with your path
- change the interpreter path in setup.py as required

To run the node follow the steps:
- Run the kinova moveit launch file:
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py   robot_ip:=yyy.yyy.yyy.yyy   use_fake_hardware:=true

- build the packages using colcon build
- Run the ros node to launch the mujoco:
ros2 run mujoco_ros2 mujoco_node

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
