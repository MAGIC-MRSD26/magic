# Multi Agent Geometric Inspection and Classification (MAGIC)

## Overview
MAGIC is an system designed for automated geometric inspection and classification using 2 robot arms. The project is part of the Master of Robot Systems Development (MRSD'26) curriculum at Carnegie Mellon University. 

## Features
- Multi-agent coordination and control
- 3D Reconstruction
- Object Pose Estimation

## System Requirements
- Python 3.8+
- Ubuntu 22.04
- ROS2 Humble
- Moveit2
- Gazebo Ignition

## Setup

```
git clone https://github.com/MAGIC-MRSD26/magic.git
source /opt/ros/humble/setup.bash
cd magic/ros2_ws
```

Build the packages except the ones needed for Gazebo sim
```
colcon build --packages-skip-regex "(.*gz.*|.*ign.*|.*gazebo.*)"
source install/setup.bash
```

## Launch Moveit using Fake Hardware
```
ros2 launch two_arm_moveit2_config robot.launch.py   robot_ip:=yyy.yyy.yyy.yyy   use_fake_hardware:=true
```

## Launch Moveit using Real Hardware

Use this doc to setup Kinova arms
```
ros2 launch two_arm_moveit2_config robot.launch.py   robot_ip:=yyy.yyy.yyy.yyy  
```

## Launch Moveit with Gazebo Ignition

Install simulator and other simulation packages
```
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress
```
```
vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos
```

Now you need to build all 75 packages
```
colcon build
source install/setup.bash
```
1. Launch gazebo first
```
ros2 launch two_arm_moveit2_config gazebo_sim.launch.py
```

2. Launch Rviz/Moveit. It will read the joint states from gazebo.
```
ros2 launch two_arm_moveit2_config moveit_gaz_control.py
```

## Run the FSM to run motion policy
```
ros2 run moveit_go motion_planning
```


## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
