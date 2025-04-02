#!/home/jarvis/miniconda3/envs/magic/bin/python
#need to change the shebang to the path of the python3 interpreter in your conda environment
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
import sys
import os
from ament_index_python.packages import get_package_share_directory

sys.path.append('../')
from control.one_arm_pid_control import SingleArmPIDControl

class MuJoCoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        control_package_path=get_package_share_directory('control')
        self.xml_filepath=os.path.join(control_package_path,'kinova_gen3_mujoco/two_arm_vention_table.xml')


        # self.xml_filepath = "../control/kinova_gen3_mujoco/two_arm_table.xml"
        #path to the xml file that describes the robot and the environment
        # This is the main node that runs the MuJoCo simulation and controls the robot using PID control

        self.model = mujoco.MjModel.from_xml_path(self.xml_filepath)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 0.001
        
        self.controller = SingleArmPIDControl(self.model, self.data)
        
        self.joint_positions = {f'joint_{i}': 0.0 for i in range(1, 8)}
        self.publisher = self.create_publisher(String, 'mujoco_state', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        self.get_logger().info(f"Received Joint States:")
        
        for i, name in enumerate(msg.name):
            if name in self.joint_positions:
                self.joint_positions[name] = msg.position[i] if i < len(msg.position) else None
                self.get_logger().info(f"  {name}: pos={self.joint_positions[name]}")
        
        # Update controller with new joint positions
        arm1_angles = [self.joint_positions[f'joint_{i}'] for i in range(1, 8)]
        self.controller.set_arm_target(1, arm1_angles)
        self.controller.set_arm_target(2, arm1_angles)

    def initialize_robot(self):
        self.controller.go_to_home(self.model, self.data)
        
        # Set specific arm positions
        arm1_angles = [0, np.pi/4, np.pi/2, np.pi/2, 0, 0, 0]  # 7 joint angles
        self.controller.set_arm_target(1, arm1_angles)
        self.controller.set_arm_target(2, arm1_angles)

        # Control grippers
        self.controller.close_gripper(1)  # close first gripper
        self.controller.close_gripper(2)
        self.controller.go_to_desired(self.model, self.data)

    def run(self):
        self.initialize_robot()
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while rclpy.ok():
                mujoco.mj_step(self.model, self.data)
                state_msg = String()
                state_msg.data = f"qpos: {self.data.qpos}, qvel: {self.data.qvel}"
                self.publisher.publish(state_msg)

                viewer.sync()
                rclpy.spin_once(self, timeout_sec=0.01)
                
                # Apply control
                self.controller.go_to_desired(self.model, self.data)

def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()