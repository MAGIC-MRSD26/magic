#!/home/jarvis/miniconda3/envs/magic/bin/python python -i

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time
import sys

class MuJoCoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        self.model = mujoco.MjModel.from_xml_path("/home/jarvis/codebase/magic_capstone/magic/magic_manip/kinova_gen3_mujoco/two_arm_table.xml")  # Update with actual path
        self.data = mujoco.MjData(self.model)
        self.joint_positions = {f'joint_{i}': 0.0 for i in range(1, 8)}
        self.publisher = self.create_publisher(String, 'mujoco_state', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/JointStates',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        self.get_logger().info(f"Received Joint States:")
        
        for i, name in enumerate(msg.name):
            if name in self.joint_positions:
                self.joint_positions[name] = msg.position[i] if i < len(msg.position) else None
                self.get_logger().info(f"  {name}: pos={self.joint_positions[name]}")
        
        for key, value in self.joint_positions.items():
            self.data.ctrl[int(key.split('_')[-1])] = value
        print(self.data.ctrl)

    def run(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while rclpy.ok():
                mujoco.mj_step(self.model, self.data)
                state_msg = String()
                state_msg.data = f"qpos: {self.data.qpos}, qvel: {self.data.qvel}"
                self.publisher.publish(state_msg)

                viewer.sync()
                rclpy.spin_once(self, timeout_sec=0.01) 

def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

