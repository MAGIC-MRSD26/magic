#!/home/jarvis/miniconda3/envs/magic/bin/python python -i

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys

class MuJoCoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        self.model = mujoco.MjModel.from_xml_path("/home/jarvis/codebase/magic_capstone/magic/magic_manip/kinova_gen3_mujoco/two_arm_table.xml")  # Update with actual path
        self.data = mujoco.MjData(self.model)

        self.publisher = self.create_publisher(String, 'mujoco_state', 10)
        self.subscription = self.create_subscription(
            String,
            '/mujoco_command',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        # Example: Apply force based on received command
        if msg.data == "move":
            self.data.ctrl[0] += 1.0  # Example control input
            self.data.ctrl[7] += 1.0  # Example control input
            print('Received command to move')
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

