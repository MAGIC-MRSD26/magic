import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

    def joint_states_callback(self, msg):
        self.get_logger().info(f"Received Joint States:")
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            velocity = msg.velocity[i] if i < len(msg.velocity) else None
            effort = msg.effort[i] if i < len(msg.effort) else None
            self.get_logger().info(f"  {name}: pos={position}, vel={velocity}, effort={effort}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
