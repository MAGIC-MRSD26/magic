import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_commander.robot_trajectory import RobotTrajectory
from moveit_commander.conversions import pose_to_list

class KinovaMoveItNode(Node):
    def __init__(self):
        super().__init__('kinova_moveit_node')
        
        # Initialize MoveIt2 components
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.move_group = MoveGroupCommander("manipulator")  # group name
        
        self.timer = self.create_timer(1.0, self.print_current_pose)
        
        # Send a target pose
        self.send_target_pose()

    def send_target_pose(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "end_effector_link"  # Adjust the frame
        target_pose.pose.position.x = 0.4
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.w = 1.0  # Neutral orientation
        
        self.move_group.set_pose_target(target_pose.pose)
        success = self.move_group.go(wait=True)
        
        if success:
            self.get_logger().info("Target pose reached successfully.")
        else:
            self.get_logger().warn("Failed to reach target pose.")
        
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def print_current_pose(self):
        current_pose = self.move_group.get_current_pose().pose
        self.get_logger().info(f"Current Pose: {current_pose}")


def main(args=None):
    rclpy.init(args=args)
    node = KinovaMoveItNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()