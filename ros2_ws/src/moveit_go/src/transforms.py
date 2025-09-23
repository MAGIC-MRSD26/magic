#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
import sys, termios, tty

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class BinPoseClient(Node):
    def __init__(self):
        super().__init__('bin_pose_client')
        self.cli = self.create_client(GetPlanningScene, '/get_planning_scene')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /get_planning_scene service...')
        self.get_logger().info("Press 'q' to query the bin pose. Ctrl+C to quit.")

    def query_scene(self):
        req = GetPlanningScene.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result():
            self.get_logger().error("Service call failed")
            return

        scene = future.result().scene

        # 1. Free objects in world
        for obj in scene.world.collision_objects:
            if obj.id == "bin":
                p = obj.pose
                self.get_logger().info(
                    f"World bin pose (frame={obj.header.frame_id}): "
                    f"pos=({p.position.x:.3f}, {p.position.y:.3f}, {p.position.z:.3f}), "
                    f"orient=({p.orientation.x:.3f}, {p.orientation.y:.3f}, "
                    f"{p.orientation.z:.3f}, {p.orientation.w:.3f})"
                )

        # 2. Attached objects
        for aobj in scene.robot_state.attached_collision_objects:
            if aobj.object.id == "bin":
                p = aobj.object.pose
                self.get_logger().info(
                    f"Attached bin pose (frame={aobj.object.header.frame_id}, link={aobj.link_name}): "
                    f"pos=({p.position.x:.3f}, {p.position.y:.3f}, {p.position.z:.3f}), "
                    f"orient=({p.orientation.x:.3f}, {p.orientation.y:.3f}, "
                    f"{p.orientation.z:.3f}, {p.orientation.w:.3f})"
                )

def main(args=None):
    rclpy.init(args=args)
    node = BinPoseClient()
    try:
        while rclpy.ok():
            key = get_key()
            if key.lower() == 'q':
                node.query_scene()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
