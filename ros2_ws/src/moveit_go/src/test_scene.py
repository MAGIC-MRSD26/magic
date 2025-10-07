#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene

class ScenePrinter(Node):
    def __init__(self):
        super().__init__('scene_printer')
        self.cli = self.create_client(GetPlanningScene, '/get_planning_scene')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /get_planning_scene service...')
        self.timer = self.create_timer(2.0, self.query_scene)

    def query_scene(self):
        req = GetPlanningScene.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        scene = future.result().scene

        world = [obj.id for obj in scene.world.collision_objects]
        attached = [a.object.id for a in scene.robot_state.attached_collision_objects]

        self.get_logger().info(f"World objects: {world}")
        self.get_logger().info(f"Attached objects: {attached}")

def main():
    rclpy.init()
    node = ScenePrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
