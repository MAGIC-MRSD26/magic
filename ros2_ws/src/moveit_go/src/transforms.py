#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose
import numpy as np
import time
import os, csv
def pose_to_matrix(p: Pose):
    T = quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    T[0:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T

def matrix_to_pose(T):
    p = Pose()
    p.position.x, p.position.y, p.position.z = T[0:3, 3]
    q = quaternion_from_matrix(T)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p

def average_pose(p1, p2):
    pos = 0.5 * (np.array([p1.position.x, p1.position.y, p1.position.z]) +
                 np.array([p2.position.x, p2.position.y, p2.position.z]))
    q1 = np.array([p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w])
    q2 = np.array([p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w])
    q = q1 + q2
    q /= np.linalg.norm(q)
    p = Pose()
    p.position.x, p.position.y, p.position.z = pos
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p

class DualArmObjectPose(Node):
    def __init__(self):
        super().__init__('dual_arm_object_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.log_file = "object_pose_log.csv"
        self._setup_csv()

        # known offsets at grasp time (tune or calibrate once)
        self.left_T_obj = np.eye(4)
        self.right_T_obj = np.eye(4)
        self.left_T_obj[0,3] = 0.0   # example offset in gripper frame
        self.right_T_obj[0,3] = -0.00

        self.timer = self.create_timer(0.5, self.compute_object_pose)

    def _setup_csv(self):
        """Create CSV with headers if not already present."""
        
        if not os.path.exists(self.log_file):
            with open(self.log_file, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])

    def compute_object_pose(self):
        try:
            tf_L = self.tf_buffer.lookup_transform('world', 'left_end_effector_link', rclpy.time.Time())
            tf_R = self.tf_buffer.lookup_transform('world', 'right_end_effector_link', rclpy.time.Time())

            def tf_to_mat(tf):
                q = tf.transform.rotation
                t = tf.transform.translation
                T = quaternion_matrix([q.x, q.y, q.z, q.w])
                T[0:3, 3] = [t.x, t.y, t.z]
                return T

            T_WL = tf_to_mat(tf_L)
            T_WR = tf_to_mat(tf_R)

            T_WO_L = T_WL @ self.left_T_obj
            T_WO_R = T_WR @ self.right_T_obj

            pose_L = matrix_to_pose(T_WO_L)
            pose_R = matrix_to_pose(T_WO_R)
            pose_avg = average_pose(pose_L, pose_R)

            #writing to csv:
            ts = time.time_ns()
            with open(self.log_file, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    f"{ts:.6f}",
                    pose_avg.position.x,
                    pose_avg.position.y,
                    pose_avg.position.z,
                    pose_avg.orientation.x,
                    pose_avg.orientation.y,
                    pose_avg.orientation.z,
                    pose_avg.orientation.w
                ])

            # extract rotation matrix of averaged pose
            q = [pose_avg.orientation.x, pose_avg.orientation.y,
                pose_avg.orientation.z, pose_avg.orientation.w]
            R = quaternion_matrix(q)[0:3, 0:3]

            self.get_logger().info(
                f"\n[Dual-arm object pose]\n"
                f"  Position (m): [{pose_avg.position.x:.3f}, {pose_avg.position.y:.3f}, {pose_avg.position.z:.3f}]\n"
                f"  Quaternion:   [{pose_avg.orientation.x:.3f}, {pose_avg.orientation.y:.3f}, "
                f"{pose_avg.orientation.z:.3f}, {pose_avg.orientation.w:.3f}]\n"
                f"  Rotation Matrix:\n"
                f"  [{R[0,0]: .3f}, {R[0,1]: .3f}, {R[0,2]: .3f}]\n"
                f"  [{R[1,0]: .3f}, {R[1,1]: .3f}, {R[1,2]: .3f}]\n"
                f"  [{R[2,0]: .3f}, {R[2,1]: .3f}, {R[2,2]: .3f}]\n"
            )

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

def main():
    rclpy.init()
    node = DualArmObjectPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
