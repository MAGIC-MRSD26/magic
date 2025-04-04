import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/camera/color/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera/color/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="camera_color_optical_frame",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)

        # Publisher for the camera feed with markers
        self.image_with_markers_pub = self.create_publisher(
            Image, "camera/image_with_markers", 10
        )

        # Publisher for the left and right grasp points
        self.left_grasp_pub = self.create_publisher(Pose, "/left_grasp_point", 10)
        self.right_grasp_pub = self.create_publisher(Pose, "/right_grasp_point", 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
        )
        if marker_ids is not None:
            if cv2.__version__ > "4.0.0":
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )

            for i, marker_id in enumerate(marker_ids):
                # Draw marker borders
                cv2.aruco.drawDetectedMarkers(cv_image, corners, marker_ids)

                # Draw pose axes
                cv2.drawFrameAxes(
                    cv_image, self.intrinsic_mat, self.distortion, rvecs[i], tvecs[i], 0.03
                )

                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

                # Left grasp point calculation (-20 cm in X, +25 cm in Z)
                left_grasp_offset = np.array([-0.20, 0, 0.25])  # 20 cm in X, 25 cm in Z (in marker's local frame)
                rot_matrix_left = np.eye(3)
                rot_matrix_left[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]  # Marker rotation matrix
                left_grasp_offset_rotated = np.dot(rot_matrix_left, left_grasp_offset)  # Apply the rotation

                # Update the left grasp position in the camera frame using the rotated offset
                left_grasp_tvec = tvecs[i][0] + left_grasp_offset_rotated  # Add the offset to the marker's position

                # Right grasp point calculation (+20 cm in X, +25 cm in Z)
                right_grasp_offset = np.array([0.20, 0, 0.25])  # 20 cm in X, 25 cm in Z (in marker's local frame)
                rot_matrix_right = np.eye(3)
                rot_matrix_right[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]  # Marker rotation matrix
                right_grasp_offset_rotated = np.dot(rot_matrix_right, right_grasp_offset)  # Apply the rotation

                # Update the right grasp position in the camera frame using the rotated offset
                right_grasp_tvec = tvecs[i][0] + right_grasp_offset_rotated  # Add the offset to the marker's position

                # Create Pose message for the left grasp point
                left_grasp = Pose()
                left_grasp.position.x = left_grasp_tvec[0]
                left_grasp.position.y = left_grasp_tvec[1]
                left_grasp.position.z = left_grasp_tvec[2]

                # Use the same rotation (rvec) as the ArUco marker for the left grasp
                rot_matrix_left = np.eye(4)
                rot_matrix_left[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat_left = tf_transformations.quaternion_from_matrix(rot_matrix_left)

                left_grasp.orientation.x = quat_left[0]
                left_grasp.orientation.y = quat_left[1]
                left_grasp.orientation.z = quat_left[2]
                left_grasp.orientation.w = quat_left[3]

                # Publish the left grasp point
                self.left_grasp_pub.publish(left_grasp)

                # Create Pose message for the right grasp point
                right_grasp = Pose()
                right_grasp.position.x = right_grasp_tvec[0]
                right_grasp.position.y = right_grasp_tvec[1]
                right_grasp.position.z = right_grasp_tvec[2]

                # Use the same rotation (rvec) as the ArUco marker for the right grasp
                rot_matrix_right = np.eye(4)
                rot_matrix_right[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat_right = tf_transformations.quaternion_from_matrix(rot_matrix_right)

                right_grasp.orientation.x = quat_right[0]
                right_grasp.orientation.y = quat_right[1]
                right_grasp.orientation.z = quat_right[2]
                right_grasp.orientation.w = quat_right[3]

                # Publish the right grasp point
                self.right_grasp_pub.publish(right_grasp)

                # Project the left grasp point to image coordinates using the updated left_grasp_tvec
                left_grasp_image = cv2.projectPoints(
                    np.array([[left_grasp.position.x, left_grasp.position.y, left_grasp.position.z]]),
                    rvecs[i], left_grasp_tvec, self.intrinsic_mat, self.distortion
                )[0].reshape(-1, 2).astype(int)

                # Project the right grasp point to image coordinates using the updated right_grasp_tvec
                right_grasp_image = cv2.projectPoints(
                    np.array([[right_grasp.position.x, right_grasp.position.y, right_grasp.position.z]]),
                    rvecs[i], right_grasp_tvec, self.intrinsic_mat, self.distortion
                )[0].reshape(-1, 2).astype(int)

                # Draw the circles on the grasp points (on the image)
                cv2.drawFrameAxes(cv_image, self.intrinsic_mat, self.distortion, rvecs[i], left_grasp_tvec, 0.05)  # Draw frame axes for left grasp
                cv2.drawFrameAxes(cv_image, self.intrinsic_mat, self.distortion, rvecs[i], right_grasp_tvec, 0.05)  # Draw frame axes for right grasp



            # Publish the poses and marker information
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)

        # Convert the image with markers back to a ROS message
        img_with_markers_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        img_with_markers_msg.header.stamp = img_msg.header.stamp
        self.image_with_markers_pub.publish(img_with_markers_msg)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
