#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from dt_apriltags import Detector
import math

TAG_SIZE_METERS = 0.092  # actual tag side length
CAMERA_PARAMS = (273.25, 261.76, 307.89, 153.84)

def rotation_matrix_to_euler(R):
    sy = math.sqrt(R[0,0] ** 2 + R[1,0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return (math.degrees(x), math.degrees(y), math.degrees(z))

class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector_node')
        self.bridge = CvBridge()

        # AprilTag detector
        self.detector = Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.create_subscription(
            Image,
            '/bluerov2/camera',
            self.image_cb,
            10
        )

        self.pub_detection = self.create_publisher(
            Float64MultiArray,
            'apriltag/detection',
            10
        )

        self.get_logger().info("AprilTagDetectorNode started (headless)")

    def image_cb(self, msg: Image):
        # Convert to OpenCV grayscale
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE_METERS
        )

        if not tags:
            self.get_logger().info("No AprilTags detected.")
            return

        tag = tags[0]  # Use only the first detection

        tid = float(tag.tag_id)
        tvec = tag.pose_t.ravel()  # [x, y, z]
        dist = float(np.linalg.norm(tvec))
        euler = rotation_matrix_to_euler(tag.pose_R)

        # Publish tag ID, x, y, z, roll, pitch, yaw
        data = Float64MultiArray()
        data.data = [tid, tvec[0], tvec[1], tvec[2], euler[0], euler[1], euler[2]]
        self.pub_detection.publish(data)

        # Log debug info
        self.get_logger().info(
            f"[ID: {int(tid)}] Pos: ({tvec[0]:.2f}, {tvec[1]:.2f}, {tvec[2]:.2f}) m | "
            f"Distance: {dist:.2f} m | "
            f"Orientation (RPY): ({euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f})°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()