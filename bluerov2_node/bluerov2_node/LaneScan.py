#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import numpy as np
import cv2

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
        # subscribe to raw camera images
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.img_cb, 10
        )
        # publisher for average lane angle (degrees)
        self.pub_angle = self.create_publisher(Float64, '/lane_angle', 10)
        # publisher for horizontal offset from center (–1.0 … +1.0)
        self.pub_offset = self.create_publisher(Float64, '/lane_offset', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Node initialized properly!")

    def img_cb(self, msg: Image):
        self.get_logger().debug("Entered img_cb")

        # convert ROS Image → OpenCV BGR
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = img.shape[:2]

        # pre‑processing
        blurred = cv2.blur(img, (23, 23))
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask    = cv2.inRange(hsv, (0, 0, 0), (180, 149, 110))
        roi     = mask[h//2:, :]

        # edge & line detection
        edges = cv2.Canny(roi, 50, 150)
        lines = cv2.HoughLinesP(
            edges, 1, np.pi/180, 60,
            minLineLength=50, maxLineGap=30
        )
        if lines is None:
            self.get_logger().debug("No lines found")
            return

        angles = []
        x_mids  = []
        for x1, y1, x2, y2 in lines[:, 0]:
            # compute line angle
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            # only keep near‑vertical-ish lines
            if abs(angle) < 20:
                angles.append(angle)
                # midpoint x (add back the ROI’s vertical offset)
                mid_x = (x1 + x2) * 0.5
                x_mids.append(mid_x)

        if not angles or not x_mids:
            self.get_logger().debug("No valid lane segments after filtering")
            return

        # 1) average angle
        avg_angle = float(np.mean(angles))
        angle_msg = Float64(data=avg_angle)
        self.pub_angle.publish(angle_msg)

        # 2) average horizontal midpoint → offset
        avg_mid_x = float(np.mean(x_mids))
        # offset from image center, normalized to [–1..+1]
        offset_pixel = (avg_mid_x - (w * 0.5))
        offset_norm  = offset_pixel / (w * 0.5)
        offset_msg   = Float64(data=offset_norm)
        self.pub_offset.publish(offset_msg)

        self.get_logger().info(
            f"Published angle={avg_angle:.1f}°, offset={offset_norm:.2f}"
        )


def main():
    rclpy.init()
    node = LaneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
