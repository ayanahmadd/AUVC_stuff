import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
import cv2

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.img_cb, 10)
        self.pub = self.create_publisher(Float32, '/lane_angle', 10)
        self.bridge = CvBridge()

        self.get_logger().info("Node initialized properly!")

    def img_cb(self, msg):
        self.get_logger().info("Ladies and gents we have entered the callback method.")

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = img.shape[:2]

        blurred = cv2.blur(img, (23, 23))
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, 0), (180, 149, 110))
        roi = mask[h//2:, :]

        edges = cv2.Canny(roi, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 60, minLineLength=50, maxLineGap=30)

        if lines is None:
            self.get_logger().info("Wow it really ran through and returned none...")
            return

        angles = []
        for x1, y1, x2, y2 in lines[:, 0]:
            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            if abs(angle) < 20:
                angles.append(angle)

        if not angles:
            return

        avg_angle = float(np.mean(angles))
        msg_out = Float32()
        msg_out.data = avg_angle
        self.pub.publish(msg_out)
        self.get_logger().info("Yep the node is properly publishing the message!")


def main():
    rclpy.init()
    node = LaneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()