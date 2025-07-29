import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera',
            self.image_callback,
            10
        )

        self.lane_pub = self.create_publisher(
            Float64MultiArray, 
            '/lane_detector/best_lane', 
            10
        )

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h = image_rgb.shape[0]
        cropped = image_rgb[h//2:, :]

        gray = cv2.cvtColor(cropped, cv2.COLOR_RGB2GRAY)
        equalized = cv2.equalizeHist(gray)
        blurred = cv2.GaussianBlur(equalized, (5, 5), 0)

        edges = cv2.Canny(blurred, 50, 150)

        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=80, minLineLength=100, maxLineGap=15)

        best_line = None
        best_score = float('inf')
        img_center_y = img.shape[0] * 3 // 4  # for bottom-half priority

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if abs(y2 - y1) > abs(x2 - x1):  # skip vertical-ish lines
                    continue
                y_center = (y1 + y2) // 2 + h // 2
                score = abs(y_center - img_center_y)
                if score < best_score:
                    best_score = score
                    best_line = (x1, y1 + h // 2, x2, y2 + h // 2)

        if best_line:
            x1, y1, x2, y2 = best_line
            if x2 == x1:
                return  # vertical, ignore

            slope = (y2 - y1) / (x2 - x1)
            angle = math.degrees(math.atan(slope))
            x_center = (x1 + x2) / 2.0

            msg_out = Float64MultiArray()
            msg_out.data = [slope, angle, x_center]
            self.lane_pub.publish(msg_out)

            self.get_logger().info(
                f"Best horizontal lane: slope={slope:.3f}, angle={angle:.2f}, x_center={x_center:.1f}"
            )
        else:
            self.get_logger().info("No valid horizontal lane detected.")

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()