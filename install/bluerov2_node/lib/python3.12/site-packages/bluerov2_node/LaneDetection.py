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

        # Subscribe to camera image
        self.subscription = self.create_subscription(
            Image,
            'bluerov2/camera',  # Adjust topic as needed
            self.image_callback,
            10)

        # Publisher for detected lane info [slope, angle, x_center]
        self.lane_pub = self.create_publisher(Float64MultiArray, '/lane_detector/best_lane', 10)

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_center_x = img.shape[1] // 2

        # Blur and HSV mask
        blurred = cv2.blur(img, (23, 23))
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([0, 0, 0])
        upper_bound = np.array([180, 149, 110])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Canny and Hough Transform
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        lane_candidates = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1:
                    continue  # skip vertical lines
                slope = (y2 - y1) / (x2 - x1)
                angle = math.degrees(math.atan(slope))
                x_center = (x1 + x2) // 2
                lane_candidates.append({"slope": slope, "angle": angle, "x_center": x_center})

        if lane_candidates:
            best_lane = min(lane_candidates, key=lambda l: abs(l["x_center"] - img_center_x))
            # Publish slope, angle, x_center as Float64MultiArray
            msg_out = Float64MultiArray()
            msg_out.data = [
                float(best_lane['slope']),
                float(best_lane['angle']),
                float(best_lane['x_center'])
            ]
            self.lane_pub.publish(msg_out)
        else:
            self.get_logger().info('No valid lane detected.')

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()