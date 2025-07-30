#base imports - just to access functions, pre-existing stuff
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from mavros_msgs.msg import ManualControl
from bluerov2_controllers import PIDController
from bluerov2_opencv import lane_detection_functions
from bluerov2_opencv import lane_following

#same imports as rosmav camera publisher node - just to avoid errors
from sensor_msgs.msg import Image
#import gi
from cv_bridge import CvBridge
import numpy as np

class ImageToCode(Node):
    def __init__(self):
        super().__init__("bluerov2_camera_subscriber")

        self.latest_image = None
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,        # the message type
            "camera",    # the topic name,
            self.image_callback,  # the subscription's callback method
            10              # QOS (will be covered later)
        )

        # Pub for direction
        self.dir_pub = self.create_publisher(
            String, 
            "/lane_direction", 
            10
        )

        # Pub for slope
        self.slope_pub = self.create_publisher(
            Float64, 
            "/lane_slope", 
            10
        )

        # Timer to process image regularly
        self.timer = self.create_timer(0.1, self.process_lane_info)  # 10Hz

        self.get_logger().info("ImageToCode node initialized.")

    def image_callback(self, msg):
        self.latest_image = msg
    
    def process_lane_info(self):
        if self.latest_image is None:
            return
        
        else:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')

            lines = lane_detection_functions.detect_lines(cv_image)
            
            lanes = lane_detection_functions.detect_lanes(lines)
            
            best_lane = lane_following.get_closest_lane(lanes)      #If best_lane ever needs to be accessed... otherwise has little use
            if best_lane is None:
                self.get_logger().info("Could not compute best lane.")
                return

            center_slope, center_intercept = lane_following.get_lane_center(lanes)  #Automatically filters for the best_lane within the function
            if center_slope is None or center_intercept is None:
                self.get_logger().info("Could not compute lane center.")
                return
            
            wanted_direction = lane_following.recommend_direction(center_intercept, center_slope)
            
            self.get_logger().info(f"Direction: {wanted_direction}, Slope: {center_slope}")
            self.dir_pub.publish(String(data=wanted_direction))
            self.slope_pub.publish(Float64(data=center_slope))


def main(args=None):
    rclpy.init(args=args)
    node = ImageToCode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()