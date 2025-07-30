#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


# === Camera intrinsics
FX, FY = 273.25, 261.76
CX, CY = 307.89, 153.84


# === ROV dimensions (meters)
FRONT_WIDTH = 0.33805
FRONT_HEIGHT = 0.251
SIDE_WIDTH = 0.4572
SIDE_HEIGHT = 0.251


class ROVPoseEstimatorNode(Node):
   def __init__(self):
       super().__init__('rov_pose_estimator_node')
       self.bridge = CvBridge()
       self.model = YOLO("bluerov2_bluecv/bluerov2_bluecv/best.pt")


       self.create_subscription(Image, 'camera', self.image_cb, 10)


       self.pub_image = self.create_publisher(Image, 'rov_detector/image', 10)
       self.pub_pose = self.create_publisher(Float64MultiArray, 'rov_detector/pose_data', 10)


       self.get_logger().info("ROVPoseEstimatorNode started")


   def image_cb(self, msg: Image):
       img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
       results = self.model.predict(img, conf=0.5)[0]
       detections = results.boxes


       best_box = self.select_optimal_detection(detections)
       annotated_img = img.copy()
       pose_data = []


       if best_box:
           x, y, w, h = best_box
           pose = self.estimate_3d_pose((x, y, w, h), img.shape[:2])
           if pose:
               x_m, y_m, z_m, angle_deg = pose
               nav = self.compute_relative_target({'x': x_m, 'y': y_m, 'z': z_m})


               # Draw
               cv2.rectangle(annotated_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
               label = f"X={x_m:.2f}m Y={y_m:.2f}m Z={z_m:.2f}m | {angle_deg:.1f}°"
               cv2.putText(annotated_img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)


               # Pack data
               pose_data = [
                   round(x_m, 4), round(y_m, 4), round(z_m, 4), round(angle_deg, 1),
                   nav['heading'], nav['distance'], nav['target_depth']
               ]
               self.get_logger().info(
                   f"Pose → X={x_m:.2f}m, Y={y_m:.2f}m, Z={z_m:.2f}m, Angle={angle_deg:.1f}° | " +
                   f"Heading={nav['heading']:.1f}°, Forward={nav['distance']:.2f}m, Depth={nav['target_depth']:.2f}m"
               )
           else:
               self.get_logger().warn("Invalid box for pose estimation")
       else:
           self.get_logger().warn("No valid ROV detection")


       # Publish annotated image
       img_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
       img_msg.header = msg.header
       self.pub_image.publish(img_msg)


       # Publish pose data
       if pose_data:
           pose_msg = Float64MultiArray()
           pose_msg.data = pose_data
           self.pub_pose.publish(pose_msg)


   def estimate_3d_pose(self, box, image_shape, fx=FX, fy=FY, cx=CX, cy=CY):
       x, y, w, h = box
       if w <= 0 or h <= 0:
           return None


       z_front = (FRONT_WIDTH * fx) / w
       z_side = (SIDE_WIDTH * fx) / w


       if z_front > z_side:
           real_width = FRONT_WIDTH
           real_height = FRONT_HEIGHT
           angle_deg = 90.0
       else:
           real_width = SIDE_WIDTH
           real_height = SIDE_HEIGHT
           angle_deg = 0.0


       z_from_width = (real_width * fx) / w
       z_from_height = (real_height * fy) / h
       z_m = (z_from_width + z_from_height) / 2


       center_x = x + w / 2
       center_y = y + h / 2
       dx = center_x - cx
       dy = center_y - cy


       x_m = (dx * z_m) / fx
       y_m = -(dy * z_m) / fy


       return round(x_m, 4), round(y_m, 4), round(z_m, 4), angle_deg


   def compute_relative_target(self, pose):
       x, y, z = pose['x'], pose['y'], pose['z']
       heading_deg = np.degrees(np.arctan2(x, z))
       distance = np.sqrt(x**2 + z**2)
       return {
           'heading': round(heading_deg, 4),
           'distance': round(distance, 4),
           'target_depth': round(y, 4)
       }


   def select_optimal_detection(self, boxes, min_area=500):
       best = None
       max_area = 0
       for box in boxes:
           x1, y1, x2, y2 = map(int, box.xyxy[0])
           w, h = x2 - x1, y2 - y1
           area = w * h
           if area > max_area and area >= min_area:
               best = (x1, y1, w, h)
               max_area = area
       return best


def main(args=None):
   rclpy.init(args=args)
   node = ROVPoseEstimatorNode()
   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   finally:
       node.destroy_node()
       rclpy.shutdown()


if __name__ == '__main__':
   main()