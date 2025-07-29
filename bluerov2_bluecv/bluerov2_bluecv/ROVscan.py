#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

# Approximate focal length in pixels (depends on your camera + calibration)
FOCAL_LENGTH_PX = 672.0
ROV_REAL_WIDTH_M = 0.33805  # Width of ROV in meters

class ROVDetectorNode(Node):
    def __init__(self):
        super().__init__('rov_detector_node')
        self.bridge = CvBridge()

        # Load YOLOv8 model trained on ROVs
        self.model = YOLO("bluerov2_bluecv/bluerov2_bluecv/best.pt")

        self.create_subscription(
            Image,
            'camera',
            self.image_cb,
            10
        )

        self.pub_image = self.create_publisher(
            Image,
            'rov_detector/image',
            10
        )
        self.pub_detections = self.create_publisher(
            Float64MultiArray,
            'rov_detector/detections',
            10
        )

        self.get_logger().info("ROVDetectorNode started, subscribing to 'camera'")

    def image_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run detection
        results = self.model.predict(img, conf=0.5)[0]
        detections = results.boxes

        annotated_img = img.copy()
        out_data = []

        for box in detections:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            width_px = x2 - x1

            # Estimate distance based on apparent width
            if width_px > 0:
                distance_m = (FOCAL_LENGTH_PX * ROV_REAL_WIDTH_M) / width_px
            else:
                distance_m = -1.0  # invalid

            # Append detection info: [x_center, y_center, width, height, distance]
            out_data += [float(cx), float(cy), float(width_px), float(y2 - y1), float(distance_m)]

            # Annotate image
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(annotated_img, f"{distance_m:.2f}m", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Publish detection info
        out_msg = Float64MultiArray()
        out_msg.data = out_data
        self.pub_detections.publish(out_msg)

        # Publish annotated image
        img_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8')
        img_msg.header = msg.header
        self.pub_image.publish(img_msg)

        self.get_logger().info(f"Detected {len(detections)} ROVs")

def main(args=None):
    rclpy.init(args=args)
    node = ROVDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

