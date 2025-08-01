#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter

# === Camera intrinsics
FX, FY = 273.25, 261.76
CX, CY = 307.89, 153.84

# === ROV dimensions (meters)
FRONT_WIDTH = 0.33805
FRONT_HEIGHT = 0.251
SIDE_WIDTH = 0.4572
SIDE_HEIGHT = 0.251

CONF_THRESHOLD = 0.5

class ROVPoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('rov_pose_estimator_node')
        self.bridge = CvBridge()

        # Load quantized YOLOv8 TFLite model
        self.interpreter = Interpreter(model_path='best_int8.tflite', num_threads=4)
        self.interpreter.allocate_tensors()
        self.input_index = self.interpreter.get_input_details()[0]['index']
        self.output_index = self.interpreter.get_output_details()[0]['index']
        self.input_shape = self.interpreter.get_input_details()[0]['shape']

        self.create_subscription(Image, 'camera', self.image_cb, 10)
        self.pub_image = self.create_publisher(Image, 'rov_detector/image', 10)
        self.pub_pose = self.create_publisher(Float64MultiArray, 'rov_detector/pose_data', 10)

        self.get_logger().info("ROVPoseEstimatorNode started with TFLite model")

    def image_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = img.shape
        input_img = self.preprocess(img)

        # Run inference
        self.interpreter.set_tensor(self.input_index, input_img)
        self.interpreter.invoke()
        output = self.interpreter.get_tensor(self.output_index)[0]  # (5, 2100)

        # Parse detections
        boxes = self.parse_detections(output, w, h)
        best_box = self.select_optimal_detection(boxes)
        annotated_img = img.copy()
        pose_data = []

        if best_box:
            x, y, w, h = best_box
            pose = self.estimate_3d_pose((x, y, w, h), img.shape[:2])
            if pose:
                x_m, y_m, z_m, angle_deg = pose
                nav = self.compute_relative_target({'x': x_m, 'y': y_m, 'z': z_m})

                # Draw box
                cv2.rectangle(annotated_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                label = f"X={x_m:.2f}m Y={y_m:.2f}m Z={z_m:.2f}m | {angle_deg:.1f}°"
                cv2.putText(annotated_img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

                # Pack pose data
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

    def preprocess(self, img):
        resized = cv2.resize(img, (self.input_shape[2], self.input_shape[1]))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        input_data = np.expand_dims(rgb, axis=0).astype(np.uint8)
        return input_data

    def parse_detections(self, output, img_w, img_h):
        # YOLOv8's TFLite output format is usually (5, N) → [cls, conf, x, y, w, h]
        results = []
        for det in output.T:
            cls, conf, x, y, w, h = det
            if conf < CONF_THRESHOLD:
                continue
            cx, cy = int(x * img_w), int(y * img_h)
            bw, bh = int(w * img_w), int(h * img_h)
            x1 = max(0, cx - bw // 2)
            y1 = max(0, cy - bh // 2)
            results.append((x1, y1, bw, bh))
        return results

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
        for (x, y, w, h) in boxes:
            area = w * h
            if area > max_area and area >= min_area:
                best = (x, y, w, h)
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