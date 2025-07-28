#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from dt_apriltags import Detector

TAG_SIZE_METERS = 0.10  # actual tag side length
CAMERA_PARAMS = (942.5, 942.5, 960, 540)  # fx, fy, cx, cy

class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector_node')
        self.bridge = CvBridge()

        # prepare a single Detector instance
        self.detector = Detector(
            families='tag36h11',
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # subscribe to raw camera feed on topic "camera"
        self.create_subscription(
            Image,
            'camera',
            self.image_cb,
            10
        )

        # publishes annotated image with bounding‐boxes
        self.pub_image = self.create_publisher(
            Image,
            'apriltag/image',
            10
        )
        # publishes flat array [id, tx, ty, tz, ...]
        self.pub_detections = self.create_publisher(
            Float64MultiArray,
            'apriltag/detections',
            10
        )

        self.get_logger().info("AprilTagDetectorNode started, subscribing to 'camera'")

    def image_cb(self, msg: Image):
        # convert ROS Image → OpenCV BGR
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # detect tags
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE_METERS
        )

        # build detection array
        data = []
        for tag in tags:
            tid = float(tag.tag_id)
            tvec = tag.pose_t.ravel()  # [x, y, z]
            dist = float(np.linalg.norm(tvec))
            data += [tid, tvec[0], tvec[1], tvec[2]]

            # draw box
            corners = tag.corners.astype(int)
            for i in range(4):
                p1 = tuple(corners[i - 1])
                p2 = tuple(corners[i])
                cv2.line(img, p1, p2, (0, 255, 0), 2)

            # draw ID and distance
            c = tag.center.astype(int)
            cv2.putText(img, f"ID:{int(tag.tag_id)}",
                        (c[0]-10, c[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 2)
            cv2.putText(img, f"{dist:.2f}m",
                        (c[0]-10, c[1]+15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)

        # publish detections
        out = Float64MultiArray()
        out.data = data
        self.pub_detections.publish(out)

        # publish annotated image
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header = msg.header
        self.pub_image.publish(img_msg)

        self.get_logger().info(f"Detected {len(tags)} tags")

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
