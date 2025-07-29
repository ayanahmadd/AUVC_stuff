#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from mavros_msgs.msg import ManualControl
from bluerov2_controllers.PIDController import PIDController
from cv_bridge import CvBridge

class FollowHorizontalLane(Node):
    def __init__(self):
        super().__init__('follow_horizontal_lane')

        # Yaw PID drives angle → 0 (perfectly horizontal)
        self.yaw_pid = PIDController(kp=2.0, ki=0.1, kd=0.75,
                                     setpoint=0.0, dt=0.1)
        # Depth PID keeps depth constant (optional, placeholder)
        self.depth_pid = PIDController(kp=1.0, ki=0.0, kd=0.0,
                                       setpoint=0.0, dt=0.1)

        # For computing offset
        self.image_height = None

        self.bridge = CvBridge()

        # Subscribe to camera so we know image height
        self.create_subscription(
            Image, 'bluerov2/camera', self._image_cb, 10
        )

        # Subscribe to best horizontal lane [slope, angle, y_center]
        self.create_subscription(
            Float64MultiArray,
            '/lane_detector/best_horizontal_lane',
            self.lane_cb,
            10
        )

        self.pub = self.create_publisher(ManualControl, '/manual_control', 10)

        # Latest inputs
        self._latest_angle = None
        self._latest_offset = None  # normalized y offset
        self.forward_speed = 0.25

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("FollowHorizontalLane ready.")

    def _image_cb(self, img_msg: Image):
        if self.image_height is None:
            self.image_height = img_msg.height
            self.get_logger().info(f"Got image height: {self.image_height}")

    def lane_cb(self, msg: Float64MultiArray):
        data = msg.data
        if len(data) < 3:
            self.get_logger().warn("best_horizontal_lane array too short")
            return

        _, angle, y_center = data
        self._latest_angle = float(angle)

        if self.image_height is None:
            self.get_logger().warn("Image height unknown, skipping offset")
            return

        half = self.image_height / 2.0
        self._latest_offset = (y_center - half) / half

    def control_loop(self):
        if self._latest_angle is None:
            self.get_logger().warn("No angle update yet")
            return
        if self._latest_offset is None:
            self.get_logger().warn("No y offset update yet")
            return

        # Close-enough thresholds
        centered = abs(self._latest_offset) < 0.05
        aligned = abs(self._latest_angle) < 2.0  # degrees

        z_cmd = self.depth_pid.compute(self._latest_offset) if not centered else 0.0
        r_cmd = self.yaw_pid.compute(self._latest_angle)   if centered else 0.0
        x_cmd = self.forward_speed if (centered and aligned) else 0.0

        x_cmd = max(min(x_cmd, 1.0), -1.0)
        z_cmd = max(min(z_cmd, 1.0), -1.0)
        r_cmd = max(min(r_cmd, 1.0), -1.0)

        mc = ManualControl()
        mc.header.stamp = self.get_clock().now().to_msg()
        mc.x = float(x_cmd)
        mc.y = 0.0
        mc.z = float(z_cmd)
        mc.r = float(r_cmd)
        self.pub.publish(mc)

        if not centered:
            self.get_logger().debug(f"[Stage 1] Adjusting depth z={z_cmd:.2f}")
        elif not aligned:
            self.get_logger().debug(f"[Stage 2] Adjusting yaw r={r_cmd:.2f}")
        else:
            self.get_logger().info(f"[Stage 3] Moving forward x={x_cmd:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = FollowHorizontalLane()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()