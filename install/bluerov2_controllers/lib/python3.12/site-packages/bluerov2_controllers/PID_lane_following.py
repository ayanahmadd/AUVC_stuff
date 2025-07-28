#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from mavros_msgs.msg import ManualControl
from bluerov2_controllers.PIDController import PIDController
from cv_bridge import CvBridge

class StickToClosestLane(Node):
    def __init__(self):
        super().__init__('stick_to_lane')

        # Yaw PID drives slope → 0 (vertical)
        self.yaw_pid = PIDController(kp=2.0, ki=0.1, kd=0.75,
                                     setpoint=0.0, dt=0.1)
        # Lateral PID drives offset → 0 (centered)
        self.lat_pid = PIDController(kp=70.0, ki=2.5, kd=5.0,
                                     setpoint=0.0, dt=0.1)

        # will be set once we see the first image
        self.image_width = None

        # Bridge for image messages
        self.bridge = CvBridge()

        # Subscribe to camera so we know width
        self.create_subscription(
            Image, 'bluerov2/camera', self._image_cb, 10
        )

        # Subscribe to best_lane [slope, angle, x_center]
        self.create_subscription(
            Float64MultiArray,
            '/lane_detector/best_lane',
            self.lane_cb,
            10
        )

        self.pub = self.create_publisher(ManualControl, '/manual_control', 10)

        # Latest sensor inputs
        self._latest_slope = None   # from best_lane[0]
        self._latest_offset = None  # normalized in (–1…1)
        self.forward_speed = .20     # thrust once aligned & centered

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("StickToClosestLane ready. Listening on /camera/image_raw and /lane_detector/best_lane")

    def _image_cb(self, img_msg: Image):
        # Grab real image width for offset normalization
        if self.image_width is None:
            self.image_width = img_msg.width
            self.get_logger().info(f"Got image width: {self.image_width}")

    def lane_cb(self, msg: Float64MultiArray):
        data = msg.data
        if len(data) < 3:
            self.get_logger().warn("best_lane array too short")
            return

        slope, _, x_center = data
        self._latest_slope = float(slope)

        if self.image_width is None:
            self.get_logger().warn("Image width unknown, skipping offset")
            return

        # normalize offset: center at 0, range ±1
        half = self.image_width / 2.0
        self._latest_offset = (x_center - half) / half

    def control_loop(self):
        # need both readings
        if self._latest_offset is None:
            self.get_logger().warn("No offset updates yet")
            return
        if self._latest_slope is None:
            self.get_logger().warn("No slope updates yet")
            return

        # “close‐enough” thresholds
        centered = abs(self._latest_offset) < 0.05
        aligned  = abs(self._latest_slope)  < 0.1

        # Stage 1: … same as you have …
        y_cmd = self.lat_pid.compute(self._latest_offset) if not centered else 0.0
        r_cmd = self.yaw_pid.compute(self._latest_slope)  if centered else 0.0
        x_cmd = self.forward_speed if (centered and aligned) else 0.0

        # clamp everything
        x_cmd = max(min(x_cmd, 1.0), -1.0)
        y_cmd = max(min(y_cmd, 1.0), -1.0)
        r_cmd = max(min(r_cmd, 1.0), -1.0)


        # publish ManualControl
        mc = ManualControl()
        mc.header.stamp = self.get_clock().now().to_msg()
        mc.x = float(x_cmd)
        mc.y = float(y_cmd)
        mc.z = 0.0
        mc.r = float(r_cmd)
        self.pub.publish(mc)

        # logging by stage
        if not centered:
            self.get_logger().debug(f"[Stage 1] Lateral only → y={y_cmd:.2f}")
        elif not aligned:
            self.get_logger().debug(f"[Stage 2] Yaw only → r={r_cmd:.2f}")
        else:
            self.get_logger().info(f"[Stage 3] Forward x={x_cmd:.2f}, Yaw r={r_cmd:.2f}")


    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StickToClosestLane()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
