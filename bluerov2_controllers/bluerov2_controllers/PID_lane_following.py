#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from mavros_msgs.msg import ManualControl
from bluerov2_controllers import PIDController

class StickToClosestLane(Node):
    def __init__(self):
        super().__init__('stick_to_lane')

        # Yaw PID drives slope → 0 (vertical)
        self.yaw_pid = PIDController(kp=2.0, ki=0.1, kd=0.75,
                                     setpoint=0.0, dt=0.1)
        # Lateral PID drives offset → 0 (centered)
        self.lat_pid = PIDController(kp=70.0, ki=2.5, kd=5.0,
                                     setpoint=0.0, dt=0.1)

        # Subscribe to best_lane [slope, angle, x_center]
        self.create_subscription(
            Float64MultiArray,
            '/lane_detector/best_lane',
            self.lane_cb,
            10
        )

        self.pub = self.create_publisher(ManualControl, '/manual_control', 10)

        self._latest_slope = None   # from best_lane[0]
        self._latest_center = None  # from best_lane[2]
        self.forward_speed = 20    # thrust once aligned & centered

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("StickToClosestLane ready, listening on /lane_detector/best_lane")

    def lane_cb(self, msg: Float64MultiArray):
        data = msg.data
        if len(data) >= 3:
            slope, _, x_center = data
            self._latest_slope = float(slope)
            # compute normalized offset: (x_center - image_center) / (image_width/2)
            # assume image_width known or passed in; here hardcode 640:
            self._latest_offset = (x_center - 640/2) / (640/2)
        else:
            self.get_logger().warn("best_lane array too short")

    def control_loop(self):
        if self._latest_slope is None or self._latest_offset is None:
            return

        # Stage 1: lateral until centered
        centered = abs(self._latest_offset) < 0.05
        aligned  = abs(self._latest_slope) < 0.1

        y_cmd = self.lat_pid.compute(self._latest_offset) if not centered else 0.0
        r_cmd = self.yaw_pid.compute(self._latest_slope)   if centered and not aligned else 0.0
        x_cmd = self.forward_speed if centered and aligned else 0.0

        # clamp
        y_cmd = max(min(y_cmd, 1.0), -1.0)
        r_cmd = max(min(r_cmd, 1.0), -1.0)

        mc = ManualControl()
        mc.header.stamp = self.get_clock().now().to_msg()
        mc.x = float(x_cmd)
        mc.y = float(y_cmd)
        mc.z = 0.0
        mc.r = float(r_cmd)
        self.pub.publish(mc)

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
