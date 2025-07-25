#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from mavros_msgs.msg import ManualControl
from bluerov2_controllers import PIDController

class StickToClosestLane(Node):
    def __init__(self):
        super().__init__('stick_to_lane')

        # --- Yaw PID (slope→r) ---
        self.yaw_pid = PIDController(kp=2.0, ki=0.1, kd=0.75, setpoint=0.0, dt=0.1)
        # --- Lateral PID (offset→y) ---
        self.lat_pid = PIDController(kp=70.0, ki=2.5, kd=5, setpoint=0.0, dt=0.1)

        # subscribe to slope error (we want slope → 0 → vertical)
        self.create_subscription(Float32, '/lane_angle', self.slope_cb, 10)
        # subscribe to cross‑track offset (we want offset → 0 → centered)
        self.create_subscription(Float32, '/lane_offset', self.offset_cb, 10)

        self.pub = self.create_publisher(ManualControl, '/manual_control', 10)

        # store last readings
        self._latest_slope = None
        self._latest_offset = None

        # run at 10Hz
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("StickToClosestLane ready.")

    def slope_cb(self, msg: Float32):
        # msg.data is slope error in radians or normalized slope; we treat zero as desired
        self._latest_slope = msg.data

    def offset_cb(self, msg: Float32):
        # msg.data is normalized offset (-1 left ... +1 right), zero = center
        self._latest_offset = msg.data

    def control_loop(self):
        # must have both readings
        if self._latest_offset is None:
            self.get_logger().warn_once("No offset updates yet")
            return
        if self._latest_slope is None:
            self.get_logger().warn_once("No slope updates yet")
            return

        # compute raw PID outputs
        y_cmd = self.lat_pid.compute(self._latest_offset)
        r_cmd = self.yaw_pid.compute(self._latest_slope)

        # clamp into [-1, +1]
        y_cmd = max(min(y_cmd, 1.0), -1.0)
        r_cmd = max(min(r_cmd, 1.0), -1.0)

        # “close enough” thresholds
        centered  = abs(self._latest_offset) < 0.05
        aligned   = abs(self._latest_slope) < 0.1

        # decide what to command
        if not centered:
            # Stage 1: only lateral correction
            x_cmd = 0.0
            y_out = y_cmd
            r_out = 0.0

        elif not aligned:
            # Stage 2: only yaw correction
            x_cmd = 0.0
            y_out = 0.0
            r_out = r_cmd

        else:
            # Stage 3: drive forward AND keep yaw corrections
            x_cmd = self.forward_speed
            y_out = 0.0
            r_out = r_cmd

        # publish ManualControl
        mc = ManualControl()
        mc.header.stamp = self.get_clock().now().to_msg()
        mc.x = float(x_cmd)
        mc.y = float(y_out)
        mc.z = 0.0
        mc.r = float(r_out)
        self.pub.publish(mc)

        # logging
        if not centered:
            self.get_logger().debug(f"[Stage 1] Lateral: y={y_out:.2f}")
        elif not aligned:
            self.get_logger().debug(f"[Stage 2] Yaw: r={r_out:.2f}")
        else:
            self.get_logger().info(f"[Stage 3] Forward x={x_cmd:.2f}, Yaw r={r_out:.2f}")


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
