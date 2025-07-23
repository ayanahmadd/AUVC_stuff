#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from mavros_msgs.msg import ManualControl


class HeadingLockOnly(Node):
    def __init__(self):
        super().__init__('heading_lock_only')

        # ── PID Tuning ──
        self.Kp = 2.0
        self.Ki = 0.05
        self.Kd = 0.75
        self.deadband = 0.5
        self.int_thresh = 15.0
        self.max_d = 35.0
        self.target = 0.0
        self._alpha = 0.7

        # ── State ──
        self.current = None
        self._prev_err = None
        self._i_term = 0.0
        self._d_filt = 0.0

        # ── ROS Setup ──
        self.pub = self.create_publisher(ManualControl, '/manual_control', 10)
        self.create_subscription(Int16, '/heading', self.heading_cb, 10)
        self.create_timer(0.1, self.loop_cb)

        self.get_logger().info("Node started: heading hold only (no forward motion).")

    def heading_cb(self, msg: Int16):
        self.current = float(msg.data)

    def loop_cb(self):
        if self.current is None:
            return

        raw = self.target - self.current
        err = (raw + 180.0) % 360.0 - 180.0

        # P term
        p = self.Kp * err

        # I term
        if abs(err) < self.int_thresh:
            self._i_term += err * self.Ki * 0.1
        i = self._i_term

        # D term
        d = 0.0
        if self._prev_err is not None:
            delta = err - self._prev_err
            if abs(delta) < 180.0:
                raw_d = delta / 0.1
                self._d_filt = self._alpha * self._d_filt + (1 - self._alpha) * raw_d
                d = self.Kd * self._d_filt
                d = max(min(d, self.max_d), -self.max_d)

        self._prev_err = err
        r_cmd = max(min(p + i + d, 700.0), -700.0)

        # Publish yaw control only
        mc = ManualControl()
        mc.header.stamp = self.get_clock().now().to_msg()
        mc.r = float(r_cmd)
        mc.x = mc.y = mc.z = 0.0
        self.pub.publish(mc)

        self.get_logger().info(
            f"[LOCK] Heading={self.current:.1f}°, err={err:.2f} → r_cmd={r_cmd:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HeadingLockOnly()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()