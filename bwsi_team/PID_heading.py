#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from mavros_msgs.msg import ManualControl

class SimpleHeadingNode(Node):
    def __init__(self):
        super().__init__('simple_heading_node')

        # ── tuning ──
        self.Kp = 2.00        # proportional gain
        self.Ki = 0.04        # integral gain
        self.Kd = 0.50        # derivative gain
        self.target = 73.0    # desired heading (deg)
        self.deadband = 0.5   # ±0.5° tolerance
        self.int_thresh = 15.0  # only integrate when |err| < 15°
        self.max_d = 35.0     # clamp D term to ±20
        # ────────────

        self.current = None
        self._prev_err = None
        self._i_term = 0.0
        self._d_filt = 0.0
        self._alpha = 0.7     # EMA alpha for derivative

        # Publisher → /manual_control
        self.pub = self.create_publisher(ManualControl, '/manual_control', 10)
        # Subscriber → /heading as Int16
        self.create_subscription(Int16, '/heading', self.heading_cb, 10)
        # 10 Hz control loop
        self.create_timer(0.1, self.loop_cb)

        self.get_logger().info("SimpleHeadingNode ready (PID, ±0.5° deadband)")

    def heading_cb(self, msg: Int16):
        self.current = float(msg.data)

    def loop_cb(self):
        if self.current is None:
            return

        # shortest‑path error ∈ [‑180, +180]
        raw = self.target - self.current
        err = (raw + 180.0) % 360.0 - 180.0

        # deadband stop
        if abs(err) <= self.deadband:
            stop = ManualControl()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.x = stop.y = stop.z = stop.r = 0.0
            self.pub.publish(stop)
            self.get_logger().info(f"Reached {self.current:.1f}° (err={err:.1f}) → stopping.")
            rclpy.shutdown()
            return

        # P term
        p_term = self.Kp * err

        # I term (only when error small)
        if abs(err) < self.int_thresh:
            self._i_term += err * self.Ki * 0.1  # dt=0.1s
        i_term = self._i_term

        # D term with EMA filter & clamp & wrap‑jump ignore
        d_term = 0.0
        if self._prev_err is not None:
            delta = err - self._prev_err
            # ignore derivative across wrap jumps
            if abs(delta) < 180.0:
                raw_d = delta / 0.1
                self._d_filt = self._alpha * self._d_filt + (1 - self._alpha) * raw_d
                d_term = self.Kd * self._d_filt
                # clamp D so it can’t dominate
                d_term = max(min(d_term, self.max_d), -self.max_d)

        # combine & clamp final command
        cmd_val = p_term + i_term + d_term
        cmd_val = max(min(cmd_val, 500.0), -500.0)

        # publish
        mc = ManualControl()
        mc.header.stamp = self.get_clock().now().to_msg()
        mc.x = mc.y = mc.z = 0.0
        mc.r = float(cmd_val)
        self.pub.publish(mc)

        # remember for next D
        self._prev_err = err

        # log
        self.get_logger().info(
            f"Heading={self.current:.1f}°, err={err:.2f}, "
            f"P={p_term:.2f}, I={i_term:.2f}, D={d_term:.2f} → r_cmd={cmd_val:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SimpleHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
