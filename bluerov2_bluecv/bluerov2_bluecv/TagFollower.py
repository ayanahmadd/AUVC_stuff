#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int16
from mavros_msgs.msg import ManualControl, OverrideRCIn
from bluerov2_controllers.bluerov2_controllers.PIDController import PIDController

class AprilTagFollower(Node):
    def __init__(self):
        super().__init__('apriltag_follower_pid')

        # PID Controllers
        self.forward_pid = PIDController(kp=1.2, ki=0.01, kd=0.4, setpoint=1.0)  # target z = 1 m
        self.yaw_pid     = PIDController(kp=500.0, ki=5.0, kd=200.0, setpoint=0.0)  # target x = 0

        # Publishers & Subscribers
        self.create_subscription(Float64MultiArray, 'apriltag/detection', self.detection_cb, 10)
        self.create_subscription(Int16, '/heading',            self.heading_cb,   10)
        self.pub_ctrl = self.create_publisher(ManualControl,  '/manual_control', 10)
        self.pub_lights = self.create_publisher(OverrideRCIn, 'override_rc',     10)

        self.timer = self.create_timer(0.1, self.control_loop)

        # State
        self.last_detection_time = None
        self.tag_position = None  # [x, y, z]
        self.current_heading = None
        self.spin_start_heading = None
        self.spin_direction = 1     # 1 = CW, -1 = CCW
        self.spin_threshold = 330.0 # degrees

        self.get_logger().info("AprilTag Follower PID node started. Using heading-based spin alternation.")

    def turn_lights_on(self, level: int):
        """ level: 0 (off) → 100 (max) """
        cmd = OverrideRCIn()
        # leave all channels unchanged except 8 & 9
        cmd.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        cmd.channels[8] = 1000 + level * 10
        cmd.channels[9] = 1000 + level * 10
        self.pub_lights.publish(cmd)
        self.get_logger().debug(f"Lights set to level {level}")

    def detection_cb(self, msg: Float64MultiArray):
        if len(msg.data) != 7:
            return
        # data = [id, x, y, z, roll, pitch, yaw]
        self.tag_position = msg.data[1:4]
        self.last_detection_time = self.get_clock().now()

    def heading_cb(self, msg: Int16):
        self.current_heading = float(msg.data)

    def control_loop(self):
        now = self.get_clock().now()
        mc = ManualControl()
        mc.header.stamp = now.to_msg()

        # 1) If stale/no detection → spin in place
        if (self.last_detection_time is None or
            (now - self.last_detection_time).nanoseconds * 1e-9 > 1.5):

            if self.current_heading is None:
                self.get_logger().warn("No heading data yet; cannot spin.")
                return

            # initialize spin
            if self.spin_start_heading is None:
                self.spin_start_heading = self.current_heading
                self.get_logger().info(f"Started spinning from heading {self.spin_start_heading:.1f}°")

            # measure delta rotation
            delta = (self.current_heading - self.spin_start_heading + 360.0) % 360.0
            if delta >= self.spin_threshold:
                self.spin_direction *= -1
                self.spin_start_heading = self.current_heading
                self.get_logger().info(
                    f"Completed ~360° spin; switching spin to "
                    f"{'CW' if self.spin_direction==1 else 'CCW'}."
                )

            mc.x = 0.0
            mc.y = 0.0
            mc.z = 0.0
            mc.r = 400.0 * self.spin_direction
            self.pub_ctrl.publish(mc)

            self.get_logger().info(
                f"Spinning {'CW' if self.spin_direction==1 else 'CCW'} "
                f"(heading: {self.current_heading:.1f}°)"
            )

            # Lights off when searching
            self.turn_lights_on(0)
            return

        # 2) Tag is fresh → reset spin state
        self.spin_start_heading = None

        # 3) Extract position & run PID
        x, y, z = self.tag_position
        forward_cmd = self.forward_pid.compute(z)
        yaw_cmd     = self.yaw_pid.compute(x)

        # clamp
        forward_cmd = max(min(forward_cmd, 600.0), -600.0)
        yaw_cmd     = max(min(yaw_cmd,     600.0), -600.0)

        mc.x = float(forward_cmd)
        mc.y = 0.0
        mc.z = 0.0
        mc.r = float(yaw_cmd)
        self.pub_ctrl.publish(mc)

        self.get_logger().info(
            f"Tag z={z:.2f} m → forward={forward_cmd:.1f}, "
            f"x={x:.2f} m → yaw={yaw_cmd:.1f}"
        )

        # 4) Flash lights if within 1 m
        if z < 1.0:
            self.turn_lights_on(100)
        else:
            self.turn_lights_on(0)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()