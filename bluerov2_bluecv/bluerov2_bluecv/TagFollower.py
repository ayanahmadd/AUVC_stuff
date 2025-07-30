#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int16
from mavros_msgs.msg import ManualControl, OverrideRCIn
from bluerov2_controllers.bluerov2_controllers.PIDController import PIDController

class AprilTagFollower(Node):
    def __init__(self):
        super().__init__('apriltag_follower_pid')

        # PID Controllers with updated values
        self.forward_pid = PIDController(kp=70.0, ki=2.5, kd=5.0, setpoint=1.0)   # target z = 1 m
        self.yaw_pid     = PIDController(kp=2.0, ki=0.1, kd=0.75, setpoint=0.0)   # target x = 0 m

        # Publishers & Subscribers
        self.create_subscription(Float64MultiArray, 'apriltag/detection', self.detection_cb, 10)
        self.create_subscription(Int16, '/heading', self.heading_cb, 10)
        self.pub_ctrl = self.create_publisher(ManualControl, '/manual_control', 10)
        self.pub_lights = self.create_publisher(OverrideRCIn, 'override_rc', 10)

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
        """ Set light override level (0–100%) on RC channels 8 & 9 """
        cmd = OverrideRCIn()
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

        # === Stage 1: Spin if no tag detected recently ===
        if (self.last_detection_time is None or
            (now - self.last_detection_time).nanoseconds * 1e-9 > 1.5):

            if self.current_heading is None:
                self.get_logger().warn("No heading data yet; cannot spin.")
                return

            # Initialize spin direction
            if self.spin_start_heading is None:
                self.spin_start_heading = self.current_heading
                self.get_logger().info(f"Started spinning from heading {self.spin_
