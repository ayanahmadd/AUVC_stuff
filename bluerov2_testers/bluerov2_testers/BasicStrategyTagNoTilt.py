#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Int16
from mavros_msgs.msg import ManualControl, OverrideRCIn
from rclpy.callback_groups import ReentrantCallbackGroup
import math
from bluerov2_controllers.PIDController import PIDController

class BasicTagMission(Node):
    def __init__(self):
        super().__init__('basic_tag_mission')

        self.fast_group = ReentrantCallbackGroup()

        self.pub_target_depth = self.create_publisher(Float64, 'target_depth', 10)
        self.pub_manual = self.create_publisher(ManualControl, '/manual_control', 10)
        self.pub_lights = self.create_publisher(OverrideRCIn, 'override_rc', 10)
        self.pub_camera_tilt = self.create_publisher(Float64, 'camera_tilt', 10)

        self.create_subscription(Float64, 'depth', self.depth_cb, 10, callback_group=self.fast_group)
        self.create_subscription(Float64MultiArray, 'apriltag/detection', self.detect_cb, 10, callback_group=self.fast_group)
        self.create_subscription(Int16, '/heading', self.heading_cb, 10, callback_group=self.fast_group)

        self.state = 0
        self.start_time = self.get_clock().now()
        self.rescan_start_time = None
        self.state6_start_time = None

        self.current_depth = None
        self.tag_position = None
        self.initial_tag_pos = None
        self.current_heading = None

        self.DIVE_DEPTH = 2.0
        self.DIVE_TOL = 0.10
        self.DIVE_SETTLE = 2.0
        self.BACK_SPEED = -0.20
        self.BACK_DIST = 2.0

        self.backed_dist = 0.0
        self.last_back_ts = None

        self.SCAN_STEP = 45.0
        self.SCAN_TOL = 2.0
        self.SCAN_HOLD = 2.0
        self.SPIN_RATE = 60.0

        self.cw_offsets = [i * self.SCAN_STEP for i in range(1, 9)]
        self.scanning_fw = True
        self.start_heading = None
        self.next_heading = None
        self.offset_idx = 0
        self.hold_start = None

        self.TILT_MIN = -30.0
        self.TILT_MAX = 30.0

        self.yaw_pid = PIDController(kp=2.0, ki=0.1, kd=0.75, setpoint=0.0, dt=0.1)

        self.create_timer(0.1, self.timer_cb, callback_group=self.fast_group)
        self.get_logger().info("🚀 BasicTagMission node started")

    def depth_cb(self, msg: Float64):
        self.current_depth = msg.data

    def detect_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            self.tag_position = list(msg.data[1:4])

    def heading_cb(self, msg: Int16):
        self.current_heading = float(msg.data)

    @staticmethod
    def angle_diff(target: float, current: float) -> float:
        return (target - current + 180.0) % 360.0 - 180.0

    def turn_lights(self, level: int):
        cmd = OverrideRCIn()
        cmd.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        for ch in (8, 9):
            cmd.channels[ch] = 1000 + level * 10
        self.pub_lights.publish(cmd)

    def timer_cb(self):
        now = self.get_clock().now()

        if self.state == 0:
            self.pub_target_depth.publish(Float64(data=self.DIVE_DEPTH))
            self.start_time = now
            self.state = 1
            self.get_logger().info("→ State 0: Diving to %.1f m" % self.DIVE_DEPTH)

        elif self.state == 1:
            if (self.current_depth is not None and
                abs(self.current_depth - self.DIVE_DEPTH) <= self.DIVE_TOL and
                (now - self.start_time).nanoseconds * 1e-9 >= self.DIVE_SETTLE):
                self.state = 2
                self.backed_dist = 0.0
                self.last_back_ts = now
                self.get_logger().info("→ Depth settled, start backing up")

        elif self.state == 2:
            dt = (now - self.last_back_ts).nanoseconds * 1e-9
            self.last_back_ts = now
            self.backed_dist += abs(self.BACK_SPEED) * dt
            if self.backed_dist < self.BACK_DIST:
                self.pub_manual.publish(ManualControl(x=self.BACK_SPEED, y=0.0, z=0.0, r=0.0))
            else:
                self.pub_manual.publish(ManualControl(x=0.0, y=0.0, z=0.0, r=0.0))
                self.state = 3
                self.get_logger().info("→ Back‑up complete (%.1f m)" % self.BACK_DIST)

        elif self.state == 3:
            if self.current_heading is not None:
                self.start_heading = self.current_heading
                self.scanning_fw = True
                self.offset_idx = 0
                self.next_heading = (self.start_heading + self.cw_offsets[0]) % 360.0
                self.hold_start = None
                self.state = 4
                self.get_logger().info(f"→ Scan initialised, first spoke at {self.next_heading:.1f}°")

        elif self.state == 4:
            if self.tag_position is not None:
                self.initial_tag_pos = list(self.tag_position)
                self.state = 5
                self.get_logger().info("→ AprilTag detected, begin approach")
                return

            if self.current_heading is None:
                self.pub_manual.publish(ManualControl(r=self.SPIN_RATE, x=0.0, y=0.0, z=0.0))
                return

            diff = self.angle_diff(self.next_heading, self.current_heading)
            if abs(diff) <= self.SCAN_TOL:
                if self.hold_start is None:
                    self.hold_start = now
                    self.pub_manual.publish(ManualControl(r=0.0, x=0.0, y=0.0, z=0.0))
                    self.get_logger().info(f"   Holding {self.next_heading:.1f}°")
                elif (now - self.hold_start).nanoseconds * 1e-9 >= self.SCAN_HOLD:
                    self.offset_idx += 1
                    if self.offset_idx >= len(self.cw_offsets):
                        self.scanning_fw = not self.scanning_fw
                        self.offset_idx = 0
                    offsets = self.cw_offsets if self.scanning_fw else [-o for o in self.cw_offsets]
                    self.next_heading = (self.start_heading + offsets[self.offset_idx]) % 360.0
                    self.hold_start = None
                    self.get_logger().info(f"→ Next spoke {self.next_heading:.1f}°")
            else:
                rate = self.SPIN_RATE if diff > 0 else -self.SPIN_RATE
                self.pub_manual.publish(ManualControl(r=rate, x=0.0, y=0.0, z=0.0))

        elif self.state == 5:
            x, y, z = self.tag_position

            if abs(y) > 0.10:
                tilt_deg = max(min(math.degrees(math.atan2(y, z)), self.TILT_MAX), self.TILT_MIN)
                self.pub_camera_tilt.publish(Float64(data=tilt_deg))

            fwd_cmd = 70.0 * (z - 1.0)
            fwd_cmd = max(min(fwd_cmd, 70.0), -70.0)
            cf = fwd_cmd / 70.0

            yaw_cmd = self.yaw_pid.compute(x)
            yaw_cmd = max(min(yaw_cmd, 70.0), -70.0)
            cy = yaw_cmd / 70.0

            self.pub_manual.publish(ManualControl(x=cf, y=0.0, z=0.0, r=cy))

            x0, y0, z0 = self.initial_tag_pos
            if (abs(z - z0) <= 0.05 and abs(x) <= 1.0 and abs(y - y0) <= 0.05):
                self.pub_manual.publish(ManualControl(x=0.0, y=0.0, z=0.0, r=0.0))
                self.rescan_start_time = now
                self.state = 55
                self.get_logger().info("→ Holding; verifying ≤1 m")

        elif self.state == 55:
            if (now - self.rescan_start_time).nanoseconds * 1e-9 < 1.0:
                return
            if self.tag_position is not None and self.tag_position[2] <= 1.0:
                self.turn_lights(100)
                self.state6_start_time = now
                self.state = 6
                self.get_logger().info("✅ Confirmed, flashing lights")
            else:
                self.get_logger().info("❌ Verification failed — restarting mission")
                self.tag_position = None
                self.state = 0
                self.start_time = now

        elif self.state == 6:
            if (now - self.state6_start_time).nanoseconds * 1e-9 >= 3.0:
                self.turn_lights(0)
                self.state = 7
                self.get_logger().info("→ Lights off, mission complete")

        elif self.state == 7:
            pass

def main():
    rclpy.init()
    node = BasicTagMission()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
