#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Int16
from mavros_msgs.msg import ManualControl, OverrideRCIn

class BasicTagMission(Node):
    def __init__(self):
        super().__init__('basic_tag_mission')

        # — Publishers —
        self.pub_target_depth = self.create_publisher(Float64,    'target_depth',    10)
        self.pub_manual       = self.create_publisher(ManualControl, '/manual_control', 10)
        self.pub_lights       = self.create_publisher(OverrideRCIn, 'override_rc',     10)

        # — Subscribers —
        self.create_subscription(Float64,            'depth',              self.depth_cb,     10)
        self.create_subscription(Float64MultiArray, 'apriltag/detection', self.detect_cb,    10)
        self.create_subscription(Int16,              '/heading',           self.heading_cb,   10)

        # — State & timing —
        self.state             = 0
        self.start_time        = self.get_clock().now()
        self.state6_start_time = None

        # — Sensor storage —
        self.current_depth   = None
        self.tag_position    = None   # [x, y, z]
        self.initial_tag_position = None
        self.current_heading = None

        # — Scan parameters —
        self.SCAN_STEP    = 45.0    # degrees between each stop
        self.SCAN_TOL     =  2.0    # ± degrees tolerance to consider “at” the heading
        self.SCAN_HOLD    =  2.0    # seconds to hold & scan at each stop
        self.SPIN_RATE    = 60.0    # °/s rotation speed

        # will be initialized when we first enter state 3
        self.start_heading    = None
        self.next_heading     = None
        self.hold_start       = None

        # scan offsets for alternating direction
        self.cw_offsets       = [i * self.SCAN_STEP for i in range(1, 9)]
        self.scanning_forward = True
        self.offset_idx       = 0

        # — Dive & back-up constants —
        self.DIVE_DEPTH    = 2.0     # meters
        self.DIVE_TOL      = 0.1     # meters
        self.DIVE_SETTLE   = 2.0     # seconds
        self.BACK_SPEED    = -1.0    # m/s (negative = backwards, full reverse)
        self.BACK_DISTANCE = 1.0     # meters

        # for integrating how far we’ve backed up
        self.backed_distance = 0.0
        self.last_back_time  = None

        # — Main loop @10 Hz —
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("🚀 BasicTagMission started")

    def depth_cb(self, msg: Float64):
        self.current_depth = msg.data

    def detect_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            # [id, x, y, z, …]
            self.tag_position = msg.data[1:4]

    def heading_cb(self, msg: Int16):
        self.current_heading = float(msg.data)

    def turn_lights(self, level: int):
        cmd = OverrideRCIn()
        cmd.channels = [OverrideRCIn.CHAN_NOCHANGE]*10
        cmd.channels[8] = 1000 + level*10
        cmd.channels[9] = 1000 + level*10
        self.pub_lights.publish(cmd)

    def angle_diff(self, target: float, current: float) -> float:
        """Smallest signed difference from current → target."""
        a = (target - current + 180.0) % 360.0 - 180.0
        return a

    def timer_cb(self):
        now = self.get_clock().now()

        # — State 0: Dive to set depth —
        if self.state == 0:
            self.pub_target_depth.publish(Float64(data=self.DIVE_DEPTH))
            self.start_time = now
            self.state = 1
            self.get_logger().info("→ State 0: Diving to 2 m")

        # — State 1: Wait until depth ± tol & settle time —
        elif self.state == 1:
            if (self.current_depth is not None
                and abs(self.current_depth - self.DIVE_DEPTH) <= self.DIVE_TOL
                and (now - self.start_time).nanoseconds * 1e-9 >= self.DIVE_SETTLE):
                self.get_logger().info("→ Depth reached")
                self.state = 2
                self.backed_distance = 0.0
                self.last_back_time  = now

        # — State 2: Back up a fixed distance using distance integration —
        elif self.state == 2:
            dt = (now - self.last_back_time).nanoseconds * 1e-9
            self.last_back_time = now
            self.backed_distance += abs(self.BACK_SPEED) * dt

            if self.backed_distance < self.BACK_DISTANCE:
                self.pub_manual.publish(
                    ManualControl(x=self.BACK_SPEED, y=0.0, z=0.0, r=0.0)
                )
            else:
                self.pub_manual.publish(ManualControl(x=0.0, y=0.0, z=0.0, r=0.0))
                self.get_logger().info("→ Backed up 1 m")
                self.state = 3

        # — State 3: Initialize discrete 45° scan sequence —
        elif self.state == 3:
            if self.current_heading is not None:
                self.start_heading    = self.current_heading
                self.scanning_forward = True
                self.offset_idx       = 0
                # first scan offset
                self.next_heading     = (self.start_heading + self.cw_offsets[self.offset_idx]) % 360.0
                self.hold_start       = None
                self.state            = 4
                self.get_logger().info(
                    f"→ State 3: first scan at {self.next_heading:.1f}°"
                )

        # — State 4: Rotate to each 45° step, hold to scan —
        elif self.state == 4:
            # if we see a tag, snapshot and move to approach
            if self.tag_position is not None:
                self.initial_tag_position = list(self.tag_position)
                self.state = 5
                self.get_logger().info("→ AprilTag detected, approaching…")
                return

            # must have a heading to proceed
            if self.current_heading is None:
                # blind-spin until we get a heading
                self.pub_manual.publish(ManualControl(x=0.0, y=0.0, z=0.0, r=self.SPIN_RATE))
                return

            diff = self.angle_diff(self.next_heading, self.current_heading)

            # close enough → hold and scan
            if abs(diff) <= self.SCAN_TOL:
                if self.hold_start is None:
                    self.hold_start = now
                    self.pub_manual.publish(
                        ManualControl(x=0.0, y=0.0, z=0.0, r=0.0)
                    )
                    self.get_logger().info(
                        f"→ Holding at {self.next_heading:.1f}° for {self.SCAN_HOLD}s"
                    )
                elif (now - self.hold_start).nanoseconds * 1e-9 >= self.SCAN_HOLD:
                    # advance offset index
                    self.offset_idx += 1
                    if self.offset_idx >= len(self.cw_offsets):
                        # full sweep complete → reverse
                        self.scanning_forward = not self.scanning_forward
                        self.offset_idx = 0
                    offsets = self.cw_offsets if self.scanning_forward else [-o for o in self.cw_offsets]
                    self.next_heading = (self.start_heading + offsets[self.offset_idx]) % 360.0
                    self.hold_start = None
                    self.get_logger().info(
                        f"→ Next scan at {self.next_heading:.1f}°"
                    )
            else:
                # rotate toward next_heading
                rate = self.SPIN_RATE if diff > 0 else -self.SPIN_RATE
                self.pub_manual.publish(
                    ManualControl(x=0.0, y=0.0, z=0.0, r=rate)
                )

        # — State 5: Approach tag & flash when back to initial pose —
        elif self.state == 5:
            x, y, z = self.tag_position
            # simple proportional approach
            raw_fwd = (z - 1.0) * 500.0
            raw_yaw = x * 500.0
            cf = max(min(raw_fwd, 600.0), -600.0) / 600.0
            cy = max(min(raw_yaw, 600.0), -600.0) / 600.0
            self.pub_manual.publish(ManualControl(x=cf, y=0.0, z=0.0, r=cy))

            x0, y0, z0 = self.initial_tag_position
            if (abs(z - z0) <= 0.05
                and abs(x) <= 1.0
                and abs(y - y0) <= 0.05):
                self.turn_lights(100)
                self.state6_start_time = now
                self.state = 6
                self.get_logger().info("→ Flashing lights!")

        # — State 6: Keep lights on for 3 s, then off —
        elif self.state == 6:
            if (now - self.state6_start_time).nanoseconds * 1e-9 >= 3.0:
                self.turn_lights(0)
                self.state = 7
                self.get_logger().info("→ Lights off, mission complete")

        # — State 7: Done —
        elif self.state == 7:
            pass  # idle

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