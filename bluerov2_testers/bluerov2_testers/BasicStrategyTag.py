#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Int16
from mavros_msgs.msg import ManualControl, OverrideRCIn


"""
@Assuming Lane Detection Does NOT Work
- Dives then moves Back
- Camera tilts up (untested) -> Robot starts rotating
- Camera tilts down (untested) -> Robot rotates otherway
-- until finds april tag
--- moves toward the tag
[rescan right here, to confirm]
---- turn on lights

find robot
go to tag pos
flash asap
"""

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # Publishers
        self.pub_target_depth = self.create_publisher(Float64,    'target_depth',    10)
        self.pub_manual       = self.create_publisher(ManualControl, '/manual_control', 10)
        self.pub_lights       = self.create_publisher(OverrideRCIn, 'override_rc',     10)
        self.pub_camera_tilt  = self.create_publisher(Float64,    'camera_tilt',     10)

        # Subscribers
        self.create_subscription(Float64,            'depth',              self.depth_cb,     10)
        self.create_subscription(Float64MultiArray, 'apriltag/detection', self.detect_cb,    10)
        self.create_subscription(Int16,              '/heading',           self.heading_cb,   10)

        # State machine
        self.state       = 0
        self.start_time  = self.get_clock().now()
        self.state6_start_time = None

        # Sensor readings
        self.current_depth   = None
        self.tag_position    = None   # [x, y, z]
        self.initial_tag_position = None
        self.current_heading = None

        # Tilt parameters
        self.TILT_DOWN      = -30.0
        self.TILT_UP        =  30.0
        self.current_tilt   = self.TILT_UP

        # Spin parameters (reduced)
        self.spin_start_head = None
        self.spin_dir        = 1       # 1 = CW, -1 = CCW
        self.SPIN_RATE      = 60.0     # °/s
        # track last revolution to toggle tilt
        # Movement constants
        self.DIVE_DEPTH     = 2.0
        self.DIVE_TOL       = 0.1
        self.DIVE_SETTLE    = 2.0
        self.BACK_SPEED     = -0.2    # m/s backward
        self.BACK_DISTANCE  = 2.0     # m

        # For distance integration
        self.backed_distance = 0.0
        self.last_back_time  = None

        # Main timer
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("🚀 Mission node started")

    def depth_cb(self, msg: Float64):
        self.current_depth = msg.data

    def detect_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            self.tag_position = msg.data[1:4]

    def heading_cb(self, msg: Int16):
        self.current_heading = float(msg.data)

    def turn_lights(self, level: int):
        cmd = OverrideRCIn()
        cmd.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        cmd.channels[8] = 1000 + level * 10
        cmd.channels[9] = 1000 + level * 10
        self.pub_lights.publish(cmd)

    def timer_cb(self):
        now = self.get_clock().now()

        # ——— State 0: Dive to 2 m ———
        if self.state == 0:
            self.pub_target_depth.publish(Float64(data=self.DIVE_DEPTH))
            self.start_time = now
            self.state = 1
            self.get_logger().info("→ State 0: Diving to 2 m")

        # ——— State 1: Wait at depth + settle ———
        elif self.state == 1:
            if (self.current_depth is not None
                and abs(self.current_depth - self.DIVE_DEPTH) <= self.DIVE_TOL
                and (now - self.start_time).nanoseconds * 1e-9 >= self.DIVE_SETTLE):
                self.get_logger().info("→ Reached dive depth")
                self.state = 2
                # initialize distance integration
                self.backed_distance  = 0.0
                self.last_back_time   = now

        # ——— State 2: Back up by *distance* ———
        elif self.state == 2:
            dt = (now - self.last_back_time).nanoseconds * 1e-9
            self.last_back_time = now
            # accumulate how far we've gone
            self.backed_distance += abs(self.BACK_SPEED) * dt

            if self.backed_distance < self.BACK_DISTANCE:
                self.pub_manual.publish(
                    ManualControl(x=self.BACK_SPEED, y=0.0, z=0.0, r=0.0)
                )
            else:
                self.get_logger().info("→ Finished backing up (distance reached)")
                self.pub_manual.publish(ManualControl(x=0.0, y=0.0, z=0.0, r=0.0))
                self.state = 3

        # ——— State 3: Initial camera tilt up ———
        elif self.state == 3:
            self.pub_camera_tilt.publish(Float64(data=self.TILT_UP))
            self.current_tilt = self.TILT_UP
            self.state = 4
            self.get_logger().info(f"→ State 3: Tilt to {self.TILT_UP}°")

        # ——— State 4: Spin & toggle tilt per revolution ———
        elif self.state == 4:
            # a) Spin in place
            mc = ManualControl(x=0.0, y=0.0, z=0.0, r=0.0)
            if self.current_heading is not None:
                if self.spin_start_head is None:
                    self.spin_start_head = self.current_heading
                delta = (self.current_heading - self.spin_start_head + 360.0) % 360.0
                mc.r = self.SPIN_RATE * self.spin_dir
                # once we've spun a full 360°, flip direction & tilt
                if delta >= 360.0:
                    self.spin_dir *= -1
                    self.spin_start_head = self.current_heading
                    self.current_tilt = (
                        self.TILT_DOWN if self.current_tilt == self.TILT_UP
                        else self.TILT_UP
                    )
                    self.pub_camera_tilt.publish(Float64(data=self.current_tilt))
                    self.get_logger().info(
                        f"→ Full revolution: tilt to {self.current_tilt}°"
                    )
            else:
                mc.r = self.SPIN_RATE
            self.pub_manual.publish(mc)

            # b) If tag appears, snapshot and go
            if self.tag_position is not None:
                self.initial_tag_position = list(self.tag_position)
                self.state = 5
                self.get_logger().info("→ AprilTag detected! Beginning approach")

        # ——— State 5: Approach & flash when back to original pose ———
        elif self.state == 5:
            x, y, z = self.tag_position
            raw_fwd = (z - 1.0) * 500.0
            raw_yaw = x * 500.0
            cf = max(min(raw_fwd, 600.0), -600.0) / 600.0
            cy = max(min(raw_yaw, 600.0), -600.0) / 600.0
            self.pub_manual.publish(ManualControl(x=cf, y=0.0, z=0.0, r=cy))

            # flash once back to the same relative pose
            x0, y0, z0 = self.initial_tag_position
            if (abs(z - z0) <= 0.05
                and abs(x) <= 1.0
                and abs(y - y0) <= 0.05):
                self.get_logger().info("→ Pose reached: flashing lights")
                self.turn_lights(100)
                self.state6_start_time = now
                self.state = 6

        # ——— State 6: Idle w/ lights on (3 s) ———
        elif self.state == 6:
            if (now - self.state6_start_time).nanoseconds * 1e-9 >= 3.0:
                self.get_logger().info("→ Turning lights off")
                self.turn_lights(0)
                self.state = 7

        # ——— State 7: Done ———
        elif self.state == 7:
            pass  # mission complete

def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()