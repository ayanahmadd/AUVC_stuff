#!/usr/bin/env python3
"""
ScanAndFlashStrategy
===================
ROS 2 mission node that *orchestrates* the existing heading-, depth- and vision-controller nodes to:
  1. Dive to 2 m.
  2. Perform a 360 ° scan in 45 ° increments.
  3. Rise to 1 m and repeat the scan.
  4. Toggle between 2 m and 1 m until an AprilTag is detected.
  5. Approach the tag, stopping 0.5 m short in the X-direction.
  6. Flash lights for 2 s, then restart at step 1.


**Prerequisite running nodes**
• `convert_to_depth`  (pressure → `/depth`)
• `depth_lock`        (DepthPIDController; listens on `/target_depth`)
• `heading_lock_only` (Heading PID;       listens on `/target_heading`)
• `apriltag_detector_node` (publishes `/apriltag/detection`)
• A camera driver publishing `sensor_msgs/Image` on `/camera`


This node only *publishes*:
• `std_msgs/Float64`   `/target_depth`
• `std_msgs/Float64`   `/target_heading`
• `mavros_msgs/ManualControl` `/manual_control` (x / y for surge/strafe)
• `mavros_msgs/OverrideRCIn`  `/override_rc`    (lights flash)
"""
import rclpy, math, time
from rclpy.node import Node
from std_msgs.msg import Float64, Int16, Float64MultiArray
from mavros_msgs.msg import ManualControl, OverrideRCIn


DEPTH_DEEP   = 2.0  # m
DEPTH_SHALLOW= 0.0  # m
HEAD_STEP    = 45   # deg
HEAD_TOL     = 3.0  # deg tolerance to consider heading reached
DEPTH_TOL    = 0.08 # m tolerance to consider depth reached
TAG_STALE    = 1.0  # s
FLASH_TIME   = 2.0  # s
APP_X_TOL    = 0.1  # m (target x-offset 0.5 ± tol)
APP_Y_TOL    = 0.1  # m lateral tol
APP_TARGET_X = 0.5  # m stand-off
CMD_SCALE    = 700  # ManualControl stick scale for x/y


class ScanAndFlashStrategy(Node):
    STATE_WAIT, STATE_SCAN, STATE_APPROACH, STATE_FLASH = range(4)


    def __init__(self):
        super().__init__('scan_and_flash_strategy')


        # ── pubs for other controllers ──
        self.pub_depth   = self.create_publisher(Float64,       '/target_depth',   10)
        self.pub_heading = self.create_publisher(Float64,       '/target_heading', 10)
        self.pub_manual  = self.create_publisher(ManualControl, '/manual_control', 10)
        self.pub_lights  = self.create_publisher(OverrideRCIn,  '/override_rc',    10)


        # ── subs ──
        self.create_subscription(Int16, '/heading', self.heading_cb, 10)
        self.create_subscription(Float64, '/depth', self.depth_cb, 10)
        self.create_subscription(Float64MultiArray, '/apriltag/detection', self.tag_cb, 10)


        # ── internal ──
        self.head   = None
        self.depth  = None
        self.tag_pos= None   # [id,x,y,z,...]
        self.t_tag  = 0.0
        self.scan_depth     = DEPTH_DEEP
        self.scan_start_head= None
        self.next_head      = None
        self.state = self.STATE_WAIT
        self.t_state = self.now()


        self.create_timer(0.1, self.loop)
        self.get_logger().info('Scan-and-flash strategy node initialised.')


    # time helper
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


    # subscriptions
    def heading_cb(self, msg: Int16):
        self.head = float(msg.data)


    def depth_cb(self, msg: Float64):
        self.depth = msg.data


    def tag_cb(self, msg: Float64MultiArray):
        self.tag_pos = list(msg.data)
        self.t_tag = self.now()


    # utilities
    def publish_depth(self, d):
        self.pub_depth.publish(Float64(data=float(d)))


    def publish_heading(self, h):
        self.pub_heading.publish(Float64(data=float(h % 360)))


    def heading_reached(self):
        if self.head is None or self.next_head is None:
            return False
        err = ((self.next_head - self.head + 540) % 360) - 180
        return abs(err) < HEAD_TOL


    def depth_reached(self):
        return self.depth is not None and abs(self.depth - self.scan_depth) < DEPTH_TOL


    def tag_fresh(self):
        return (self.now() - self.t_tag) < TAG_STALE


    # main FSM loop
    def loop(self):
        if self.head is None or self.depth is None:
            return  # wait for sensors


        if self.state == self.STATE_WAIT:
            # initialise first scan
            self.scan_start_head = round(self.head / HEAD_STEP) * HEAD_STEP
            self.next_head = self.scan_start_head % 360
            self.publish_heading(self.next_head)
            self.publish_depth(self.scan_depth)
            self.state = self.STATE_SCAN; self.t_state = self.now()
            self.get_logger().info('→ SCAN start')


        elif self.state == self.STATE_SCAN:
            # tag found?
            if self.tag_fresh():
                self.state = self.STATE_APPROACH; self.t_state = self.now()
                self.get_logger().info('Tag detected → APPROACH')
                return
            # move through headings
            if self.heading_reached():
                # step to next 45°
                self.next_head = (self.next_head + HEAD_STEP) % 360
                self.publish_heading(self.next_head)
                # completed full circle?
                if self.next_head == self.scan_start_head:
                    # toggle depth level
                    self.scan_depth = DEPTH_SHALLOW if self.scan_depth == DEPTH_DEEP else DEPTH_DEEP
                    self.publish_depth(self.scan_depth)
                    self.get_logger().info(f'Full rotation done → toggling depth to {self.scan_depth:.1f} m')
            # ensure depth PID is on target
            if not self.depth_reached():
                self.publish_depth(self.scan_depth)


        elif self.state == self.STATE_APPROACH:
            if not self.tag_fresh():
                # lost tag → resume scan
                self.state = self.STATE_SCAN; self.t_state = self.now()
                self.get_logger().info('Lost tag → resume scan')
                self.publish_manual(0,0)
                return
            _, x, y, z, *_ = self.tag_pos
            # keep 0.5 m standoff in +x (forward) axis, centre y
            dx = x - APP_TARGET_X
            dy = y
            surge  = CMD_SCALE if dx > APP_X_TOL else -CMD_SCALE if dx < -APP_X_TOL else 0
            strafe = CMD_SCALE if dy > APP_Y_TOL else -CMD_SCALE if dy < -APP_Y_TOL else 0
            self.publish_manual(surge, strafe)
            # align depth setpoint toward tag z coordinate
            self.publish_depth(self.depth + z)
            if abs(dx) < APP_X_TOL and abs(dy) < APP_Y_TOL:
                self.publish_manual(0,0)
                self.state = self.STATE_FLASH; self.t_state = self.now()
                self.get_logger().info('Within stand-off → FLASH')


        elif self.state == self.STATE_FLASH:
            rc = OverrideRCIn(); rc.channels=[2000]*8
            self.pub_lights.publish(rc)
            self.state = self.STATE_FLASHING; self.t_state = self.now()
        elif self.state == self.STATE_FLASHING:
            if self.now() - self.t_state > FLASH_TIME:
                rc = OverrideRCIn(); rc.channels=[1000]*8
                self.pub_lights.publish(rc)
                # restart mission
                self.scan_depth = DEPTH_DEEP
                self.state = self.STATE_WAIT; self.t_state = self.now()
                self.get_logger().info('Flash done → restart')


    def publish_manual(self, x=0.0, y=0.0):
        mc = ManualControl(x=float(x), y=float(y), z=0.0, r=0.0)
        self.pub_manual.publish(mc)




def main():
    rclpy.init()
    node = ScanAndFlashStrategy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()