#!/usr/bin/env python3
import rclpy, math
from rclpy.node         import Node
from rclpy.qos          import qos_profile_sensor_data
from mavros_msgs.msg    import ManualControl, OverrideRCIn
from std_msgs.msg       import Float64, Float64MultiArray, Int16

class FSMMissionMode(Node):
    # ── States ─────────────────────────────────────────────────────
    STATE_INIT, STATE_DIVE, STATE_TURN_180, STATE_SCAN, \
    STATE_MOVE_TO_TAG, STATE_FLASH, STATE_FLASHING, STATE_IDLE = range(8)

    def __init__(self):
        super().__init__('fsm_mission_mode')

        # — Parameters you may tune —
        self.declare_parameter('dive_depth_step', 2.0)
        self.declare_parameter('vertical_speed', 0.07)
        self.declare_parameter('turn_speed', 0.05)
        self.declare_parameter('spin_rate', 0.05)
        self.declare_parameter('pose_tolerance', 0.1)
        self.declare_parameter('flash_duration', 2.0)
        self.declare_parameter('stick_scale', 1000.0)
        self.declare_parameter('heading_tolerance_deg', 5.0)
        self.declare_parameter('depth_timeout', 1.0)
        self.declare_parameter('heading_timeout', 1.0)
        self.declare_parameter('detection_timeout', 1.0)

        p                   = self.get_parameter
        self.dive_step      = p('dive_depth_step').value
        self.vert_speed     = p('vertical_speed').value
        self.turn_speed     = p('turn_speed').value
        self.spin_rate      = p('spin_rate').value
        self.pose_tol       = p('pose_tolerance').value
        self.flash_duration = p('flash_duration').value
        self.stick_scale    = p('stick_scale').value
        self.head_tol       = p('heading_tolerance_deg').value
        self.depth_timeout  = p('depth_timeout').value
        self.heading_timeout = p('heading_timeout').value
        self.detection_timeout = p('detection_timeout').value

        # — Comms —
        self.pub_manual = self.create_publisher(ManualControl, 'manual_control', 10)
        self.pub_lights = self.create_publisher(OverrideRCIn,   'override_rc',   10)
        self.create_subscription(Float64, '/depth', self.depth_cb, 10)
        self.create_subscription(Int16, '/heading', self.heading_cb, 10)
        self.create_subscription(Float64MultiArray, '/apriltag/detection', self.detect_cb, 10)

        # — State variables —
        self.state            = self.STATE_INIT
        self.current_depth    = None
        self.current_heading  = None
        self.last_depth_t     = None
        self.last_head_t      = None
        self.detection        = False
        self.tag_pose         = [0.0, 0.0, 0.0]
        self.last_det_t       = None
        self.initial_heading  = None
        self.target_heading   = None
        self.dive_target      = None
        self.flash_start      = None
        # scanning direction and initial heading
        self.scan_direction      = 1
        self.scan_heading_start  = None

        self.timer = self.create_timer(0.1, self.loop)   # 10 Hz
        self.get_logger().info('FSM mission node ready')

    # ── Call-backs ───────────────────────────────────────────────────
    def depth_cb(self, msg):   self.current_depth, self.last_depth_t = msg.data, self.now()
    def heading_cb(self, msg): self.current_heading, self.last_head_t = msg.data, self.now()
    def detect_cb(self, msg):
        data = list(msg.data)
        tag_id = int(data[0])
        # only accept tags 0 through 11
        if 0 <= tag_id <= 11:
            # store just the x,y,z
            self.tag_pose   = data[1:4]
            self.detection  = True
            self.last_det_t = self.now()
        else:
            # drop any other tag
            self.detection = False

    # ── Helpers ─────────────────────────────────────────────────────
    def now(self): return self.get_clock().now().nanoseconds * 1e-9
    def hold(self): self.pub_manual.publish(ManualControl())              # zero cmd
    def cmd(self, x=0.0, y=0.0, z=0.0, r=0.0):
        m = ManualControl()
        m.header.stamp = self.get_clock().now().to_msg()
        m.x, m.y, m.z, m.r = (v * self.stick_scale for v in (x, y, z, r))
        self.pub_manual.publish(m)

    # ── Main loop ───────────────────────────────────────────────────
    def loop(self):
        t = self.now()

        # Sensor stale check
        if (self.current_depth is None or self.current_heading is None or
            t - self.last_depth_t > self.depth_timeout or
            t - self.last_head_t  > self.heading_timeout):
            self.get_logger().warn('Stale sensors; holding')
            self.hold()
            return
        if self.detection and t - self.last_det_t > self.detection_timeout:
            self.detection = False

        # Idle state
        if self.state == self.STATE_IDLE:
            self.hold()
            return

        # Tag pre-empt (except when already moving or flashing)
        if self.detection and self.state not in (self.STATE_MOVE_TO_TAG,
                                                 self.STATE_FLASH,
                                                 self.STATE_FLASHING):
            self.state = self.STATE_MOVE_TO_TAG
            self.get_logger().info('TAG → MOVE_TO_TAG')

        # ── FSM ───────────────────────────────────────────────────────
        if self.state == self.STATE_INIT:
            self.initial_heading = self.current_heading
            self.dive_target     = self.current_depth + self.dive_step
            self.state           = self.STATE_DIVE
            self.get_logger().info(f'INIT → DIVE to {self.dive_target:.2f} m')

        elif self.state == self.STATE_DIVE:
            if self.current_depth < self.dive_target - 0.1:
                self.cmd(z=-self.vert_speed)
            else:
                self.hold()
                self.target_heading = (self.initial_heading + 180) % 360
                self.state          = self.STATE_TURN_180
                self.get_logger().info(f'DIVE done → TURN_180 to {self.target_heading:.1f}°')

        elif self.state == self.STATE_TURN_180:
            err = ((self.target_heading - self.current_heading + 540) % 360) - 180
            if abs(err) > self.head_tol:
                self.cmd(r=self.turn_speed if err > 0 else -self.turn_speed)
            else:
                self.hold()
                self.state = self.STATE_SCAN
                self.scan_heading_start = self.current_heading
                self.scan_direction = 1
                self.get_logger().info('TURN complete → SCAN start')

        elif self.state == self.STATE_SCAN:
            # continuous spin
            self.cmd(r=self.scan_direction * self.spin_rate)
            # check for full revolution
            delta = (self.current_heading - self.scan_heading_start + 360) % 360
            if delta > 350:
                self.scan_direction *= -1
                self.scan_heading_start = self.current_heading
                self.get_logger().info(f'Full revolution → reverse scan direction to {self.scan_direction}')

        elif self.state == self.STATE_MOVE_TO_TAG:
            # move until 1m away
            target_dist = 1.0
            dx, dy, dz = self.tag_pose
            v = self.vert_speed
            x_cmd =  v if dz > target_dist + self.pose_tol else -v if dz < target_dist - self.pose_tol else 0
            y_cmd =  v if dx >  self.pose_tol else -v if dx < -self.pose_tol else 0
            z_cmd =  v if -dy > self.pose_tol else -v if -dy < -self.pose_tol else 0
            self.cmd(x=x_cmd, y=y_cmd, z=z_cmd)
            if abs(dz - target_dist) < self.pose_tol and abs(dx) < self.pose_tol and abs(dy) < self.pose_tol:
                self.hold()
                self.detection = False
                self.state     = self.STATE_FLASH
                self.get_logger().info('At 1m from tag → FLASH')

        elif self.state == self.STATE_FLASH:
            on = OverrideRCIn()
            on.channels = [2000] * 8
            self.pub_lights.publish(on)
            self.flash_start = t
            self.state       = self.STATE_FLASHING
            self.get_logger().info('FLASHING…')

        elif self.state == self.STATE_FLASHING:
            if t - self.flash_start > self.flash_duration:
                off = OverrideRCIn()
                off.channels = [1000] * 8
                self.pub_lights.publish(off)
                self.state = self.STATE_IDLE
                self.get_logger().info('FLASH done → IDLE')


def main():
    rclpy.init()
    node = FSMMissionMode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
