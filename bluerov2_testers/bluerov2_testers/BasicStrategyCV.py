#!/usr/bin/env python3
"""
Finite-state mission node that drives a BlueROV2 solely through topic commands:
* Vertical motion is delegated to a separate DepthPIDController listening to `/target_depth`.
* Yaw motion is delegated to HeadingLockOnly (PID) listening to `/target_heading`.
* This node orchestrates the mission: dive, 180° turn, step-scan every 45° holding 5 s
  at each heading, reverses direction after each full revolution, runs YOLOv8 on the
  camera stream, approaches a detected ROV to 1 m, flashes the lights, and idles.
"""
import rclpy, cv2, numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Int16
from mavros_msgs.msg import ManualControl, OverrideRCIn
from cv_bridge import CvBridge
from ultralytics import YOLO

# === Camera intrinsics (px) ===
FX, FY = 273.25, 261.76
CX, CY = 307.89, 153.84

# === ROV physical dimensions (m) ===
FRONT_WIDTH, FRONT_HEIGHT = 0.33805, 0.251
SIDE_WIDTH,  SIDE_HEIGHT  = 0.4572,  0.251

class FSMMissionMode(Node):
    STATE_INIT, STATE_DIVE, STATE_TURN_180, STATE_SCAN, \
    STATE_MOVE_TO_TARGET, STATE_FLASH, STATE_FLASHING, STATE_IDLE = range(8)

    def __init__(self):
        super().__init__('fsm_mission_mode')
        self.fast_group = ReentrantCallbackGroup()
        self.slow_group = ReentrantCallbackGroup()

        # ── Parameters ──
        defaults = {
            'dive_depth_step': 2.0,
            'vertical_speed':  0.06,
            'pose_tolerance':  0.10,
            'flash_duration':  2.0,
            'stick_scale':     1000.0,
            'heading_tolerance_deg': 3.0,
            'depth_timeout':      1.0,
            'heading_timeout':    1.0,
            'detection_timeout':  1.0,
            'scan_step_deg':      45.0,
            'scan_hold_sec':      5.0,
            'scan_tol_deg':       2.0,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        gp = self.get_parameter
        self.dive_step   = gp('dive_depth_step').value
        self.vert_speed  = gp('vertical_speed').value
        self.pose_tol    = gp('pose_tolerance').value
        self.flash_dur   = gp('flash_duration').value
        self.scale       = gp('stick_scale').value
        self.head_tol    = gp('heading_tolerance_deg').value
        self.depth_to    = gp('depth_timeout').value
        self.head_to     = gp('heading_timeout').value
        self.det_to      = gp('detection_timeout').value
        self.scan_step   = gp('scan_step_deg').value
        self.scan_hold   = gp('scan_hold_sec').value
        self.scan_tol    = gp('scan_tol_deg').value

        # ── Publishers ──
        self.pub_manual = self.create_publisher(ManualControl, '/manual_control', 10, callback_group=self.fast_group)
        self.pub_lights = self.create_publisher(OverrideRCIn,   '/override_rc',    10, callback_group=self.fast_group)
        self.pub_tdepth = self.create_publisher(Float64,        '/target_depth',   10, callback_group=self.fast_group)
        self.pub_thead  = self.create_publisher(Float64,        '/target_heading', 10, callback_group=self.fast_group)

        # ── Subscriptions ──
        self.create_subscription(Float64, '/depth',   self.depth_cb,   qos_profile_sensor_data, callback_group=self.fast_group)
        self.create_subscription(Int16,   '/heading', self.heading_cb, qos_profile_sensor_data, callback_group=self.fast_group)
        self.bridge = CvBridge()
        self.model  = YOLO('bluerov2_bluecv/bluerov2_bluecv/yolov8n.pt')
        self.create_subscription(Image,   '/camera',  self.image_cb,   10, callback_group=self.slow_group)

        # ── State vars ──
        self.state      = self.STATE_INIT
        self.cur_depth  = None
        self.cur_head   = None
        self.t_depth    = None
        self.t_head     = None
        self.detection  = False
        self.rov_pos    = None
        self.t_det      = None
        self.init_head  = None
        self.turn_target= None
        self.dive_target= None
        self.flash_start= None

        # ── Scan sequencing ──
        self.scan_offsets = [i * self.scan_step for i in range(8)]  # [45, 90, ..., 360)
        self.scan_forward = True
        self.scan_idx     = 0
        self.scan_hold_start = None

        self.create_timer(0.1, self.loop, callback_group=self.fast_group)
        self.get_logger().info('FSM mission node ready')

    # ── Callbacks ──
    def depth_cb(self, msg: Float64):
        self.cur_depth, self.t_depth = msg.data, self.now()

    def heading_cb(self, msg: Int16):
        self.cur_head, self.t_head = float(msg.data), self.now()

    def image_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        small = cv2.resize(img, (320, 240))
        results = self.model.predict(small, conf=0.5)[0]
        box = self.best_box(results.boxes)
        if box:
            pose = self.pose_from_box(box)
            if pose:
                self.rov_pos, self.detection, self.t_det = pose, True, self.now()
            else:
                self.detection = False
        else:
            self.detection = False

    # ── Helpers ──
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def best_box(self, boxes, min_area=500):
        best, area = None, 0
        for b in boxes:
            x1, y1, x2, y2 = map(int, b.xyxy[0])
            w, h = x2 - x1, y2 - y1
            a = w * h
            if a >= min_area and a > area:
                best, area = (x1, y1, w, h), a
        return best

    def pose_from_box(self, box):
        x, y, w, h = box
        if w <= 0 or h <= 0:
            return None
        z_front = (FRONT_WIDTH * FX) / w
        z_side  = (SIDE_WIDTH  * FX) / w
        if z_front > z_side:
            real_w, real_h = FRONT_WIDTH, FRONT_HEIGHT
        else:
            real_w, real_h = SIDE_WIDTH, SIDE_HEIGHT
        z_w = (real_w * FX) / w
        z_h = (real_h * FY) / h
        z   = (z_w + z_h) / 2.0
        cx_img = x + w / 2
        cy_img = y + h / 2
        dx = cx_img - CX
        dy = cy_img - CY
        x_m = (dx * z) / FX
        y_m = -(dy * z) / FY
        return [round(x_m, 4), round(y_m, 4), round(z, 4)]

    def send_target_depth(self, val):
        self.pub_tdepth.publish(Float64(data=float(val)))

    def send_target_heading(self, val):
        self.pub_thead.publish(Float64(data=float(val % 360)))

    def diff_angle(self, a, b):
        return ((a - b + 180) % 360) - 180

    # ── Main FSM loop ──
    def loop(self):
        t = self.now()
        # sensor freshness
        if (self.cur_depth is None or self.cur_head is None or
            t - self.t_depth > self.depth_to or
            t - self.t_head  > self.head_to):
            return
        if self.detection and t - self.t_det > self.det_to:
            self.detection = False
        if self.state == self.STATE_IDLE:
            return
        if self.detection and self.state not in (self.STATE_MOVE_TO_TARGET, self.STATE_FLASH, self.STATE_FLASHING):
            self.state = self.STATE_MOVE_TO_TARGET

        if self.state == self.STATE_INIT:
            self.init_head   = self.cur_head
            self.dive_target = self.cur_depth + self.dive_step
            self.send_target_depth(self.dive_target)
            self.state = self.STATE_DIVE

        elif self.state == self.STATE_DIVE:
            if abs(self.cur_depth - self.dive_target) < 0.05:
                self.turn_target = (self.init_head + 180) % 360
                self.send_target_heading(self.turn_target)
                self.state = self.STATE_TURN_180

        elif self.state == self.STATE_TURN_180:
            if abs(self.diff_angle(self.cur_head, self.turn_target)) < self.head_tol:
                self._start_scan_sequence()

        elif self.state == self.STATE_SCAN:
            if abs(self.diff_angle(self.cur_head, self.scan_target)) < self.scan_tol:
                if self.scan_hold_start is None:
                    self.scan_hold_start = t
                elif t - self.scan_hold_start >= self.scan_hold:
                    self._advance_scan()
            else:
                self.scan_hold_start = None

        elif self.state == self.STATE_MOVE_TO_TARGET:
            self._approach_target()

        elif self.state == self.STATE_FLASH:
            on = OverrideRCIn(); on.channels = [2000] * 8
            self.pub_lights.publish(on)
            self.flash_start = t
            self.state = self.STATE_FLASHING

        elif self.state == self.STATE_FLASHING and t - self.flash_start > self.flash_dur:
            off = OverrideRCIn(); off.channels = [1000] * 8
            self.pub_lights.publish(off)
            self.state = self.EVENT_IDLE

    # ── Scan helpers ──
    def _start_scan_sequence(self):
        self.state = self.STATE_SCAN
        self.scan_forward = True
        self.scan_idx     = 0
        self.scan_hold_start = None
        self.scan_base = self.cur_head
        self.scan_target = (self.scan_base + self.scan_offsets[0]) % 360
        self.send_target_heading(self.scan_target)

    def _advance_scan(self):
        self.scan_idx += 1
        if self.scan_idx >= len(self.scan_offsets):
            self.scan_forward = not self.scan_forward
            self.scan_idx = 0
        offset_list = self.scan_offsets if self.scan_forward else [-o for o in self.scan_offsets]
        self.scan_target = (self.scan_base + offset_list[self.scan_idx]) % 360
        self.send_target_heading(self.scan_target)
        self.scan_hold_start = None

    # ── Approach helper ──
    def _approach_target(self):
        d   = 1.0
        x,y,z = self.rov_pos
        v   = self.vert_speed
        # vertical via depth PID
        self.send_target_depth(self.cur_depth + y)
        # forward/side via manual control
        mc = ManualControl()
        mc.x = v if z > d + self.pose_tol else -v if z < d - self.pose_tol else 0.0
        mc.y = v if x > self.pose_tol else -v if x < -self.pose_tol else 0.0
        mc.z = 0.0
        mc.r = 0.0
        self.pub_manual.publish(mc)
        if abs(z - d) < self.pose_tol and abs(x) < self.pose_tol and abs(y) < self.pose_tol:
            self.pub_manual.publish(ManualControl())
            self.state = self.STATE_FLASH

# ── main ──
def main():
    rclpy.init()
    node = FSMMissionMode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
