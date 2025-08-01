#!/usr/bin/env python3
"""
Finite-state mission node (BlueROV2) – **updated** per July 31 specs
Initial dive → deep scan (10 s) → rise → shallow scan (10 s) → short surge forward  
YOLOv8-nano detection triggers approach; stop when |x| < 1 m **and** depth aligned; flash lights 2 s, then restart FSM
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

# === Camera intrinsics ===
FX, FY = 273.25, 261.76
CX, CY = 307.89, 153.84

# === ROV dimensions (m) ===
FRONT_WIDTH, FRONT_HEIGHT = 0.33805, 0.251
SIDE_WIDTH,  SIDE_HEIGHT  = 0.4572,  0.251

# === Tunable constants ===
DIVE_DELTA       = 2.0    # metres down from current depth
RISE_DELTA       = 1.0    # metres up from current depth
SCAN_DURATION    = 10.0   # seconds at each scan depth
FORWARD_DURATION = 1.0    # seconds of surge forward
FLASH_DURATION   = 2.0    # seconds lights on
X_TOL            = 1.0    # succeed when |x| < 1 m
DEPTH_TOL        = 0.10   # succeed when |Δdepth| < 0.10 m
DET_TIMEOUT      = 1.0    # seconds before a stale detection is cleared

class FSMMissionNode(Node):
    STATE_INIT, STATE_DIVE2, STATE_SCAN2, STATE_RISE, STATE_SCAN1, STATE_FORWARD, \
    STATE_APPROACH, STATE_FLASH, STATE_FLASHING = range(9)

    def __init__(self):
        super().__init__('fsm_mission_node')
        self.bridge = CvBridge()
        self.model  = YOLO('bluerov2_bluecv/bluerov2_bluecv/yolov8n.pt')

        # ── ROS I/O ────────────────────────────────────────────────────────
        self.create_subscription(Image,   '/camera',   self.image_cb, 10)
        self.create_subscription(Float64, '/depth',    self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(Int16,   '/heading',  self.heading_cb, qos_profile_sensor_data)

        self.pub_manual = self.create_publisher(ManualControl, '/manual_control', 10)
        self.pub_tdepth = self.create_publisher(Float64,        '/target_depth',   10)
        self.pub_lights = self.create_publisher(OverrideRCIn,   '/override_rc',    10)

        # ── State variables ───────────────────────────────────────────────
        self.depth      = None            # latest measured depth (m, positive down)
        self.heading    = None
        self.detection  = False
        self.rov_pos    = None            # [x_m, y_m, z_m] from box_to_pose
        self.t_last_det = 0.0
        self.state      = self.STATE_INIT
        self.t_state    = self.now_sec()  # time we entered current state
        self.dive_start_depth = None      # depth reference captured at INIT

        self.create_timer(0.1, self.loop)  # 10 Hz

    # ── Utility ───────────────────────────────────────────────────────────
    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ── Callbacks ─────────────────────────────────────────────────────────
    def depth_cb(self, msg):   self.depth   = msg.data
    def heading_cb(self, msg): self.heading = msg.data

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        res = self.model.predict(img, conf=0.5, verbose=False)[0]
        box = self.best_box(res.boxes)
        if box is not None:
            pose = self.box_to_pose(box)
            if pose:
                self.rov_pos    = pose
                self.detection  = True
                self.t_last_det = self.now_sec()
                return
        # no valid box → clear flag
        self.detection = False

    # ── Vision helpers ────────────────────────────────────────────────────
    @staticmethod
    def best_box(boxes, min_area=500):
        best, area = None, 0
        for b in boxes:
            x1, y1, x2, y2 = map(int, b.xyxy[0])
            a = (x2 - x1) * (y2 - y1)
            if a >= min_area and a > area:
                best, area = (x1, y1, x2 - x1, y2 - y1), a
        return best

    def box_to_pose(self, box):
        x, y, w, h = box
        if w <= 0 or h <= 0:
            return None
        # pick the face whose z-estimate is more plausible (front or side)
        z_front = FRONT_WIDTH * FX / w
        z_side  = SIDE_WIDTH  * FX / w
        real_w, real_h = (FRONT_WIDTH, FRONT_HEIGHT) if z_front > z_side else (SIDE_WIDTH, SIDE_HEIGHT)
        z = ((real_w * FX) / w + (real_h * FY) / h) / 2.0
        dx = (x + w / 2 - CX)
        dy = (y + h / 2 - CY)
        x_m =  dx * z / FX        # + → right
        y_m = -dy * z / FY        # + → down in image ⇒ deeper
        return [round(x_m, 3), round(y_m, 3), round(z, 3)]

    # ── Finite-state loop ────────────────────────────────────────────────
    def loop(self):
        t_now = self.now_sec()

        # clear stale detection
        if self.detection and (t_now - self.t_last_det) > DET_TIMEOUT:
            self.detection = False

        # guard until depth is available
        if self.depth is None:
            return

        # ── STATE MACHINE ───────────────────────────────────────────────
        if self.state == self.STATE_INIT:
            self.dive_start_depth = self.depth
            target_depth = self.dive_start_depth + DIVE_DELTA
            self.pub_tdepth.publish(Float64(data=target_depth))
            self.transition(self.STATE_DIVE2)

        elif self.state == self.STATE_DIVE2:
            if abs(self.depth - (self.dive_start_depth + DIVE_DELTA)) < DEPTH_TOL:
                self.transition(self.STATE_SCAN2)
            # nothing else – keep descending automatically via depth controller

        elif self.state == self.STATE_SCAN2:
            if self.detection:
                self.transition(self.STATE_APPROACH)
            elif (t_now - self.t_state) > SCAN_DURATION:
                # rise 1 m
                target_depth = max(0.0, self.depth - RISE_DELTA)
                self.pub_tdepth.publish(Float64(data=target_depth))
                self.transition(self.STATE_RISE)

        elif self.state == self.STATE_RISE:
            if abs(self.depth - (self.dive_start_depth + DIVE_DELTA - RISE_DELTA)) < DEPTH_TOL:
                self.transition(self.STATE_SCAN1)

        elif self.state == self.STATE_SCAN1:
            if self.detection:
                self.transition(self.STATE_APPROACH)
            elif (t_now - self.t_state) > SCAN_DURATION:
                # surge forward for FORWARD_DURATION
                self.pub_manual.publish(ManualControl(x=70.0, y=0.0, z=0.0, r=0.0))
                self.transition(self.STATE_FORWARD)

        elif self.state == self.STATE_FORWARD:
            if (t_now - self.t_state) > FORWARD_DURATION:
                self.pub_manual.publish(ManualControl())  # stop motion
                self.transition(self.STATE_SCAN2)

        elif self.state == self.STATE_APPROACH:
            x_m, y_m, _ = self.rov_pos
            # ─ horizontal strafing ─
            mc = ManualControl()
            #mc.y =  500 if x_m >  X_TOL else (-500 if x_m < -X_TOL else 0)
            mc.y =  70.0 if x_m >  X_TOL else (-70.0 if x_m < -X_TOL else 0.0)
            # ─ depth correction ─
            depth_delta = y_m    # +y_m ⇒ object below centre ⇒ we must go deeper
            self.pub_tdepth.publish(Float64(data=self.depth + depth_delta))
            self.pub_manual.publish(mc)

            # success criteria: |x|<1 m and depth aligned (|Δdepth|<DEPTH_TOL)
            if abs(x_m) < X_TOL and abs(depth_delta) < DEPTH_TOL:
                self.pub_manual.publish(ManualControl())  # full stop
                self.transition(self.STATE_FLASH)

        elif self.state == self.STATE_FLASH:
            # lights on (all channels max for simplicity)
            rc = OverrideRCIn(); rc.channels = [2000]*8
            self.pub_lights.publish(rc)
            self.transition(self.STATE_FLASHING)

        elif self.state == self.STATE_FLASHING:
            if (t_now - self.t_state) > FLASH_DURATION:
                rc = OverrideRCIn(); rc.channels = [1000]*8
                self.pub_lights.publish(rc)
                # ─ restart mission from scratch ─
                self.transition(self.STATE_INIT)

    # ── helper to set new state ──────────────────────────────────────────
    def transition(self, new_state):
        self.state   = new_state
        self.t_state = self.now_sec()

# ── ROS entrypoint ────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = FSMMissionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


