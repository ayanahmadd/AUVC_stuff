#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray, Int16
from mavros_msgs.msg import ManualControl, OverrideRCIn
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import math
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

"""
Dive → Back-up → (Rotate→Hold→Scan)×∞
             ↘ if detect
               Approach (prop-drive + camera tilt)
               ↘ if close enough
                 Hold & confirm
                    ↘ success → Flash lights → Idle
                    ↘ failure → restart whole sequence
"""

# === Camera intrinsics
FX, FY = 273.25, 261.76
CX, CY = 307.89, 153.84

# === ROV dimensions (meters)
FRONT_WIDTH, FRONT_HEIGHT = 0.33805, 0.251
SIDE_WIDTH, SIDE_HEIGHT   = 0.4572,  0.251

class BasicCVMission(Node):
    def __init__(self):
        super().__init__('basic_cv_mission')

        # — Callback groups for async execution —
        self.fast_group = ReentrantCallbackGroup()
        self.slow_group = ReentrantCallbackGroup()

        # — Publishers —
        self.pub_target_depth = self.create_publisher(Float64,    'target_depth',    10)
        self.pub_manual       = self.create_publisher(ManualControl, '/manual_control', 10)
        self.pub_lights       = self.create_publisher(OverrideRCIn, 'override_rc',     10)
        # — Camera tilt publisher —
        self.pub_camera_tilt = self.create_publisher(Float64, 'camera_tilt', 10)

        # — CV Detector setup —
        self.bridge = CvBridge()
        self.model  = YOLO("bluerov2_bluecv/bluerov2_bluecv/yolov8n.pt")
        self.create_subscription(Image, 'camera', self.image_cb, 10, callback_group=self.slow_group)

        # — Subscribers for depth & heading —
        self.create_subscription(Float64,            'depth',    self.depth_cb,   10, callback_group=self.fast_group)
        self.create_subscription(Int16,              '/heading', self.heading_cb, 10, callback_group=self.fast_group)

        # — State & timing —
        self.state             = 0
        self.start_time        = self.get_clock().now()
        self.state6_start_time = None
        # — Timestamp for “hold‑and‑rescan” confirmation —
        self.rescan_start_time = None

        # — Sensor storage —
        self.current_depth   = None
        self.rov_position    = None   # [x_m, y_m, z_m]
        self.initial_rov_position = None
        self.current_heading = None

        # — Scan parameters —
        self.SCAN_STEP    = 45.0    # degrees
        self.SCAN_TOL     =  2.0    # ± tolerance
        self.SCAN_HOLD    =  2.0    # seconds to scan
        self.SPIN_RATE    = 60.0    # °/s
        self.start_heading    = None
        self.next_heading     = None
        self.hold_start       = None
        self.cw_offsets       = [i * self.SCAN_STEP for i in range(1, 9)]
        self.scanning_forward = True
        self.offset_idx       = 0

        # — Inference throttle (seconds) —
        self.INFERENCE_INTERVAL = 0.5
        self.last_inference = self.get_clock().now()

        # — Dive & back-up constants —
        self.DIVE_DEPTH    = 2.0     # m
        self.DIVE_TOL      = 0.1     # m
        self.DIVE_SETTLE   = 2.0     # s
        self.BACK_SPEED    = 1.0     # m/s
        self.BACK_DISTANCE = 2.0     # m
        self.backed_distance = 0.0
        self.last_back_time  = None

        # — Main loop @10 Hz —
        self.create_timer(0.1, self.timer_cb, callback_group=self.fast_group)
        self.get_logger().info("BasicCVMission started")

    #
    # — Sensor callbacks —
    #
    def depth_cb(self, msg: Float64):
        self.current_depth = msg.data

    def heading_cb(self, msg: Int16):
        self.current_heading = float(msg.data)

    def angle_diff(self, target: float, current: float) -> float:
        """Smallest signed difference from current to target."""
        a = (target - current + 180.0) % 360.0 - 180.0
        return a

    def turn_lights(self, level: int):
        cmd = OverrideRCIn()
        cmd.channels = [OverrideRCIn.CHAN_NOCHANGE]*10
        cmd.channels[8] = 1000 + level*10
        cmd.channels[9] = 1000 + level*10
        self.pub_lights.publish(cmd)

    #
    # — Image callback: run YOLO, pick best box, estimate pose, set rov_position —
    #
    def image_cb(self, msg: Image):
        now = self.get_clock().now()
        if (now - self.last_inference).nanoseconds * 1e-9 < self.INFERENCE_INTERVAL:
            return
        self.last_inference = now

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        small = cv2.resize(img, (320, 240))
        results = self.model.predict(small, conf=0.5)[0]
        # pick largest box
        best = self.select_optimal_detection(results.boxes)
        if best:
            x, y, w, h = best
            pose = self.estimate_3d_pose((x, y, w, h), img.shape[:2])
            if pose:
                x_m, y_m, z_m, _ = pose  # ignore angle
                self.rov_position = [x_m, y_m, z_m]
                # debug log:
                self.get_logger().info(f"→ Detected ROV at X={x_m:.2f}m, Y={y_m:.2f}m, Z={z_m:.2f}m")
            else:
                self.rov_position = None
        else:
            self.rov_position = None

    def select_optimal_detection(self, boxes, min_area=500):
        best, max_area = None, 0
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            w, h = x2 - x1, y2 - y1
            area = w * h
            if area >= min_area and area > max_area:
                best, max_area = (x1, y1, w, h), area
        return best

    def estimate_3d_pose(self, box, image_shape, fx=FX, fy=FY, cx=CX, cy=CY):
        x, y, w, h = box
        if w <= 0 or h <= 0:
            return None

        # decide orientation
        z_front = (FRONT_WIDTH * fx) / w
        z_side  = (SIDE_WIDTH * fx) / w
        if z_front > z_side:
            real_w, real_h = FRONT_WIDTH, FRONT_HEIGHT
        else:
            real_w, real_h = SIDE_WIDTH, SIDE_HEIGHT

        z_w = (real_w * fx) / w
        z_h = (real_h * fy) / h
        z_m = (z_w + z_h) / 2.0

        cx_img = x + w/2
        cy_img = y + h/2
        dx = cx_img - cx
        dy = cy_img - cy

        x_m = (dx * z_m) / fx
        y_m = -(dy * z_m) / fy
        return round(x_m,4), round(y_m,4), round(z_m,4), 0.0

    #
    # — Main state machine timer —
    #
    def timer_cb(self):
        now = self.get_clock().now()

        # State 0 → dive
        if self.state == 0:
            self.pub_target_depth.publish(Float64(data=self.DIVE_DEPTH))
            self.start_time = now
            self.state = 1
            self.get_logger().info("→ State 0: Diving to 2 m")

        # State 1 → wait settle
        elif self.state == 1:
            if (self.current_depth is not None
                and abs(self.current_depth - self.DIVE_DEPTH) <= self.DIVE_TOL
                and (now - self.start_time).nanoseconds * 1e-9 >= self.DIVE_SETTLE):
                self.get_logger().info("→ Depth reached")
                self.state = 2
                self.backed_distance = 0.0
                self.last_back_time  = now

        # State 2 → back up
        elif self.state == 2:
            dt = (now - self.last_back_time).nanoseconds * 1e-9
            self.last_back_time = now
            self.backed_distance += abs(self.BACK_SPEED) * dt
            if self.backed_distance < self.BACK_DISTANCE:
                self.pub_manual.publish(ManualControl(x=self.BACK_SPEED, y=0.0, z=0.0, r=0.0))
            else:
                self.pub_manual.publish(ManualControl(x=0.0, y=0.0, z=0.0, r=0.0))
                self.get_logger().info("→ Backed up {BACKUP_DISTANCE} m")
                self.state = 3

        # State 3 → init scan
        elif self.state == 3:
            if self.current_heading is not None:
                self.start_heading    = self.current_heading
                self.scanning_forward = True
                self.offset_idx       = 3
                self.next_heading     = (self.start_heading + self.cw_offsets[self.offset_idx]) % 360.0
                self.hold_start       = None
                self.state            = 4
                self.get_logger().info(f"→ State 3: first scan at {self.next_heading:.1f}°")

        # State 4 → scan or detect
        elif self.state == 4:
            if self.rov_position is not None:
                # detected—go approach
                self.initial_rov_position = list(self.rov_position)
                self.state = 5
                self.get_logger().info("→ ROV detected, approaching…")
                return

            if self.current_heading is None:
                # spin until we get heading
                self.pub_manual.publish(ManualControl(x=0.0, y=0.0, z=0.0, r=self.SPIN_RATE))
                return

            diff = self.angle_diff(self.next_heading, self.current_heading)
            if abs(diff) <= self.SCAN_TOL:
                if self.hold_start is None:
                    self.hold_start = now
                    self.pub_manual.publish(ManualControl(x=0.0, y=0.0, z=0.0, r=0.0))
                    self.get_logger().info(f"→ Holding at {self.next_heading:.1f}°")
                elif (now - self.hold_start).nanoseconds * 1e-9 >= self.SCAN_HOLD:
                    self.offset_idx += 1
                    if self.offset_idx >= len(self.cw_offsets):
                        self.scanning_forward = not self.scanning_forward
                        self.offset_idx = 0
                    offsets = self.cw_offsets if self.scanning_forward else [-o for o in self.cw_offsets]
                    self.next_heading = (self.start_heading + offsets[self.offset_idx]) % 360.0
                    self.hold_start = None
                    self.get_logger().info(f"→ Next scan at {self.next_heading:.1f}°")
            else:
                rate = self.SPIN_RATE if diff > 0 else -self.SPIN_RATE
                self.pub_manual.publish(ManualControl(x=0.0, y=0.0, z=0.0, r=rate))

        # State 5 → approach & flash
        elif self.state == 5:
            x, y, z = self.rov_position
            # If ROV is above/below, tilt camera to face it
            vertical_tolerance = 0.1  # meters
            y_m = y  # current vertical offset from rov_position
            z_m = z  # current forward distance
            if abs(y_m) > vertical_tolerance:
                # compute tilt angle (positive tilts up)
                tilt_angle = math.degrees(math.atan2(y_m, z_m))
                self.pub_camera_tilt.publish(Float64(data=tilt_angle))
                self.get_logger().info(f"→ Tilting camera to {tilt_angle:.1f}° to align target")
                return

            # proportional drive
            raw_fwd = (z - 1.0) * 500.0
            raw_yaw = x * 500.0
            cf = max(min(raw_fwd, 600.0), -600.0) / 600.0
            cy = max(min(raw_yaw, 600.0), -600.0) / 600.0
            self.pub_manual.publish(ManualControl(x=cf, y=0.0, z=0.0, r=cy))

            # ── Reached predicted spot: stop & hold for a fresh re‑scan ──
            x0, y0, z0 = self.initial_rov_position
            if (abs(z - z0) <= 0.05 and abs(x) <= 1.0 and abs(y - y0) <= 0.05):
                # Freeze thrusters and start a timed re‑scan
                self.pub_manual.publish(ManualControl(x=0.0, y=0.0, z=0.0, r=0.0))
                self.rescan_start_time = now
                self.state = 55      # new “re‑scan confirmation” state
                self.get_logger().info("→ Holding position; rescanning to confirm ≤ 1 m")
                return

        # State 55 → hold & re‑scan to confirm we are truly ≤ 1 m
        elif self.state == 55:
            # Wait at least 1 s to gather fresh detections
            if (now - self.rescan_start_time).nanoseconds * 1e-9 < 1.0:
                return

            # If detector still sees the ROV and z ≤ 1 m, flash; otherwise restart mission
            if self.rov_position is not None and self.rov_position[2] <= 1.0:
                self.turn_lights(100)
                self.state6_start_time = now
                self.state = 6
                self.get_logger().info("→ Confirmed ≤ 1 m; flashing lights!")
            else:
                self.get_logger().info("→ Re‑scan failed; restarting full search cycle")
                self.rov_position = None
                self.backed_distance = 0.0
                self.state = 0          # jump back to the dive/back‑up sequence
                self.start_time = now   # reset timer for State 0
                return

        # State 6 → lights off
        elif self.state == 6:
            if (now - self.state6_start_time).nanoseconds * 1e-9 >= 3.0:
                self.turn_lights(0)
                self.state = 7
                self.get_logger().info("→ Lights off, mission complete")

        # State 7 → idle
        elif self.state == 7:
            pass

def main():
    rclpy.init()
    node = BasicCVMission()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()