#!/usr/bin/env python3
"""
Combined FSM + Depth-PID mission node for BlueROV2
  • Initial dive → deep scan → rise → shallow scan → forward surge  
  • YOLOv8‐nano detection → approach → flash lights → restart  
  • FluidPressure → depth conversion  
  • Single ManualControl publisher arbitrates x/y from FSM and z from PID
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, FluidPressure
from std_msgs.msg import Int16
from mavros_msgs.msg import ManualControl, OverrideRCIn
from cv_bridge import CvBridge
from ultralytics import YOLO
from bluerov2_controllers.PIDController import PIDController
# ── Camera intrinsics ───────────────────────────────────────────
FX, FY = 273.25, 261.76
CX, CY = 307.89, 153.84
# ── ROV dimensions (m) ──────────────────────────────────────────
FRONT_WIDTH, FRONT_HEIGHT = 0.33805, 0.251
SIDE_WIDTH,  SIDE_HEIGHT  = 0.4572,  0.251
# ── Tunable constants ───────────────────────────────────────────
DIVE_DELTA       = 2.0    # m below start
RISE_DELTA       = 1.0    # m up from deep
SCAN_DURATION    = 10.0   # s
FORWARD_DURATION = 1.0    # s
FLASH_DURATION   = 2.0    # s
X_TOL            = 1.0    # m
DEPTH_TOL        = 0.10   # m
DET_TIMEOUT      = 1.0    # s
# ── Fluid parameters ───────────────────────────────────────────
ATM_PRESSURE = 101325.0    # Pa
RHO          = 1025.0      # kg/m³ (sea water)
G            = 9.80665     # m/s²
class FSMPIDMissionNode(Node):
    # FSM states
    (STATE_INIT, STATE_DIVE2, STATE_SCAN2, STATE_RISE,
     STATE_SCAN1, STATE_FORWARD, STATE_APPROACH,
     STATE_FLASH, STATE_FLASHING) = range(9)
    def __init__(self):
        super().__init__('fsm_pid_mission_node')
        self.bridge = CvBridge()
        self.model  = YOLO('bluerov2_bluecv/bluerov2_bluecv/yolov8n.pt')
        # single ManualControl message we’ll reuse/publish
        self.manual_cmd = ManualControl()
        # PID depth controller
        self.pid = PIDController(kp=70.0, ki=2.5, kd=5.0, setpoint=0.0, dt=0.1)
        # State
        self.depth            = None      # m
        self.heading          = None      # deg
        self.detection        = False
        self.rov_pos          = None      # [x, y, z] in m
        self.t_last_det       = 0.0
        self.state            = self.STATE_INIT
        self.t_state          = self.now_sec()
        self.dive_start_depth = None
        # Publishers
        self.pub_manual = self.create_publisher(
            ManualControl, '/manual_control', 10
        )
        self.pub_lights = self.create_publisher(
            OverrideRCIn,  '/override_rc',    10
        )
        # Subscriptions
        # — vision in its own callback group so .predict() can run in parallel
        vision_cg = ReentrantCallbackGroup()
        self.create_subscription(
            Image, '/camera', self.image_cb, 10,
            callback_group=vision_cg
        )
        # — pressure → depth
        self.create_subscription(
            FluidPressure, '/fluid_pressure',
            self.pressure_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Int16, '/heading',
            self.heading_cb, qos_profile_sensor_data
        )
        # 10 Hz combined FSM + PID loop
        self.create_timer(0.1, self.loop)
        self.get_logger().info("FSMPIDMissionNode started")
    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9
    # ── Callbacks ───────────────────────────────────────────────────
    def pressure_cb(self, msg: FluidPressure):
        # convert Pa → depth (m)
        self.depth = (msg.fluid_pressure - ATM_PRESSURE) / (RHO * G)
    def heading_cb(self, msg: Int16):
        self.heading = msg.data
    def image_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        res = self.model.predict(img, conf=0.5, verbose=False)[0]
        box = self.best_box(res.boxes)
        if box:
            pose = self.box_to_pose(box)
            if pose:
                self.rov_pos    = pose
                self.detection  = True
                self.t_last_det = self.now_sec()
                return
        # no valid box → clear flag
        self.detection = False
    # ── Vision helpers ─────────────────────────────────────────────
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
        # estimate Z from front‐face vs side‐face
        zf = FRONT_WIDTH * FX / w
        zs = SIDE_WIDTH  * FX / w
        real_w, real_h = (FRONT_WIDTH, FRONT_HEIGHT) if zf > zs else (SIDE_WIDTH, SIDE_HEIGHT)
        z = ((real_w * FX) / w + (real_h * FY) / h) / 2.0
        dx = (x + w/2 - CX)
        dy = (y + h/2 - CY)
        xm =  dx * z / FX
        ym = -dy * z / FY
        return [round(xm,3), round(ym,3), round(z,3)]
    # ── Main loop: FSM + PID merge ───────────────────────────────────
    def loop(self):
        t = self.now_sec()
        if self.depth is None:
            return
        # clear stale detection
        if self.detection and (t - self.t_last_det) > DET_TIMEOUT:
            self.detection = False
        # FSM transitions + lateral commands
        if self.state == self.STATE_INIT:
            # mark reference and dive
            self.dive_start_depth = self.depth
            self.pid.setpoint = self.depth + DIVE_DELTA
            self.manual_cmd = ManualControl()  # zero x/y until needed
            self.transition(self.STATE_DIVE2)
        elif self.state == self.STATE_DIVE2:
            if abs(self.depth - self.pid.setpoint) < DEPTH_TOL:
                self.transition(self.STATE_SCAN2)
        elif self.state == self.STATE_SCAN2:
            if self.detection:
                self.transition(self.STATE_APPROACH)
            elif (t - self.t_state) > SCAN_DURATION:
                # go up one metre
                new_sp = max(0.0, self.depth - RISE_DELTA)
                self.pid.setpoint = new_sp
                self.transition(self.STATE_RISE)
        elif self.state == self.STATE_RISE:
            if abs(self.depth - self.pid.setpoint) < DEPTH_TOL:
                self.transition(self.STATE_SCAN1)
        elif self.state == self.STATE_SCAN1:
            if self.detection:
                self.transition(self.STATE_APPROACH)
            elif (t - self.t_state) > SCAN_DURATION:
                # surge forward
                self.manual_cmd = ManualControl(x=70.0, y=0.0, z=0.0, r=0.0)
                self.transition(self.STATE_FORWARD)
        elif self.state == self.STATE_FORWARD:
            if (t - self.t_state) > FORWARD_DURATION:
                self.manual_cmd = ManualControl()  # stop
                self.transition(self.STATE_SCAN2)
        elif self.state == self.STATE_APPROACH:
            x_m, y_m, _ = self.rov_pos
            mc = ManualControl()
            # strafe until |x|< tol
            #mc.y =  500 if x_m >  X_TOL else (-500 if x_m < -X_TOL else 0)
            mc.y =  70.0 if x_m >  X_TOL else (-70.0 if x_m < -X_TOL else 0.0)
            self.manual_cmd = mc
            # adjust depth setpoint toward object
            self.pid.setpoint = self.depth + y_m
            if abs(x_m) < X_TOL and abs(y_m) < DEPTH_TOL:
                self.manual_cmd = ManualControl()
                self.transition(self.STATE_FLASH)
        elif self.state == self.STATE_FLASH:
            rc = OverrideRCIn()
            rc.channels = [2000]*8
            self.pub_lights.publish(rc)
            self.transition(self.STATE_FLASHING)
        elif self.state == self.STATE_FLASHING:
            if (t - self.t_state) > FLASH_DURATION:
                rc = OverrideRCIn()
                rc.channels = [1000]*8
                self.pub_lights.publish(rc)
                self.transition(self.STATE_INIT)
        # ── PID vertical control & publish combined command ───────────
        thrust = self.pid.compute(self.depth)
        # original PID node inverted sign for z
        self.manual_cmd.z = -thrust
        self.pub_manual.publish(self.manual_cmd)
    def transition(self, new_state: int):
        self.state   = new_state
        self.t_state = self.now_sec()
def main():
    rclpy.init()
    node = FSMPIDMissionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()