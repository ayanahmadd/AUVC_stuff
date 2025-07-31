#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl, OverrideRCIn
from std_msgs.msg import Float64, Float64MultiArray, Int16
import time

class FSMMissionMode(Node):
    STATE_INIT, STATE_SCAN, STATE_DIVE, STATE_TURN_CW, STATE_RISE, \
    STATE_TURN_CCW, STATE_CHECK_SURFACE, STATE_MOVE_TO_TAG, \
    STATE_RESCAN, STATE_FLASH = range(10)

    def __init__(self):
        super().__init__('fsm_mission_mode')

        # Movement publisher using ManualControl
        self.pub_manual = self.create_publisher(ManualControl, 'manual_control', 10)
        # Lights override
        self.pub_lights = self.create_publisher(OverrideRCIn, 'override_rc', 10)

        # Subscribers
        self.create_subscription(Float64,           'depth',              self.depth_cb,   10)
        self.create_subscription(Float64MultiArray, 'apriltag/detection', self.detect_cb,  10)
        self.create_subscription(Int16,             '/heading',           self.heading_cb, 10)

        # FSM state & data
        self.state = self.STATE_INIT
        self.initial_heading = None
        self.target_heading  = None
        self.turn_start_heading = None
        self.scan_start_time = None
        self.current_depth   = 0.0
        self.current_heading = None
        self.detection       = False
        self.tag_pose        = [0.0,0.0,0.0]
        self.dive_target     = None
        self.rise_target     = None

        # Loop at 10Hz
        self.timer = self.create_timer(0.1, self.fsm_loop)
        self.get_logger().info('FSM manual-control mode initialized')

    def depth_cb(self, msg: Float64):
        self.current_depth = msg.data

    def heading_cb(self, msg: Int16):
        self.current_heading = msg.data

    def detect_cb(self, msg: Float64MultiArray):
        self.detection = True
        self.tag_pose = list(msg.data)[:3]

    def publish_manual(self, x=0.0, y=0.0, z=0.0, r=0.0):
        """
        Publish ManualControl with x,y,z,r in the same units as sample (e.g., ±100).
        We'll scale from normalized [-1..1] inputs by 100.
        """
        cmd = ManualControl()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.x = float(x * 100)
        cmd.y = float(y * 100)
        cmd.z = float(z * 100)
        cmd.r = float(r * 100)
        # leave other fields default
        self.pub_manual.publish(cmd)

    def flash_lights(self, duration=2.0):
        blink = OverrideRCIn()
        blink.channels = [2000]*8
        self.pub_lights.publish(blink)
        time.sleep(duration)
        blink.channels = [1000]*8
        self.pub_lights.publish(blink)

    def fsm_loop(self):
        now = time.time()
        # detection preempt
        if self.detection and self.state not in (self.STATE_MOVE_TO_TAG, self.STATE_FLASH):
            self.publish_manual(0.0,0.0,0.0,0.0)
            self.state = self.STATE_MOVE_TO_TAG
            self.get_logger().info('Tag detected → MOVE_TO_TAG')
            return

        # INIT: 180° turn
        if self.state == self.STATE_INIT:
            if self.initial_heading is None and self.current_heading is not None:
                self.initial_heading = self.current_heading
                self.target_heading  = (self.initial_heading + 180) % 360
                self.get_logger().info(f'INIT → rotate to {self.target_heading}°')
            if self.current_heading is not None:
                err = ((self.target_heading - self.current_heading + 540) % 360) - 180
                if abs(err) > 5:
                    self.publish_manual(r=0.5 if err>0 else -0.5)
                else:
                    self.publish_manual(0.0,0.0,0.0,0.0)
                    self.state = self.STATE_SCAN
                    self.scan_start_time = now
                    self.get_logger().info('INIT done → SCAN')

        # SCAN: spin 5s
        elif self.state == self.STATE_SCAN:
            self.publish_manual(r=0.2)
            if now - self.scan_start_time > 5.0:
                self.publish_manual(0.0,0.0,0.0,0.0)
                self.dive_target = self.current_depth + 3.0
                self.state = self.STATE_DIVE
                self.get_logger().info(f'No tag → DIVE to {self.dive_target:.2f}m')

        # DIVE
        elif self.state == self.STATE_DIVE:
            if self.current_depth < self.dive_target - 0.1:
                self.publish_manual(z=-0.3)
            else:
                self.publish_manual(0.0,0.0,0.0,0.0)
                self.turn_start_heading = self.current_heading
                self.state = self.STATE_TURN_CW
                self.get_logger().info('Reached dive → TURN_CW')

        # TURN_CW
        elif self.state == self.STATE_TURN_CW:
            delta = (self.current_heading - self.turn_start_heading + 360) % 360
            if delta < 350:
                self.publish_manual(r=0.5)
            else:
                self.publish_manual(0.0,0.0,0.0,0.0)
                self.rise_target = self.current_depth - 1.0
                self.state = self.STATE_RISE
                self.get_logger().info('TURN_CW → RISE')

        # RISE
        elif self.state == self.STATE_RISE:
            if self.current_depth > self.rise_target + 0.1:
                self.publish_manual(z=0.3)
            else:
                self.publish_manual(0.0,0.0,0.0,0.0)
                self.turn_start_heading = self.current_heading
                self.state = self.STATE_TURN_CCW
                self.get_logger().info('RISE → TURN_CCW')

        # TURN_CCW
        elif self.state == self.STATE_TURN_CCW:
            delta = (self.turn_start_heading - self.current_heading + 360) % 360
            if delta < 350:
                self.publish_manual(r=-0.5)
            else:
                self.publish_manual(0.0,0.0,0.0,0.0)
                self.state = self.STATE_CHECK_SURFACE
                self.get_logger().info('TURN_CCW → CHECK_SURFACE')

        # CHECK_SURFACE
        elif self.state == self.STATE_CHECK_SURFACE:
            if self.current_depth <= 0.2:
                self.dive_target = self.current_depth + 3.0
                self.state = self.STATE_DIVE
                self.get_logger().info('At surface → DIVE')

        # MOVE_TO_TAG
        elif self.state == self.STATE_MOVE_TO_TAG:
            dx, dy, dz = self.tag_pose
            x_vel =  0.3 if dz >  0.1 else -0.3 if dz < -0.1 else 0.0
            y_vel =  0.3 if dx >  0.1 else -0.3 if dx < -0.1 else 0.0
            z_vel =  0.3 if dy >  0.1 else -0.3 if dy < -0.1 else 0.0
            self.get_logger().info(f'MOVE_TO_TAG velocities → x={x_vel:.2f}, y={y_vel:.2f}, z={z_vel:.2f}')

            self.publish_manual(x_vel, y_vel, z_vel, 0.0)
            if abs(dx) < 0.1 and abs(dy) < 0.1 and abs(dz) < 0.1:
                self.publish_manual(0.0,0.0,0.0,0.0)
                self.state = self.STATE_RESCAN
                self.scan_start_time = now
                self.detection = False
                self.get_logger().info('MOVE_TO_TAG → RESCAN')

        # RESCAN
        elif self.state == self.STATE_RESCAN:
            self.publish_manual(r=0.2)
            if self.detection:
                self.publish_manual(0.0,0.0,0.0,0.0)
                self.state = self.STATE_FLASH
                self.get_logger().info('RESCAN → FLASH')

        # FLASH
        elif self.state == self.STATE_FLASH:
            self.publish_manual(0.0,0.0,0.0,0.0)
            self.flash_lights()
            self.detection = False
            self.state = self.STATE_SCAN
            self.scan_start_time = now
            self.get_logger().info('FLASH → SCAN')


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