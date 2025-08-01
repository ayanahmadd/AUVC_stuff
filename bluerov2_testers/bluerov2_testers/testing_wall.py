#!/usr/bin/env python3
"""
BlueROV2 – pure-movement demo
 • Dive 2 m below current depth
 • Rise 1 m
 • Surge forward for 1 s
 • Flash lights 2 s
 • Restart sequence
"""

import rclpy, time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64
from mavros_msgs.msg import ManualControl, OverrideRCIn

# ── Tunables ─────────────────────────────────────────────────────────────
DIVE_DELTA       = 2.0     # m down
RISE_DELTA       = 1.0     # m up (from deep point)
DEPTH_TOL        = 0.10    # m tolerance
FORWARD_DURATION = 1.0     # s
FLASH_DURATION   = 2.0     # s
LOOP_HZ          = 10      # control loop frequency

class MovementFSM(Node):
    STATE_INIT, STATE_DIVE, STATE_RISE, STATE_FORWARD, STATE_FLASH, STATE_FLASHING = range(6)

    def __init__(self):
        super().__init__('movement_fsm')
        self.depth_sub = self.create_subscription(FluidPressure, '/rov1/pressure', self.pressure_cb, 10)
        self.pub_tdepth      = self.create_publisher(Float64,  '/target_depth', 10)
        self.pub_manual      = self.create_publisher(ManualControl, '/rov1/manual_control', 10)
        self.pub_lights      = self.create_publisher(OverrideRCIn, '/rov1/override_rc', 10)

        self.depth      = None
        self.start_depth= None
        self.state      = self.STATE_INIT
        self.t_state    = self.now()

        self.create_timer(1.0 / LOOP_HZ, self.loop)

    # ── helpers ──────────────────────────────────────────────────────────
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def pressure_cb(self, msg: FluidPressure):
        # Convert pressure (Pa) to depth (meters)
        P = msg.fluid_pressure      # Pascals
        P0 = 101325                 # Surface pressure (sea level)
        rho = 997                   # Density of freshwater (kg/m^3)
        g = 9.80665                 # Gravity (m/s^2)

        self.depth = (P - P0) / (rho * g)

        # Optional: clamp small negative depths to 0
        if self.depth < 0:
            self.depth = 0.0


    def set_target_depth(self, d):
        self.pub_tdepth.publish(Float64(data=d))

    def stop_motion(self):
        self.pub_manual.publish(ManualControl())          # zeroed fields

    def surge_forward(self):
        self.pub_manual.publish(ManualControl(x=1000))    # +x thrust

    def lights_on(self, on: bool):
        pwm = 2000 if on else 1000
        rc  = OverrideRCIn(); rc.channels = [pwm] * 8
        self.pub_lights.publish(rc)

    # ── main loop ────────────────────────────────────────────────────────
    def loop(self):
        if self.depth is None:
            return                                            # wait for first depth

        t = self.now()

        if self.state == self.STATE_INIT:
            self.start_depth = self.depth
            self.set_target_depth(self.start_depth + DIVE_DELTA)
            self.switch(self.STATE_DIVE)

        elif self.state == self.STATE_DIVE:
            if abs(self.depth - (self.start_depth + DIVE_DELTA)) < DEPTH_TOL:
                self.set_target_depth(self.start_depth + DIVE_DELTA - RISE_DELTA)
                self.switch(self.STATE_RISE)

        elif self.state == self.STATE_RISE:
            if abs(self.depth - (self.start_depth + DIVE_DELTA - RISE_DELTA)) < DEPTH_TOL:
                self.surge_forward()
                self.switch(self.STATE_FORWARD)

        elif self.state == self.STATE_FORWARD:
            if t - self.t_state > FORWARD_DURATION:
                self.stop_motion()
                self.lights_on(True)
                self.switch(self.STATE_FLASH)

        elif self.state == self.STATE_FLASH:
            self.switch(self.STATE_FLASHING)                  # timer starts

        elif self.state == self.STATE_FLASHING:
            if t - self.t_state > FLASH_DURATION:
                self.lights_on(False)
                self.switch(self.STATE_INIT)                  # restart sequence

    def switch(self, new_state):
        self.state   = new_state
        self.t_state = self.now()

def main():
    rclpy.init()
    node = MovementFSM()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

