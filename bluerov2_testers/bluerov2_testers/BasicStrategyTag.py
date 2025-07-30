#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Int16
from mavros_msgs.msg import ManualControl, OverrideRCIn
import math


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


       # Time when entering state 6 (flash idle)
       self.state6_start_time = None


       # Sensor readings
       self.current_depth   = None
       self.tag_position    = None  # [x, y, z]
       self.current_heading = None


       # Tilt‐wiggle parameters
       self.TILT_DOWN      = -30.0   # degrees
       self.TILT_UP        =  30.0   # degrees
       self.TILT_PERIOD    =   2.0   # seconds between toggles
       self.last_tilt_time = None
       self.current_tilt   = self.TILT_UP


       # Spin parameters
       self.spin_start_head = None
       self.spin_dir        = 1       # 1 = CW, -1 = CCW
       self.SPIN_RATE      = 200.0
       self.SPIN_THRESH    = 330.0    # degrees before reversing


       # Movement constants
       self.DIVE_DEPTH     = 2.0     # m
       self.DIVE_TOL       = 0.1     # m
       self.DIVE_SETTLE    = 2.0     # s after reaching depth
       self.BACK_SPEED     = -0.2    # m/s
       self.BACK_DISTANCE  = 2.0     # m
       self.BACK_TIME      = self.BACK_DISTANCE / abs(self.BACK_SPEED)


       # Main timer at 10 Hz
       self.create_timer(0.1, self.timer_cb)
       self.get_logger().info("🚀 Mission node started")


   def depth_cb(self, msg: Float64):
       self.current_depth = msg.data


   def detect_cb(self, msg: Float64MultiArray):
       if len(msg.data) >= 4:
           # msg.data = [id, x, y, z, ...]
           self.tag_position = msg.data[1:4]


   def heading_cb(self, msg: Int16):
       self.current_heading = float(msg.data)


   def turn_lights(self, level: int):
       """ level: 0 (off) → 100 (max) """
       cmd = OverrideRCIn()
       cmd.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
       cmd.channels[8] = 1000 + level * 10
       cmd.channels[9] = 1000 + level * 10
       self.pub_lights.publish(cmd)


   def timer_cb(self):
       now = self.get_clock().now()


       # State 0: dive to 2 m
       if self.state == 0:
           self.get_logger().info("→ State 0: Diving to 2 m")
           self.pub_target_depth.publish(Float64(data=self.DIVE_DEPTH))
           self.start_time = now
           self.state = 1


       # State 1: wait until at depth + settle
       elif self.state == 1:
           if (self.current_depth is not None
               and abs(self.current_depth - self.DIVE_DEPTH) <= self.DIVE_TOL
               and (now - self.start_time).nanoseconds * 1e-9 >= self.DIVE_SETTLE):
               self.get_logger().info("→ Reached dive depth")
               self.start_time = now
               self.state = 2


       # State 2: back up 2 m
       elif self.state == 2:
           elapsed = (now - self.start_time).nanoseconds * 1e-9
           if elapsed <= self.BACK_TIME:
               self.pub_manual.publish(
                   ManualControl(x=self.BACK_SPEED, y=0.0, z=0.0, r=0.0)
               )
           else:
               self.get_logger().info("→ Finished backing up")
               self.state = 3
               self.pub_manual.publish(ManualControl(x=0.0, y=0.0, z=0.0, r=0.0))


       # State 3: initial camera tilt up
       elif self.state == 3:
           self.get_logger().info(f"→ State 3: Tilting camera to {self.TILT_UP}°")
           self.pub_camera_tilt.publish(Float64(data=self.TILT_UP))
           self.last_tilt_time = now
           self.current_tilt   = self.TILT_UP
           self.state = 4


       # State 4: spin & wiggle camera until AprilTag seen
       elif self.state == 4:
           # 4a) spin in place
           mc = ManualControl(x=0.0, y=0.0, z=0.0)
           if self.current_heading is not None:
               if self.spin_start_head is None:
                   self.spin_start_head = self.current_heading
               delta = (self.current_heading - self.spin_start_head + 360.0) % 360.0
               if delta >= self.SPIN_THRESH:
                   self.spin_dir *= -1
                   self.spin_start_head = self.current_heading
               mc.r = self.SPIN_RATE * self.spin_dir
           else:
               mc.r = self.SPIN_RATE
           self.pub_manual.publish(mc)


           # 4b) wiggle tilt every TILT_PERIOD
           if (now - self.last_tilt_time).nanoseconds * 1e-9 >= self.TILT_PERIOD:
               self.current_tilt = (
                   self.TILT_UP if self.current_tilt == self.TILT_DOWN
                   else self.TILT_DOWN
               )
               self.pub_camera_tilt.publish(Float64(data=self.current_tilt))
               self.get_logger().info(f"→ Wiggle tilt to {self.current_tilt}°")
               self.last_tilt_time = now


           # 4c) if tag appears, transition
           if self.tag_position is not None:
               self.get_logger().info("→ AprilTag detected! Beginning approach")
               self.state = 5


       # State 5: approach tag, flash at <1 m
       elif self.state == 5:
           x, y, z = self.tag_position


           # Compute raw control efforts
           raw_forward = (z - 1.0) * 500.0
           raw_yaw     = x         * 500.0
           # Clamp to ±600
           clamped_fwd = max(min(raw_forward, 600.0), -600.0)
           clamped_yaw = max(min(raw_yaw,     600.0), -600.0)
           # Normalize into [-1.0, 1.0] for ManualControl
           norm_fwd = clamped_fwd / 600.0
           norm_yaw = clamped_yaw / 600.0
           self.pub_manual.publish(ManualControl(x=norm_fwd, y=0.0, z=0.0, r=norm_yaw))


           # Flash lights when within 1 m
           if z < 1.0:
               self.get_logger().info("→ Within 1 m: flashing lights")
               self.turn_lights(100)
               self.state = 6
               self.state6_start_time = now


       # State 6: idle (lights remain on)
       elif self.state == 6:
           # After flashing, wait 5 seconds then turn lights off
           if self.state6_start_time is not None and (now - self.state6_start_time).nanoseconds * 1e-9 >= 3.0:
               self.get_logger().info("→ Turning lights off after 3 seconds")
               self.turn_lights(0)
               self.state = 7


       elif self.state == 7:
           # Idle after lights-off
           pass


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
