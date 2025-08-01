#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Float64MultiArray

class FlashLights(Node):
    def __init__(self):
        super().__init__("flashy_light_node")

        # — Parameters you may tune —
        self.declare_parameter("flash_on_duration",    1.0)  # seconds ON
        self.declare_parameter("flash_off_duration",   1.0)  # seconds OFF
        self.declare_parameter("detection_timeout",    1.0)  # seconds before we drop flash
        self.declare_parameter("distance_threshold",   1.0)  # meters

        self.flash_on_duration   = self.get_parameter("flash_on_duration").value
        self.flash_off_duration  = self.get_parameter("flash_off_duration").value
        self.detection_timeout   = self.get_parameter("detection_timeout").value
        self.distance_threshold  = self.get_parameter("distance_threshold").value

        # pub /sub
        self.command_pub = self.create_publisher(OverrideRCIn, "override_rc", 10)
        self.create_subscription(
            Float64MultiArray, "apriltag/detection",
            self.april_tag_callback, 10
        )

        # state
        self.should_flash = False
        self.light_on     = False
        self.last_toggle  = self.now()
        self.last_det_t   = None

        # high-rate timer
        self.timer = self.create_timer(0.1, self._on_timer)
        self.get_logger().info("FlashLights ready")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def april_tag_callback(self, msg: Float64MultiArray):
        # msg.data = [id, x, y, z, …]
        dist = (msg.data[1]**2 + msg.data[2]**2 + msg.data[3]**2)**0.5
        in_range = dist < self.distance_threshold

        if in_range:
            # start / refresh flash
            if not self.should_flash:
                self.get_logger().info("Tag in range → begin flashing")
            self.should_flash = True
            self.last_det_t  = self.now()
        else:
            # immediate stop if tag is too far
            if self.should_flash:
                self.get_logger().info("Tag out of range → stop flashing")
            self.should_flash = False

    def _on_timer(self):
        t = self.now()
        # expire stale detection
        if self.should_flash and self.last_det_t is not None:
            if (t - self.last_det_t) > self.detection_timeout:
                self.get_logger().info("Detection timeout → stop flashing")
                self.should_flash = False

        if self.should_flash:
            # handle on/off durations
            if self.light_on:
                if (t - self.last_toggle) >= self.flash_on_duration:
                    self.light_on    = False
                    self.last_toggle = t
                    self._set_light_level(0)
            else:
                if (t - self.last_toggle) >= self.flash_off_duration:
                    self.light_on    = True
                    self.last_toggle = t
                    self._set_light_level(100)
        else:
            # always force lights off
            if self.light_on:
                self.light_on = False
                self._set_light_level(0)

    def _set_light_level(self, level: int):
        """Map 0–100% → 1000–2000μs on channels 8 & 9."""
        pw = 1000 + level * 10
        cmd = OverrideRCIn()
        cmd.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        cmd.channels[8] = pw
        cmd.channels[9] = pw
        self.command_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FlashLights()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()