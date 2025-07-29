import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from dt_apriltags import Detector
from std_msgs.msg import Float64MultiArray
import numpy as np
import math


class FlashLights(Node):
    def __init__(self):
        super().__init__("flashy_light_node")

        self.command_pub = self.create_publisher(
            OverrideRCIn, "override_rc", 10
        )

        #Here, sub to a topic that will tell you to assign level = 100 when tag is within 1 meter
        self.create_subscription(
            Float64MultiArray,
            'apriltag/detection',
            self.april_tag_dist,
            10
        )

        level = 0
        self.turn_lights_on(level)

    def turn_lights_on(self, level):
        """
        Turn the lights on.

        Args:
            level (int): The level to turn the lights on to. 0 is off, 100 is full
        """
        self.get_logger().info(f"Turning lights on to level {level}")
        commands = OverrideRCIn()
        commands.channels = [OverrideRCIn.CHAN_NOCHANGE] * 10
        commands.channels[8] = 1000 + level * 10
        commands.channels[9] = 1000 + level * 10
        self.command_pub.publish(commands)

    def april_tag_dist(self, msg):
        magnitude_sqrd = (msg.data[1]**2) + (msg.data[2]**2) + (msg.data[3]**2)
        magnitude_norm = magnitude_sqrd**0.5
        if magnitude_norm < 1.0:
            self.level = 100
            self.turn_lights_on(100)

def main(args=None):
    rclpy.init(args=args)
    node = FlashLights()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
