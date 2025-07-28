import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn


class FlashLights(Node):
    def __init__(self):
        super().__init__("flashy_light_node")

        self.command_pub = self.create_publisher(
            OverrideRCIn, "override_rc", 10
        )

        #Here, sub to a topic that will tell you to assign level = 100 when tag is within 1 meter
        
        
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

def main(args=None):
    rclpy.init(args=args)
    node = FlashLights()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
