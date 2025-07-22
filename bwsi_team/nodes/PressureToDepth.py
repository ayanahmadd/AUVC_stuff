import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure

class ConvertToDepth(Node):
    def __init__(self):
        super().__init__("pressure subscriber")

        self.sub = self.create_subscription(
            FluidPressure,        # the message type
            "/pressure",    # the topic name,
            self.convert_to_depth,  # the subscription's callback method
            10              # QOS (will be covered later)
        )

        self.get_logger().info("initialized pressure converter node")

    def convert_to_depth(self, msg, base_pressure = 101325.0, base_density = 1000.0, gravity = 9.8):
        depth = (msg.fluid_pressure - base_pressure)/(gravity*base_density)
        self.get_logger().info(f"Depth reading: {depth} meters!")
        return depth

def main(args=None):
    rclpy.init(args=args)
    node = ConvertToDepth()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()