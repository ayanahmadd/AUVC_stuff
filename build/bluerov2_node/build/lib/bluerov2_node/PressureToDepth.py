import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure

class ConvertToDepth(Node):
    def __init__(self):
        super().__init__("pressure_subscriber")

        self.base_pressure = None  # dynamically set
        self.sub = self.create_subscription(
            FluidPressure,
            "/pressure",
            self.convert_to_depth,
            10
        )

        self.pub = self.create_publisher(
            Float64,
            "/depth",
            10
        )

        self.get_logger().info("Initialized pressure-to-depth node")

    def convert_to_depth(self, msg, base_density=1000.0, gravity=9.8):
        if self.base_pressure is None:
            self.base_pressure = msg.fluid_pressure
            self.get_logger().info(f"Baseline pressure set: {self.base_pressure:.2f} Pa")

        depth_meters = (msg.fluid_pressure - self.base_pressure) / (gravity * base_density)
        depth_meters = max(depth_meters, 0.0)

        self.get_logger().info(f"Depth reading: {depth_meters:.2f} meters")

        depth_msg = Float64()
        depth_msg.data = depth_meters
        self.pub.publish(depth_msg)

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
