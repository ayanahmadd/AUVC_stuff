import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl

class AUVMovement(Node):
    def __init__(self):
        super().__init__("auv_movement_node")
        self.publisher = self.create_publisher(ManualControl, 'manual_control', 10)
        self.get_logger().info("🚀 AUVMovement node has started.")

        self.timer = self.create_timer(1.0, self.run_sequence)


    def run_sequence(self):
        msg = ManualControl()

        msg.x = 0.0
        msg.y = 0.0
        msg.z = -10.0
        msg.r = 20.0
        self.get_logger().info("AUV rotate & sink!")

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AUVMovement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()