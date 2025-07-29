import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class SetpointPublisher(Node):
    def __init__(self):
        super().__init__('setpoint_publisher')
        self.publisher_ = self.create_publisher(Float64, 'target_depth', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = 2.0  # Set your desired depth here
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published target depth: {msg.data} m')

def main():
    rclpy.init()
    node = SetpointPublisher()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()