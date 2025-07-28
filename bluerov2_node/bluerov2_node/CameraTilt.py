import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn

class CameraTiltController(Node):
    def __init__(self):
        super().__init__('camera_tilt_controller')
        self.pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.get_logger().info("CameraTiltController started")

    def send_pwms(self, pwm_value):
        msg = OverrideRCIn()
        msg.channels = [0] * 18
        msg.channels[7] = pwm_value  # channel 8
        self.pub.publish(msg)
        self.get_logger().info(f"Sent tilt PWM: {pwm_value}")

    def tilt_up(self):
        self.send_pwms(1900)

    def tilt_down(self):
        self.send_pwms(1100)

    def center(self):
        self.send_pwms(1500)

def main():
    rclpy.init()
    node = CameraTiltController()

    # Example command sequence
    node.center()
    node.get_logger().info("Centered")
    rclpy.spin_once(node, timeout_sec=1.0)

    node.tilt_up()
    rclpy.spin_once(node, timeout_sec=1.0)

    node.tilt_down()
    rclpy.spin_once(node, timeout_sec=1.0)

    node.center()
    rclpy.spin_once(node, timeout_sec=1.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()