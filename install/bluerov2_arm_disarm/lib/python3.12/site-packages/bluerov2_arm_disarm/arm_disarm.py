import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

class ArmDisarm(Node):
    def __init__(self):
        super().__init__("arm_disarm_node")
        self.client = self.create_client(SetBool, "/arming")
        self.get_logger().info("Arm/Disarm node initialized.")

        

    def arm(self, boolean_value=True):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /arming service to become available...")
        request = SetBool.Request()
        request.data = boolean_value
        self.get_logger().info(f"Requesting to {'arm' if boolean_value else 'disarm'} the vehicle.")
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Service replied: success={future.result().success}, message='{future.result().message}'")
        else:
            self.get_logger().error("No response received from /arming.")

def main(args=None):
    rclpy.init(args=args)
    node = ArmDisarm()
    node.arm(True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.arm(False)
        print("\nKeyboardInterrupt received, disarming AUV...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
