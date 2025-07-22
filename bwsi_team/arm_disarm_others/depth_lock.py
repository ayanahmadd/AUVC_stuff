import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class DepthLock:
    def __init__(self):
        super.__init__("depth_lock_node")
        self.subscription = self.create_subscription(
            Vector3,
            '/depth_control_input',     
            self.vector_callback,
            10
        )

    def vector_callback(self, msg):
        self.z_val = msg.z
        

