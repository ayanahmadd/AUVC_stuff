#ros2 topic pub /target_depth std_msgs/msg/Float64 "{data: 5.0}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import ManualControl
from bluerov2_controllers import PIDController

class DepthPIDController(Node):
    def __init__(self):
        super().__init__('depth_lock')

        # --- PID setup with a dummy initial setpoint (0m) ---
        self.pid = PIDController.PIDController(kp=70.0, ki=2.5, kd=5,
                                 setpoint=0.0, dt=0.1)

        # --- State ---
        # self.base_pressure = None   # will hold “one atmosphere” at start
        self.current_depth = 0.0

        # --- Publishers & Subscribers ---
        # Control output
        self.pub = self.create_publisher(ManualControl, 'manual_control', 10)

        # Pressure → depth conversion
        self.sub = self.create_subscription(Float64, 'depth', self.converting_depth, 10)

        # New: target depth (in feet) → setpoint (in meters)
        self.sub_setpoint = self.create_subscription(
            Float64, 'target_depth', self.setpoint_callback, 10)

        # --- Timer for control loop ---
        self.timer = self.create_timer(self.pid.dt, self.control_loop)

        self.get_logger().info("Depth PID controller initialized")

    def setpoint_callback(self, msg: Float64):
        """Receive desired depth in feet → convert to meters and update PID."""
        meters = msg.data
        feet = meters * 3.28084
        self.pid.setpoint = meters
        self.get_logger().info(f"New setpoint: {feet:.2f} ft → {meters:.2f} m")

    def converting_depth(self, msg: Float64):
        self.current_depth = msg.data  # Save the depth value for use in control loop
        self.get_logger().info(f"Current depth: {self.current_depth:.2f} feet")
        return self.current_depth

    def control_loop(self):

        # run PID on the current depth
        thrust = self.pid.compute(self.current_depth)

        # clamp into [-1, 1]
        # thrust = max(min(thrust, 1.0), -1.0)

        # publish to mavros manual_control
        m = ManualControl(x=0.0, y=0.0, z=-thrust, r=0.0)
        self.pub.publish(m)
        self.get_logger().info(
            f"[PID] depth={self.current_depth:.3f} m  setpt={self.pid.setpoint:.3f} m  → thrust z={m.z:.3f}"
        )

def main():
    rclpy.init()
    node = DepthPIDController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
