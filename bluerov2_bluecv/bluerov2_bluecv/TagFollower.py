import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int16
from mavros_msgs.msg import ManualControl
from bluerov2_controllers.bluerov2_controllers.PIDController import PIDController

class AprilTagFollower(Node):
    def __init__(self):
        super().__init__('apriltag_follower_pid')

        # PID Controllers
        self.forward_pid = PIDController(kp=1.2, ki=0.01, kd=0.4, setpoint=1.0)  # z-axis: 1 meter distance
        self.yaw_pid = PIDController(kp=500.0, ki=5.0, kd=200.0, setpoint=0.0)   # x-axis: horizontal center

        # ROS setup
        self.create_subscription(Float64MultiArray, 'apriltag/detection', self.detection_cb, 10)
        self.create_subscription(Int16, '/heading', self.heading_cb, 10)
        self.pub = self.create_publisher(ManualControl, '/manual_control', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # State
        self.last_detection_time = None
        self.tag_position = None
        self.current_heading = None
        self.spin_start_heading = None
        self.spin_direction = 1  # 1 = clockwise, -1 = counterclockwise
        self.spin_threshold = 330.0  # Degrees considered as a full rotation

        self.get_logger().info("AprilTag Follower PID node started. Using heading-based spin alternation.")

    def detection_cb(self, msg: Float64MultiArray):
        if len(msg.data) != 7:
            return
        self.tag_position = msg.data[1:4]  # x, y, z
        self.last_detection_time = self.get_clock().now()

    def heading_cb(self, msg: Int16):
        self.current_heading = float(msg.data)

    def control_loop(self):
        now = self.get_clock().now()
        twist = ManualControl()
        twist.header.stamp = now.to_msg()

        # If no recent detection → spin in place using heading
        if self.last_detection_time is None or \
           (now - self.last_detection_time).nanoseconds * 1e-9 > 1.5:

            if self.current_heading is None:
                self.get_logger().warn("No heading data yet, cannot spin.")
                return

            if self.spin_start_heading is None:
                self.spin_start_heading = self.current_heading
                self.get_logger().info(f"Started spinning from heading {self.spin_start_heading:.1f}°")

            # Calculate how far we've turned
            delta = (self.current_heading - self.spin_start_heading + 360.0) % 360.0

            if delta >= self.spin_threshold:
                self.spin_direction *= -1
                self.spin_start_heading = self.current_heading
                self.get_logger().info(
                    f"Completed ~360° spin. Switching spin direction to {'CW' if self.spin_direction == 1 else 'CCW'}."
                )

            twist.x = 0.0
            twist.y = 0.0
            twist.z = 0.0
            twist.r = 400.0 * self.spin_direction

            self.pub.publish(twist)
            self.get_logger().info(
                f"Spinning {'CW' if self.spin_direction == 1 else 'CCW'} — heading: {self.current_heading:.1f}°"
            )
            return

        # Reset spin state when tag is found
        self.spin_start_heading = None

        # PID control logic
        x, y, z = self.tag_position
        forward_cmd = self.forward_pid.compute(z)
        yaw_cmd = self.yaw_pid.compute(x)

        forward_cmd = max(min(forward_cmd, 600.0), -600.0)
        yaw_cmd = max(min(yaw_cmd, 600.0), -600.0)

        twist.x = float(forward_cmd)
        twist.y = 0.0
        twist.z = 0.0
        twist.r = float(yaw_cmd)

        self.pub.publish(twist)
        self.get_logger().info(
            f"Tag z={z:.2f} → x_cmd={forward_cmd:.1f}, x={x:.2f} → yaw_cmd={yaw_cmd:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
