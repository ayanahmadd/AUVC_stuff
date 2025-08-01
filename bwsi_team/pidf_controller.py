import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import Twist

class pidf_controller(Node):
    def __init__(self):
        super().__init__('depth_pidf_controller')

        # PIDF 
        self.Kp = 2.0
        self.Ki = 0.1
        self.Kd = 1.2
        self.FF = 0.15 #feedforward

        # Desired depth CHANGE
        self.setpoint = 5.0

        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.depth_sub = self.create_subscription(
            FluidPressure,
            '/BlueROV2/pressure',
            self.pressure_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/BlueROV2/cmd_vel', 10)

        self.get_logger().info("PIDF depth controller started.")

    def pressure_callback(self, msg):
        pressure_pa = msg.fluid_pressure
        rho = 997.0 
        g = 9.80665  
        current_depth = pressure_pa / (rho * g)

        
        error = self.setpoint - current_depth

        
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        if dt == 0:
            return

        
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative) + self.FF

        
        output = max(min(output, 1.0), -1.0)

        
        cmd = Twist()
        cmd.linear.z = output
        self.cmd_pub.publish(cmd)

       
        self.get_logger().info(f"Depth: {current_depth:.2f} m | Output: {output:.2f}")

        
        self.last_error = error
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = pidf_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PIDF controller shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
