import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from mavros_msgs.msg import ManualControl
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
import time

tick = 0.0

class DanceMovement(Node):
    def __init__(self):
        super().__init__("auv_movement_node")
        self.publisher = self.create_publisher(ManualControl, '/rov1/manual_control', 10)
        self.get_logger().info("🚀 DanceMovement node has started (ARM + MANUAL mode).")

        # sensor‐feedback state
        self.yaw_rate = 0.0
        self.vx = self.vy = self.vz = 0.0

        # subscribe to IMU yaw rate
        self.create_subscription(Imu,
                                 '/mavros/imu/data',
                                 self._imu_cb, 10)
        # subscribe to body‑frame velocity
        self.create_subscription(TwistStamped,
                                 '/mavros/local_position/velocity_body',
                                 self._vel_cb, 10)

        # drive the sequence at 1 Hz
        self.timer = self.create_timer(1.0, self.run_sequence)

    def _imu_cb(self, msg: Imu):
        self.yaw_rate = msg.angular_velocity.z

    def _vel_cb(self, msg: TwistStamped):
        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.vz = msg.twist.linear.z

    def publish_sensor_brake(self):
        """Publish zeros until yaw_rate and body velocities are
           below threshold for 2 s (20 ticks at 10 Hz)."""
        self.get_logger().info("⏹️  Sensor‑driven brake start…")
        stable = 0
        required = 20      # 20 * 0.1s = 2s stable
        rate_hz = 10
        interval = 1.0 / rate_hz
        thresh_yaw = 0.02  # rad/s
        thresh_lin = 0.05  # m/s
        timeout = time.time() + 6.0  # 6s safety timeout

        while time.time() < timeout and stable < required:
            # publish zero command
            msg = ManualControl()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.x = msg.y = msg.z = msg.r = 0.0
            self.publisher.publish(msg)

            # check feedback
            if (abs(self.yaw_rate) < thresh_yaw and
                abs(self.vx)      < thresh_lin and
                abs(self.vy)      < thresh_lin and
                abs(self.vz)      < thresh_lin):
                stable += 1
            else:
                stable = 0

            time.sleep(interval)

        if stable >= required:
            self.get_logger().info("✅ Sensor‑driven brake: motion held zero for 2 s.")
        else:
            self.get_logger().warn("⚠️ Brake timeout before full 2 s.")

    def run_sequence(self):
        global tick

        if tick < 2.0:
            # 2 s initial setup
            self.publish_setup()

        if tick <  7.0:
            self.publish_first_move_part_one()
        elif tick < 12.0:
            self.publish_first_move_part_two()

        elif tick < 17.0:
            self.publish_second_move_part_one()
        elif tick < 22.0:
            self.publish_second_move_part_two()
        elif tick < 27.0:
            self.publish_second_move_part_three()
        elif tick < 32.0:
            self.publish_second_move_part_four()


        elif tick < 34.0:
            # 2 s sensor‑verified rotation brake
            self.publish_sensor_brake()

        elif tick < 39.0:
            self.publish_setup_for_third_move()
        elif tick < 44.0:
            self.publish_third_move_part_one()
        elif tick < 49.0:
            self.publish_third_move_part_two()

        else:
            self.get_logger().info("✅ All moves complete.")
            self.timer.cancel()
            return

        tick += 1.0

    def publish_setup(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x, msg.y, msg.z, msg.r = 15.0, 0.0, -10.0, 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f"[Setup] x=15,y=0,z=-10,r=0 (tick={tick:.0f})")

    def publish_second_move_part_one(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x, msg.y, msg.z, msg.r = 20.0, 20.0, 0.0, 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f"[2.1] x=20,y=20,z=0,r=0 (tick={tick:.0f})")

    def publish_second_move_part_two(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x, msg.y, msg.z, msg.r = 20.0, -20.0, 0.0, 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f"[2.2] x=20,y=-20,z=0,r=0 (tick={tick:.0f})")

    def publish_second_move_part_three(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x, msg.y, msg.z, msg.r = -20.0, -20.0, 0.0, 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f"[2.3] x=-20,y=-20,z=0,r=0 (tick={tick:.0f})")

    def publish_second_move_part_four(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x, msg.y, msg.z, msg.r = -20.0, 20.0, 0.0, 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f"[2.4] x=-20,y=20,z=0,r=0 (tick={tick:.0f})")

    def publish_first_move_part_one(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x, msg.y, msg.z, msg.r = 0.0, 0.0, -20.0, -10.0
        self.publisher.publish(msg)
        self.get_logger().info(f"[1.1] x=0,y=0,z=-20,r=-10 (tick={tick:.0f})")

    def publish_first_move_part_two(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x, msg.y, msg.z, msg.r = 0.0, 0.0, 20.0, 10.0
        self.publisher.publish(msg)
        self.get_logger().info(f"[1.2] x=0,y=0,z=20,r=10 (tick={tick:.0f})")

    def publish_setup_for_third_move(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x, msg.y, msg.z, msg.r = 30.0, 0.0, 0.0, 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f"[Setup 3] x=30,y=0,z=0,r=0 (tick={tick:.0f})")

    def publish_third_move_part_one(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x, msg.y, msg.z, msg.r = -10.0, 0.0, 0.0, 15.0
        self.publisher.publish(msg)
        self.get_logger().info(f"[3.1] x=-10,y=0,z=0,r=15 (tick={tick:.0f})")

    def publish_third_move_part_two(self):
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x, msg.y, msg.z, msg.r = -10.0, 0.0, 0.0, -15.0
        self.publisher.publish(msg)
        self.get_logger().info(f"[3.2] x=-10,y=0,z=0,r=-15 (tick={tick:.0f})")

def main(args=None):
    rclpy.init(args=args)
    node = DanceMovement()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.des