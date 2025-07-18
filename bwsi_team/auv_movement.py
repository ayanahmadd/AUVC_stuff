# Testing changes
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
import random
import time

class AUVMovement(Node):
    def __init__(self):
        super().__init__("auv_movement_node")
        self.publisher = self.create_publisher(ManualControl, 'manual_control', 10)
        self.get_logger().info("🚀 AUVMovement node has started.")

        self.tick = 0.0
        self.swing = 1
        self.beat_counter = 0
        self.in_random_mode = False

        self.start_time = time.time()
        self.last_switch_time = self.start_time
        self.switch_interval_sec = 5.0
        self.current_axes = self.get_random_axes(exclude=[])

        self.timer = self.create_timer(1.0, self.run_sequence)  # 1 Hz

    def run_sequence(self):
        now = time.time()
        elapsed_sec = now - self.start_time

        msg = ManualControl()

        if elapsed_sec < 72.0:
            # Scripted movement
            if 0.0 <= self.tick < 3.0:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = -30.0
                msg.r = 0.0
                self.get_logger().info("AUV running move #0!")

            elif 3.0 <= self.tick < 5.0:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("Recalibrating...")

            elif 5.0 <= self.tick < 10.0:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = -20.0
                msg.r = -10.0
                self.get_logger().info("AUV running move #1, part 1!")

            elif 10.0 <= self.tick < 12.0:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("Recalibrating...")

            elif 12.0 <= self.tick < 17.0:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = 20.0
                msg.r = 10.0
                self.get_logger().info("AUV running move #1, part 2!")

            elif 17.0 <= self.tick < 19.0:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("Recalibrating...")

            elif 19.0 <= self.tick < 24.0:
                msg.x = 20.0
                msg.y = -20.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("AUV running move #2, part 1!")

            elif 24.0 <= self.tick < 26.0:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("Recalibrating...")

            elif 26.0 <= self.tick < 28.0:
                msg.x = 20.0
                msg.y = 20.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("AUV running move #2, part 2!")

            elif 28.0 <= self.tick < 30.0:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("Recalibrating...")

            elif 30.0 <= self.tick < 35.0:
                msg.x = -20.0
                msg.y = 20.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("AUV running move #2, part 3!")

            elif 35.0 <= self.tick < 37.0:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("Recalibrating...")

            elif 37.0 <= self.tick < 42.0:
                msg.x = -20.0
                msg.y = -20.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("AUV running move #2, part 4!")

            elif 42.0 <= self.tick < 44.0:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("Recalibrating...")

            elif 44.0 <= self.tick < 49.0:
                msg.x = 20.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("AUV running move #3, part 0!")

            elif 49.0 <= self.tick < 51.0:
                msg.x = 0.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = 0.0
                self.get_logger().info("Recalibrating...")

            elif 51.0 <= self.tick < 56.0:
                msg.x = -20.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = 25.0
                self.get_logger().info("AUV running move #3, part 1!")


            elif 56.0 <= self.tick < 61.0:
                msg.x = -20.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = -25.0
                self.get_logger().info("AUV running move #3, part 2!")

            elif 61.0 <= self.tick < 66.0:
                msg.x = -20.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = 25.0
                self.get_logger().info("AUV running move #3, part 1 again!")

            elif 66.0 <= self.tick < 71.0:
                msg.x = -20.0
                msg.y = 0.0
                msg.z = 0.0
                msg.r = -25.0
                self.get_logger().info("AUV running move #3, part 2 again!")

            self.tick += 1.0

        else:
            # Random movement mode
            if not self.in_random_mode:
                self.in_random_mode = True
                self.get_logger().info("🔀 Switching to RANDOM movement mode!")
                self.last_switch_time = now

            time_since_last_switch = now - self.last_switch_time
            if time_since_last_switch >= self.switch_interval_sec:
                prev_axes = self.current_axes
                self.current_axes = self.get_random_axes(exclude=prev_axes)
                self.last_switch_time = now
                self.get_logger().info(f"🎭 New movement pattern: {self.current_axes}")

            msg.x = 0.0
            msg.y = 0.0
            msg.z = 20.0
            msg.r = 0.0

            amp = 400.0 * self.swing

            if 'x' in self.current_axes:
                msg.x = amp
            if 'y' in self.current_axes:
                msg.y = amp
            if 'z' in self.current_axes:
                msg.z = 500.0 + (100.0 * self.swing)
            if 'r' in self.current_axes:
                msg.r = amp

            self.get_logger().info(
                f"🎵 Beat {self.beat_counter+1}: moving {self.current_axes} ({'→' if self.swing > 0 else '←'})"
            )
            self.swing *= -1
            self.beat_counter += 1

        self.publisher.publish(msg)

    def get_random_axes(self, exclude):
        all_axes = ['x', 'y', 'z', 'r']
        new_set = []
        while not new_set or set(new_set) == set(exclude):
            new_set = random.sample(all_axes, k=random.randint(1, 4))
        return new_set

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