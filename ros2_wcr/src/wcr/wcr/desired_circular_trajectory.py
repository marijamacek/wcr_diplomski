import rclpy
from rclpy.node import Node
from wcr_interfaces.msg import DesiredPoseTwist
from rosgraph_msgs.msg import Clock
import numpy as np

# === Configurable Parameters ===
radius = 1.0                      # Radius of circle (meters)
angular_velocity = 0.05            # Angular speed (rad/s)
sampling_rate = 100.0             # Hz
frame_id = "odom"                 # Frame for output messages

class DesiredCircularNode(Node):
    def __init__(self):
        super().__init__('desired_circular_node')

        self.publisher_ = self.create_publisher(DesiredPoseTwist, '/wcr/desired_pose_twist', 10)
        self.timer = self.create_timer(1.0 / sampling_rate, self.trajectory_callback)

        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.start_time = None
        self.current_time_s = 0.0

        self.get_logger().info('Publishing desired circular trajectory to /wcr/desired_pose_twist')

    def clock_callback(self, msg):
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = now
        self.current_time_s = now - self.start_time

    def trajectory_callback(self):
        if self.start_time is None:
            return  # Wait for clock sync

        t = self.current_time_s
        omega = angular_velocity

        # Center of the circle is at (-radius, 0)
        x = radius * (np.cos(omega * t)-radius)  
        y = radius * (np.sin(omega * t))

        vx = -radius * omega * np.sin(omega * t)
        vy = radius * omega * np.cos(omega * t)

        msg = DesiredPoseTwist()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.pose.position.x = x
        msg.pose.position.y = y

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy

        self.publisher_.publish(msg)

        # Debug info
        self.get_logger().info(
            f"t={t:.2f} | pos=({x:.2f}, {y:.2f}) | vel=({vx:.2f}, {vy:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DesiredCircularNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
