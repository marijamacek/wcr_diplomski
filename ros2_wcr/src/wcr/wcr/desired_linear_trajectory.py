import rclpy
from rclpy.node import Node
from wcr_interfaces.msg import DesiredPoseTwist
import numpy as np
from rosgraph_msgs.msg import Clock

# === Configurable Parameters ===
use_quintic_scaling = False

line_angle = np.deg2rad(0.0)  # Example: 90 degrees (robot moves along y-axis)
line_length = 1.0              # 1 meter length
vx_amplitude = 0.05            # Max velocity in X (m/s)
vy_amplitude = 0.05            # Max velocity in Y (m/s)
sampling_rate = 1000           # Hz (1000Hz publishing rate)

class LinearTrajectoryNode(Node):
    def __init__(self):
        super().__init__('linear_trajectory_node')

        self.publisher_ = self.create_publisher(DesiredPoseTwist, '/wcr/desired_pose_twist', 10)
        self.timer = self.create_timer(1 / sampling_rate, self.trajectory_callback)
        self.desired_pose_twist_msg = DesiredPoseTwist()

        # Initializing velocity components
        self.vx = vx_amplitude * np.cos(line_angle)
        self.vy = vy_amplitude * np.sin(line_angle)
        speed = np.sqrt(self.vx**2 + self.vy**2)
        self.total_time = line_length / speed  # Time to complete the trajectory
        self.start_time_q = self.get_clock().now().nanoseconds
        self.current_time_q = (self.get_clock().now().nanoseconds - self.start_time_q) * 1e-9

        self.start_time = None  # Will be set when the clock is received

        # Subscribe to /clock topic to synchronize with simulation time
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        self.has_finished = False  # To track if the trajectory has completed

    def clock_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.current_time_s = msg.clock.sec + msg.clock.nanosec * 1e-9 - self.start_time

    def trajectory_callback(self):
        if self.start_time is None:
            return  # Wait until the first clock message is received

        if not self.has_finished and self.current_time_s <= self.total_time:
            self.desired_pose_twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.desired_pose_twist_msg.header.frame_id = "odom"

            if use_quintic_scaling == False:
                # Constant velocity along the trajectory
                self.desired_pose_twist_msg.pose.position.x = self.vx * self.current_time_s
                self.desired_pose_twist_msg.pose.position.y = self.vy * self.current_time_s

                self.desired_pose_twist_msg.twist.linear.x = self.vx
                self.desired_pose_twist_msg.twist.linear.y = self.vy

            else:
                # Quintic time scaling for smooth acceleration and deceleration
                t = self.current_time_s
                T = self.total_time
                tau = t / T
                pos_scale = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
                vel_scale = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T

                self.desired_msg.pose.position.x = pos_scale * self.vx * T
                self.desired_msg.pose.position.y = pos_scale * self.vy * T
                self.desired_msg.twist.linear.x = vel_scale * self.vx
                self.desired_msg.twist.linear.y = vel_scale * self.vy

            self.publisher_.publish(self.desired_pose_twist_msg)

        else:
            # Stop publishing the desired trajectory once the robot has traveled 1m
            if not self.has_finished:
                self.has_finished = True  # Set flag to indicate the trajectory is complete
                self.get_logger().info("Desired trajectory completed. Stopping the robot.")
            
            # Stop publishing anything after the trajectory is completed
            self.desired_pose_twist_msg.twist.linear.x = 0.0
            self.desired_pose_twist_msg.twist.linear.y = 0.0
            self.publisher_.publish(self.desired_pose_twist_msg)

        self.get_logger().info(f"Time: {self.current_time_s}, vx: {self.vx}, vy: {self.vy}, total_time: {self.total_time}")


def main(args=None):
    rclpy.init(args=args)
    node = LinearTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
