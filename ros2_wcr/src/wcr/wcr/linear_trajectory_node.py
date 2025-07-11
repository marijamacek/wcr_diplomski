import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from wcr_interfaces.msg import DesiredPoseTwist
import math

class LinearFollowerNode(Node):
    def __init__(self):
        super().__init__('linear_follower_node')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(DesiredPoseTwist, '/wcr/desired_pose_twist', self.desired_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.desired_twist = None
        self.current_yaw = None
        self.current_pose_x = None
        self.current_pose_y = None
        self.odom_received = False

        # Control gains
        self.k_linear = 2.0  # Linear control gain
        self.k_lateral = 3.0 # Lateral (sideways) control gain

        self.get_logger().info('LinearFollowerNode initialized.')

    def desired_callback(self, msg):
        """Callback for desired trajectory (target position and velocities)."""
        self.desired_twist = msg

    def odom_callback(self, msg):
        """Callback for odometry information."""
        self.odom_received = True
        # Get robot's current position from odometry
        self.current_pose_x = msg.pose.pose.position.x
        self.current_pose_y = msg.pose.pose.position.y

        # Extract robot's yaw (orientation) from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        """Main control loop to update robot velocity."""
        if not self.odom_received or self.desired_twist is None:
            return

        # Get current robot position
        current_x = self.current_pose_x
        current_y = self.current_pose_y

        # Get target position from desired_twist
        target_x = self.desired_twist.pose.position.x
        target_y = self.desired_twist.pose.position.y

        # Calculate the vector from current position to the target
        dx = target_x - current_x
        dy = target_y - current_y

        # Calculate desired angle (heading) towards the target
        desired_theta = math.atan2(dy, dx)

        # Normalize the angle error (difference between current yaw and desired heading)
        angle_error = self.normalize_angle(desired_theta - self.current_yaw)

        # Calculate distance to the target
        distance = math.sqrt(dx**2 + dy**2)

        # Linear velocity: proportional to the distance to target
        linear_x = self.k_linear * distance * math.cos(angle_error)

        # Lateral velocity (sideways motion): proportional to the lateral error
        lateral_y = self.k_lateral * dy  # Moving sideways towards the target

        # Create a Twist message and set linear and angular velocities
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = lateral_y  # Set lateral movement
        cmd.angular.z = 0.0  # No angular movement

        # Publish command to /cmd_vel
        self.publisher_.publish(cmd)

        # Debug info
        self.get_logger().info(
            f"Current Pos: ({current_x:.2f}, {current_y:.2f}) | "
            f"Target Pos: ({target_x:.2f}, {target_y:.2f}) | "
            f"Dist: {distance:.2f} | Angle Error: {math.degrees(angle_error):.1f}° | "
            f"Lateral Velocity: {lateral_y:.2f}"
        )

    def normalize_angle(self, angle):
        """Normalize angle to range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    """Main function to initialize ROS2 and start the node."""
    rclpy.init(args=args)
    node = LinearFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
