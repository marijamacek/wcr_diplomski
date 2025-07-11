import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from wcr_interfaces.msg import DesiredPoseTwist
import math

class CircularFollowerNode(Node):
    def __init__(self):
        super().__init__('circular_follower_node')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(DesiredPoseTwist, '/wcr/desired_pose_twist', self.desired_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.desired_twist = None
        self.current_yaw = None
        self.odom_received = False

        # Control gains
        self.k_linear = 1.0
        self.k_angular = 3.0

        self.get_logger().info('CircularFollowerNode initialized.')

    def desired_callback(self, msg):
        self.desired_twist = msg

    def odom_callback(self, msg):
        self.odom_received = True
        # In odom_callback
        self.current_pose_x = msg.pose.pose.position.x
        self.current_pose_y = msg.pose.pose.position.y


        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        if not self.odom_received or self.desired_twist is None:
            return

       # Extract robot position from odometry
        current_x = self.current_pose_x
        current_y = self.current_pose_y

    # Extract target position from desired msg
        target_x = self.desired_twist.pose.position.x
        target_y = self.desired_twist.pose.position.y

# Vector from current to desired
        dx = target_x - current_x
        dy = target_y - current_y

# Recompute desired_theta and speed
        desired_theta = math.atan2(dy, dx)
        angle_error = self.normalize_angle(desired_theta - self.current_yaw)

# Distance to target
        distance = math.sqrt(dx**2 + dy**2)

        linear_x = self.k_linear * distance * math.cos(angle_error)
        angular_z = self.k_angular * angle_error

        # Send command
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.publisher_.publish(cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = CircularFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
