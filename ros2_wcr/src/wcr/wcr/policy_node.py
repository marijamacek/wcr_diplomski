import rclpy
from rclpy.node import Node
import numpy as np
import torch
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations
import matplotlib.pyplot as plt  # at the top


from .ppo_agent import ActorNetwork  # Make sure this is implemented and matches trained policy

class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')

        # === Load Actor Network ===
        self.actor = ActorNetwork(input_dims=8, n_actions=3, alpha=3e-4,
                                  chkpt_dir='/home/marija/IsaacLab/logs/skrl/wcr_direct/trained_policy/checkpoints/best_agent.pt')
        self.actor.load_checkpoint()
        self.actor.eval()

        # === ROS Setup ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # === Control and State ===
        self.last_action = np.zeros(3, dtype=np.float32)
        self.beta = 0.2

        self.position = None
        self.yaw = None
        self.initialized_trajectory = False

        # === Generate Figure-8 ===
        self.num_goals = 300
        self.env_spacing = 16.0
        self.course_length = 8.0
        self.course_width = 8.0

        self.trajectory = self.generate_figure8_trajectory()
        self.target_index = 0
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        plt.ion()

    def generate_figure8_trajectory(self):
        t = np.linspace(0, 2 * np.pi, self.num_goals)
        x = np.sin(t)
        y = np.sin(t) * np.cos(t)

        # Rotate 90°
        x_rot = x
        y_rot = y

        # Scale like in IsaacLab
        x_scaled = x_rot * (self.env_spacing / self.course_length / 2.0)
        y_scaled = y_rot * (self.env_spacing / self.course_width / 2.0)

        return np.stack([x_scaled, y_scaled], axis=1)  # shape: (num_goals, 2)
    
    def visualize(self):
        self.ax.clear()
        traj = self.trajectory
        current_target = self.target_index
        pos = self.position

        self.ax.plot(traj[:, 0], traj[:, 1], 'bo-', label='Waypoints')
        if pos is not None:
            self.ax.plot(pos[0], pos[1], 'ro', label='Robot')
        self.ax.plot(traj[current_target, 0], traj[current_target, 1], 'go', label='Current Target')

        self.ax.set_title("Waypoints and Robot Position")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.legend()
        self.ax.grid(True)
        self.ax.axis('equal')

        plt.draw()
        plt.pause(0.001)

    def odom_callback(self, msg):
        # === Extract position and heading ===
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        current_position = np.array([pos.x, pos.y], dtype=np.float32)
        self.position = current_position
        self.yaw = yaw

        # === Anchor trajectory to first pose ===
        if not self.initialized_trajectory:
            self.trajectory += current_position  # Offset entire trajectory
            self.initialized_trajectory = True

        # === Waypoint Tracking ===
        goal = self.trajectory[self.target_index]
        position_error_vec = goal - current_position
        position_error = np.linalg.norm(position_error_vec)

        if position_error < 0.05:
            self.target_index = (self.target_index + 1) % self.num_goals
            goal = self.trajectory[self.target_index]
            position_error_vec = goal - current_position
            position_error = np.linalg.norm(position_error_vec)

        # === Heading Error ===
        target_heading = np.arctan2(position_error_vec[1], position_error_vec[0])
        heading_error = np.arctan2(np.sin(target_heading - yaw), np.cos(target_heading - yaw))

        # === Get robot velocities ===
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        # Transform to body frame
        vx_b = vx * np.cos(yaw) + vy * np.sin(yaw)
        vy_b = -vx * np.sin(yaw) + vy * np.cos(yaw)

        # === Build normalized observation ===
        obs = np.array([
            position_error / 20.0,
            np.cos(heading_error),
            np.sin(heading_error),
            vx_b / 4.0,
            vy_b / 4.0,
            wz / 4.0,
            self.last_action[0] / 4.0,
            self.last_action[1] / 1.4,
        ], dtype=np.float32)

        obs_tensor = torch.tensor(obs).unsqueeze(0).to(self.actor.device)
        print(obs_tensor)

        with torch.no_grad():
            action = self.actor(obs_tensor).cpu().numpy().flatten()
            print(action)
        # === Smooth and Scale Actions ===
        smoothed_action = self.beta * self.last_action + (1 - self.beta) * action
        self.last_action = smoothed_action

        limit = 4 # [rad/s]
        vx = np.clip(smoothed_action[0]*10, -limit, limit) * 0.0254    # [rad/s] --> [m/s]
        vy = np.clip(smoothed_action[1]*10, -limit, limit) * 0.0254    # [rad/s] --> [m/s]
        omega = np.clip(smoothed_action[2], -1.0, 1.0) * 1.4           # [rad]

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

        # Optional: print debug
        self.get_logger().info(f"Cmd: {cmd.linear.x:.2f}, {cmd.linear.y:.2f}, {cmd.angular.z:.2f}")
        self.visualize() 

def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
