import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from scan_slam_msgs.msg import PoseGraph
import time

plt.rcParams['lines.linewidth'] = 2.5
plt.rcParams['font.size'] = 30

class OdometryPlotter(Node):
    def __init__(self):
        super().__init__('odometry_plotter')
        self.real_pose_sub = self.create_subscription(PoseStamped, 'real_pose', self.real_pose_callback, 10)
        self.estimated_pose_sub = self.create_subscription(PoseGraph, 'pose_graph', self.pose_graph_callback, 10)
        self.current_pose_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.save_data_service = self.create_service(Trigger, 'save_trajectory_data', self.save_plot_data_callback)
        self.real_poses = []
        self.estimated_poses = []
        self.current_poses = []

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(16, 10))
        self.line_real, = self.ax.plot([], [], 'b-', label='Real Pose')
        self.current_pose_lines, = self.ax.plot([], [], 'g-', label='Odometry-Only Pose')
        self.current_pose_lines.set_alpha(0.3)
        self.line_estimated, = self.ax.plot([], [], 'r-', label='Estimated Pose')
        self.real_robot_point, = self.ax.plot([], [], 'bo', markersize=10)
        self.current_robot_point, = self.ax.plot([], [], 'go', markersize=10)
        self.estimated_robot_point, = self.ax.plot([], [], 'ro', markersize=10)
        self.ax.legend()
        # self.ax.set_title('Robot Trajectory')
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.axis('equal')
        self.ax.grid()
        plt.tight_layout()

        self.timer = self.create_timer(0.1, self.update_plot)

    def real_pose_callback(self, msg):
        self.real_poses.append((msg.pose.position.x, msg.pose.position.y))

    def pose_graph_callback(self, msg: PoseGraph):
        self.estimated_poses.clear()
        for frame in msg.keyframes:
            self.estimated_poses.append((frame.pose.position.x, frame.pose.position.y))

    def odom_callback(self, msg):
        self.current_poses.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def save_plot_data_callback(self, request, response):
        np.savez(f'plot_data_{time.strftime("%Y-%m-%d_%H-%M-%S")}.npz', real_poses=np.array(self.real_poses), estimated_poses=np.array(self.estimated_poses), current_poses=np.array(self.current_poses))
        self.fig.savefig('trajectories.png', dpi=300, bbox_inches='tight')
        self.get_logger().info('Plot data saved to plot_data.npz')
        response.success = True
        return response

    def update_plot(self):
        if len(self.real_poses) < 2 or len(self.estimated_poses) < 2:
            return
        x, y = zip(*self.real_poses)
        self.line_real.set_data(x, y)
        self.real_robot_point.set_data(x[-1], y[-1])

        x, y = zip(*self.estimated_poses)
        self.line_estimated.set_data(x, y)
        self.estimated_robot_point.set_data(x[-1], y[-1])

        if len(self.current_poses) > 0:
            x, y = zip(*self.current_poses)
            self.current_pose_lines.set_data(x, y)
            self.current_robot_point.set_data(x[-1], y[-1])

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.tight_layout()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    odometry_plotter = OdometryPlotter()
    rclpy.spin(odometry_plotter)
    odometry_plotter.destroy_node()
    rclpy.shutdown()