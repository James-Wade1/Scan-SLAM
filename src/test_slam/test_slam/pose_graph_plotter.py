import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from std_srvs.srv import Trigger
from scan_slam_msgs.msg import PoseGraph, Constraint, KeyFrame
import networkx as nx
import time

plt.rcParams['lines.linewidth'] = 2.5
plt.rcParams['font.size'] = 20

class PoseGraphPlotter(Node):
    def __init__(self):
        super().__init__('odometry_plotter')
        self.estimated_pose_sub = self.create_subscription(PoseGraph, 'pose_graph', self.pose_graph_callback, 10)
        self.save_data_service = self.create_service(Trigger, 'save_pose_graph_data', self.save_plot_data_callback)

        self.G = nx.Graph()
        self.poses: dict[int, tuple[float, float]] = {}
        self.constraints: list[Constraint] = []

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(16, 8))
        # self.ax.set_title('Pose Graph Visualization')
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.grid()
        self.timer = self.create_timer(0.1, self.update_plot)
        plt.tight_layout()

    def pose_graph_callback(self, msg: PoseGraph):
        for frame in msg.keyframes:
            self.poses[frame.id] = (frame.pose.position.x, frame.pose.position.y)
        self.constraints = msg.constraints

    def save_plot_data_callback(self, request, response):
        self.fig.savefig('pose_graph.png', dpi=300, bbox_inches='tight')
        response.success = True
        return response

    def update_plot(self):
        if not self.poses:
            return

        self.ax.cla()  # clear axes each frame

        self.G.clear()
        for frame_id, pose in self.poses.items():
            self.G.add_node(frame_id, pos=(pose[0], pose[1]))
        for constraint in self.constraints:
            is_loop = constraint.is_loop_closure
            self.G.add_edge(constraint.from_id, constraint.to_id, loop_closure=is_loop)

        pos = nx.get_node_attributes(self.G, 'pos')
        sequential_edges = [(u, v) for u, v, d in self.G.edges(data=True) if not d['loop_closure']]
        loop_edges = [(u, v) for u, v, d in self.G.edges(data=True) if d['loop_closure']]

        nx.draw_networkx_nodes(self.G, pos, node_size=20, node_color='blue', ax=self.ax)
        nx.draw_networkx_edges(self.G, pos, edgelist=sequential_edges, edge_color='gray', width=1, ax=self.ax)
        nx.draw_networkx_edges(self.G, pos, edgelist=loop_edges, edge_color='red', width=2, ax=self.ax)

        self.ax.set_aspect('equal')
        self.ax.grid(True)
        # self.ax.set_title('Pose Graph Visualization')
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.tight_layout()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    odometry_plotter = PoseGraphPlotter()
    rclpy.spin(odometry_plotter)
    odometry_plotter.destroy_node()
    rclpy.shutdown()