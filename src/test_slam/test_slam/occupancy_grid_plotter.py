import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import OccupancyGrid
from scan_slam_msgs.msg import PoseGraph, Constraint, KeyFrame
from std_srvs.srv import Trigger
from skimage.draw import line
import time

plt.rcParams['lines.linewidth'] = 2.5
plt.rcParams['font.size'] = 40

class OccpuancyGridNumpy:
    def __init__(self, l_x, l_y, width, height, resolution, l_occ=0.85, l_free=-0.4, l_min=-10.0, l_max=3.0):
        self.width = width
        self.height = height
        self.l_x = l_x
        self.l_y = l_y
        self.x_min = -l_x / 2
        self.y_min = -l_y / 2
        self.resolution = resolution
        self.grid = np.zeros((height, width), dtype=np.float32)
        self.l_occ = l_occ
        self.l_free = l_free
        self.l_min = l_min
        self.l_max = l_max

    def clear_grid(self):
        self.grid.fill(0.0)

    def world_to_grid(self, x, y):
        x = np.asarray(x)
        y = np.asarray(y)
        grid_x = np.floor((x - self.x_min) / self.resolution).astype(int)
        grid_y = np.floor((y - self.y_min) / self.resolution).astype(int)
        return grid_x, grid_y

    def update_with_keyframe(self, keyframe: KeyFrame):
        robot_x = keyframe.pose.position.x
        robot_y = keyframe.pose.position.y
        robot_yaw = 2 * np.arctan2(keyframe.pose.orientation.z, keyframe.pose.orientation.w)
        robot_grid_x, robot_grid_y = self.world_to_grid(robot_x, robot_y)

        scan = keyframe.scan
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges))

        hit_mask = np.isfinite(ranges) & \
                (ranges > max(scan.range_min, 0.01)) & \
                (ranges < scan.range_max)

        miss_mask = ~np.isfinite(ranges) | (ranges >= scan.range_max)

        hit_ranges = ranges[hit_mask]
        hit_angles = angles[hit_mask] + robot_yaw
        end_xs = robot_x + hit_ranges * np.cos(hit_angles)
        end_ys = robot_y + hit_ranges * np.sin(hit_angles)
        end_grid_x, end_grid_y = self.world_to_grid(end_xs, end_ys)

        for end_gx, end_gy in zip(end_grid_x, end_grid_y):
            rr, cc = line(robot_grid_y, robot_grid_x, end_gy, end_gx)
            in_bounds = (rr[:-1] >= 0) & (rr[:-1] < self.height) & \
                        (cc[:-1] >= 0) & (cc[:-1] < self.width)
            self.grid[rr[:-1][in_bounds], cc[:-1][in_bounds]] += self.l_free
            if 0 <= end_gx < self.width and 0 <= end_gy < self.height:
                self.grid[end_gy, end_gx] += self.l_occ

        miss_angles = angles[miss_mask]
        miss_end_xs = robot_x + scan.range_max * np.cos(miss_angles)
        miss_end_ys = robot_y + scan.range_max * np.sin(miss_angles)
        miss_grid_x, miss_grid_y = self.world_to_grid(miss_end_xs, miss_end_ys)

        for end_gx, end_gy in zip(miss_grid_x, miss_grid_y):
            end_gx_c = int(np.clip(end_gx, 0, self.width - 1))
            end_gy_c = int(np.clip(end_gy, 0, self.height - 1))
            rr, cc = line(robot_grid_y, robot_grid_x, end_gy_c, end_gx_c)
            in_bounds = (rr >= 0) & (rr < self.height) & \
                        (cc >= 0) & (cc < self.width)
            self.grid[rr[in_bounds], cc[in_bounds]] += self.l_free

        np.clip(self.grid, self.l_min, self.l_max, out=self.grid)

class OccupancyGridPlotter(Node):
    def __init__(self):
        super().__init__('occupancy_grid_plotter')
        self.declare_parameter('publish_grid', False)
        self.declare_parameter('plot_grid', True)
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('grid_x_size', 8.0)
        self.declare_parameter('grid_y_size', 6.0)

        self.publish_grid = self.get_parameter('publish_grid').get_parameter_value().bool_value
        self.plot_grid = self.get_parameter('plot_grid').get_parameter_value().bool_value
        self.grid_resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.grid_x_size = self.get_parameter('grid_x_size').get_parameter_value().double_value
        self.grid_y_size = self.get_parameter('grid_y_size').get_parameter_value().double_value
        self.grid_width = int(self.grid_x_size / self.grid_resolution)
        self.grid_height = int(self.grid_y_size / self.grid_resolution)

        self.pose_graph = None
        self.pose_graph_sub = self.create_subscription(PoseGraph, 'pose_graph', self.pose_graph_callback, 10)
        self.save_occupancy_grid_srv = self.create_service(Trigger, 'save_occupancy_grid', self.save_occupancy_grid_callback)
        if self.plot_grid:
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(16, 10))
            # self.ax.set_title('Occupancy Grid Map')
            self.ax.set_xlabel('X Position (m)')
            self.ax.set_ylabel('Y Position (m)')
            self.ax.axis('equal')
            self.ax.grid()
            plt.tight_layout()
            self.fig_timer = self.create_timer(0.05, lambda: self.fig.canvas.flush_events())

        if self.publish_grid:
            self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)

        self.timer = self.create_timer(3.0, self.update_plot)
        self.occupancy_grid = OccpuancyGridNumpy(self.grid_x_size, self.grid_y_size, self.grid_width, self.grid_height, self.grid_resolution)

    def pose_graph_callback(self, msg: PoseGraph):
        self.pose_graph = msg

    def update_plot(self):
        if self.pose_graph is None:
            self.get_logger().warn('No pose graph received yet, cannot update occupancy grid.')
            return

        self.occupancy_grid.clear_grid()
        for keyframe in self.pose_graph.keyframes:
            self.occupancy_grid.update_with_keyframe(keyframe)

        prob = 1 - 1 / (1 + np.exp(self.occupancy_grid.grid))
        if self.plot_grid:
            self.plot_grid_fn(prob)

    def plot_grid_fn(self, prob):
        self.ax.clear()
        # self.ax.set_title('Occupancy Grid Map')
        self.ax.axis('equal')
        self.ax.grid()
        self.ax.axis('off')
        extent = [self.occupancy_grid.x_min,
                  self.occupancy_grid.x_min + self.grid_width * self.grid_resolution,
                  self.occupancy_grid.y_min,
                  self.occupancy_grid.y_min + self.grid_height * self.grid_resolution]
        display = np.full_like(prob, 0.5)
        display[prob < 0.001] = 0.0
        display[prob > 0.55] = 1.0
        im = self.ax.imshow(display, origin='lower', extent=extent, cmap='gray_r', vmin=0.0, vmax=1.0)
        plt.pause(0.001)

    def save_occupancy_grid_callback(self, request, response):
        self.get_logger().info('Saving occupancy grid to file...')
        filename = f'occupancy_grid_{time.strftime("%Y-%m-%d_%H-%M-%S")}.npy'
        with open(filename, 'wb') as f:
            np.save(f, self.occupancy_grid.grid)
        self.fig.savefig('occupancy_grid.png', dpi=300, bbox_inches='tight')
        self.get_logger().info(f'Occupancy grid saved to {filename}')
        return response

def main():
    rclpy.init()
    node = OccupancyGridPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()