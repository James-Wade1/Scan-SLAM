from cProfile import label

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_srvs.srv import Trigger
import message_filters
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['lines.linewidth'] = 2.5
plt.rcParams['font.size'] = 20

class PoseCovarianceAnalyzer(Node):
    def __init__(self):
        super().__init__('pose_covariance_analyzer')

        pose_sub  = message_filters.Subscriber(self, PoseWithCovarianceStamped, '/pose_estimate')
        truth_sub = message_filters.Subscriber(self, PoseStamped, '/real_pose')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [pose_sub, truth_sub],
            queue_size=50,
            slop=0.1,
        )
        self.ts.registerCallback(self.synced_callback)

        self.save_srv = self.create_service(Trigger, '/save_covariance_plot', self.save_callback)

        plt.ion()
        self.fig, self.axes = plt.subplots(2, 3, figsize=(16, 8), constrained_layout=True)

        self.time_data    = []
        self.x_est_data   = []
        self.y_est_data   = []
        self.yaw_est_data = []
        self.x_truth_data = []
        self.y_truth_data = []
        self.yaw_truth_data = []
        self.sigma_x_data = []
        self.sigma_y_data = []
        self.sigma_yaw_data = []
        self.first_time   = None

        self.fig_update_timer  = self.create_timer(0.01, lambda: self.fig.canvas.flush_events())
        self.plot_update_timer = self.create_timer(1.0, self.update_plot)

    def save_callback(self, request, response):
        self.update_plot()  # force a final render with all data
        self.fig.savefig('pose_analysis.png', dpi=300, bbox_inches='tight')
        self.get_logger().info('Figure saved to pose_analysis.png')
        response.success = True
        response.message = 'Figure saved to pose_analysis.png'
        return response

    def synced_callback(self, pose_msg: PoseWithCovarianceStamped, truth_msg: PoseStamped):
        time = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9
        if self.first_time is None:
            self.first_time = time
        time -= self.first_time

        cov = np.array(pose_msg.pose.covariance).reshape(6, 6)

        q     = pose_msg.pose.pose.orientation
        q_t   = truth_msg.pose.orientation
        yaw_est   = 2 * np.arctan2(q.z,   q.w)
        yaw_truth = 2 * np.arctan2(q_t.z, q_t.w)
        yaw_est   = (yaw_est   + np.pi) % (2 * np.pi) - np.pi
        yaw_truth = (yaw_truth + np.pi) % (2 * np.pi) - np.pi

        self.time_data.append(time)
        self.x_est_data.append(pose_msg.pose.pose.position.x)
        self.y_est_data.append(pose_msg.pose.pose.position.y)
        self.yaw_est_data.append(yaw_est)
        self.x_truth_data.append(truth_msg.pose.position.x)
        self.y_truth_data.append(truth_msg.pose.position.y)
        self.yaw_truth_data.append(yaw_truth)
        self.sigma_x_data.append(np.sqrt(cov[0, 0]))
        self.sigma_y_data.append(np.sqrt(cov[1, 1]))
        self.sigma_yaw_data.append(np.sqrt(cov[5, 5]))

    def update_plot(self):
        if len(self.time_data) < 2:
            return

        t         = np.array(self.time_data)
        estimates = [np.array(self.x_est_data),
                     np.array(self.y_est_data),
                     np.array(self.yaw_est_data)]
        truths    = [np.array(self.x_truth_data),
                     np.array(self.y_truth_data),
                     np.array(self.yaw_truth_data)]
        sigmas    = [np.array(self.sigma_x_data),
                     np.array(self.sigma_y_data),
                     np.array(self.sigma_yaw_data)]
        labels    = ['X (m)', 'Y (m)', 'Yaw (rad)']

        for col, (est, truth, sigma, label) in enumerate(zip(estimates, truths, sigmas, labels)):
            ax = self.axes[0, col]
            ax.clear()
            ax.plot(t, truth, 'r--', linewidth=1.5, label='Ground truth')
            ax.plot(t, est,   'b-',  linewidth=1.5, label='Estimate')
            ax.fill_between(t, est - sigma,       est + sigma,
                            alpha=0.35, color='blue', label='1σ')
            ax.fill_between(t, est - 2 * sigma,   est + 2 * sigma,
                            alpha=0.15, color='blue', label='2σ')
            ax.set_ylabel(label)
            if col == 0:
                ax.legend(loc='upper right', fontsize=15)

            motion_cov_lower = [0.1, 0.1, 0.05]

            ax = self.axes[1, col]
            ax.clear()
            ax.plot(t, sigma, 'g-', linewidth=1.5)
            ax.axhline(y=np.sqrt(motion_cov_lower[col]), color='r', linestyle='--',
                    linewidth=1.5, label=f'Motion cov bound (√{motion_cov_lower[col]})')
            ax.set_xlabel('Time (s)')
            short_label = label.split(' ')[0]
            ax.set_ylabel(f'$\\sigma_{{{short_label}}}$ ({"m" if "rad" not in label else "rad"})')
            ax.legend(loc='upper right', fontsize=15)

        self.fig.canvas.draw()


def main(args=None):
    rclpy.init(args=args)
    analyzer = PoseCovarianceAnalyzer()
    rclpy.spin(analyzer)
    analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()