import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import message_filters
import numpy as np
import pickle
from pathlib import Path
from std_srvs.srv import Trigger


class TrajectoryLoggerNode(Node):
    def __init__(self):
        super().__init__('trajectory_logger_node')

        self.declare_parameter('output_dir', 'trajectories')
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        pose_sub  = message_filters.Subscriber(self, PoseWithCovarianceStamped, '/pose_estimate')
        truth_sub = message_filters.Subscriber(self, PoseStamped, '/real_pose')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [pose_sub, truth_sub],
            queue_size=100,
            slop=0.1,
        )
        self.ts.registerCallback(self.synced_callback)

        self.save_srv = self.create_service(Trigger, '~/save_data', self.save_data_callback)

        self.est_trajectory   = []   # list of (t, x, y, yaw)
        self.truth_trajectory = []   # list of (t, x, y, yaw)
        self.first_time = None

        self.get_logger().info(f'Trajectory logger started. Saving to {self.output_dir}')

    def synced_callback(self, pose_msg: PoseWithCovarianceStamped, truth_msg: PoseStamped):
        t = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9
        if self.first_time is None:
            self.first_time = t
        t -= self.first_time
        self.get_logger().info(f'Received synced messages at t={t:.2f}s')

        # Estimated
        est_x   = pose_msg.pose.pose.position.x
        est_y   = pose_msg.pose.pose.position.y
        q       = pose_msg.pose.pose.orientation
        est_yaw = (2 * np.arctan2(q.z, q.w) + np.pi) % (2 * np.pi) - np.pi

        # Ground truth
        truth_x   = truth_msg.pose.position.x
        truth_y   = truth_msg.pose.position.y
        q_t       = truth_msg.pose.orientation
        truth_yaw = (2 * np.arctan2(q_t.z, q_t.w) + np.pi) % (2 * np.pi) - np.pi

        self.est_trajectory.append((t, est_x, est_y, est_yaw))
        self.truth_trajectory.append((t, truth_x, truth_y, truth_yaw))

    def save_data_callback(self, request, response):
        est   = np.array(self.est_trajectory)    # (N, 4)
        truth = np.array(self.truth_trajectory)  # (N, 4)

        np.save(self.output_dir / 'est_trajectory.npy',   est)
        np.save(self.output_dir / 'truth_trajectory.npy', truth)

        response.success = True
        response.message = f'Saved {len(est)} poses to {self.output_dir}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()