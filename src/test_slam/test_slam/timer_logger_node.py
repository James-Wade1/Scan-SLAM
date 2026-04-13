import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from pathlib import Path


class TimingLoggerNode(Node):
    def __init__(self):
        super().__init__('timing_logger_node')

        self.declare_parameter('output_dir', 'timing_results')
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.create_subscription(Float64MultiArray, '/frontend_timing',
                                 self.frontend_callback, 10)
        self.create_subscription(Float64MultiArray, '/loop_closure_timing',
                                 self.loop_closure_callback, 10)
        self.create_subscription(Float64MultiArray, '/backend_timing',
                                 self.backend_callback, 10)

        self.frontend_times       = []
        self.frontend_iterations  = []

        self.loop_closure_times      = []
        self.loop_closure_iterations = []

        self.backend_times       = []
        self.backend_iterations  = []
        self.backend_graph_sizes = []

    def frontend_callback(self, msg: Float64MultiArray):
        self.frontend_times.append(msg.data[0])
        self.frontend_iterations.append(msg.data[1])

    def loop_closure_callback(self, msg: Float64MultiArray):
        self.loop_closure_times.append(msg.data[0])
        self.loop_closure_iterations.append(msg.data[1])

    def backend_callback(self, msg: Float64MultiArray):
        self.backend_times.append(msg.data[0])
        self.backend_iterations.append(msg.data[1])
        self.backend_graph_sizes.append(msg.data[2])

    def save(self):
        np.save(self.output_dir / 'frontend_times.npy',          np.array(self.frontend_times))
        np.save(self.output_dir / 'frontend_iterations.npy',     np.array(self.frontend_iterations))
        np.save(self.output_dir / 'loop_closure_times.npy',      np.array(self.loop_closure_times))
        np.save(self.output_dir / 'loop_closure_iterations.npy', np.array(self.loop_closure_iterations))
        np.save(self.output_dir / 'backend_times.npy',           np.array(self.backend_times))
        np.save(self.output_dir / 'backend_iterations.npy',      np.array(self.backend_iterations))
        np.save(self.output_dir / 'backend_graph_sizes.npy',     np.array(self.backend_graph_sizes))
        self.get_logger().info(f'Saved timing data to {self.output_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = TimingLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()