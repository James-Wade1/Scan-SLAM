import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Quaternion
from nav_msgs.msg import Odometry
import numpy as np

class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        self.declare_parameter('linear_vel_noise_std', 0.01)
        self.declare_parameter('angular_vel_noise_std', 0.05)
        self.declare_parameter('noise_seed', 42)

        linear_noise_std  = self.get_parameter('linear_vel_noise_std').get_parameter_value().double_value
        angular_noise_std = self.get_parameter('angular_vel_noise_std').get_parameter_value().double_value
        noise_seed        = self.get_parameter('noise_seed').get_parameter_value().integer_value

        self.linear_noise_std  = linear_noise_std
        self.angular_noise_std = angular_noise_std
        np.random.seed(noise_seed)

        self.cmd_vel_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

    def cmd_vel_callback(self, cmd_vel_msg: TwistStamped):
            now = self.get_clock().now()

            if self.last_time is None:
                self.last_time = now
                return

            dt = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now

            # Add noise to the velocity commands before integrating
            v = cmd_vel_msg.twist.linear.x + np.random.normal(0, self.linear_noise_std)
            w = cmd_vel_msg.twist.angular.z + np.random.normal(0, self.angular_noise_std)

            # Integrate
            self.x += v * np.cos(self.theta) * dt
            self.y += v * np.sin(self.theta) * dt
            self.theta += w * dt

            # Publish
            odom_msg = Odometry()
            odom_msg.header.stamp = now.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.orientation = self.theta_to_quaternion(self.theta)
            odom_msg.twist.twist = cmd_vel_msg.twist
            self.odom_pub.publish(odom_msg)

    def theta_to_quaternion(self, theta):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=np.sin(theta / 2),
            w=np.cos(theta / 2)
        )

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()