import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_srvs.srv import SetBool
import math

WAYPOINTS = [
    (2.56, 1.01, 1.17), #x, y, theta
    (2.31, 2.09, 2.88),
    (-2.10, 2.01, 3.49),
    (-2.22, -0.01, 5.52),
    (0.0, 0.0, 0.0),
    (2.91, 0.45, 5.34),
    (3.25, -0.23, 4.73),
    (3.20, -2.31, 3.12),
    (-3.13, -2.20, 1.59),
    (-3.17, -0.41, 0.26),
    (0.0, 0.0, 0.0)
]


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def wrap_angle(angle_rad):
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(z, w):
    return 2.0 * math.atan2(z, w)


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/real_pose', self.pose_callback, 10)
        self.pause_cmd_vel = self.create_service(SetBool, '/pause_cmd_vel', self.pause_cmd_vel_callback)

        # Controller settings
        self.max_linear_speed = 0.55     # m/s
        self.max_angular_speed = 1.8     # rad/s
        self.max_linear_accel = 0.8      # m/s^2
        self.max_angular_accel = 3.0     # rad/s^2
        self.kp_pos = 1.2
        self.kp_heading = 2.5
        self.kp_goal_yaw = 2.2
        self.pos_tolerance = 0.08        # m
        self.yaw_tolerance = 0.08        # rad

        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.waypoint_idx = 0
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_update_time = self.get_clock().now()
        self.publish_zero_max = 20

        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.publish_cmd_vel)
        self.publish_zero_counter = 0

        self.cmd_vel_paused = False
        self.get_logger().info('Waypoint follower ready. Waiting for /real_pose ...')

    def pause_cmd_vel_callback(self, request, response):
        self.cmd_vel_paused = request.data
        state_str = 'paused' if self.cmd_vel_paused else 'resumed'
        self.get_logger().info(f'Cmd_vel {state_str} via service call')
        response.success = True
        return response

    def pose_callback(self, msg):
        self.current_x = float(msg.pose.position.x)
        self.current_y = float(msg.pose.position.y)
        self.current_yaw = yaw_from_quaternion(msg.pose.orientation.z, msg.pose.orientation.w)

    def _publish(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.publisher_.publish(msg)

    def _advance_waypoint_if_reached(self):
        tx, ty, ttheta = WAYPOINTS[self.waypoint_idx]
        dx = tx - self.current_x
        dy = ty - self.current_y
        dist = math.hypot(dx, dy)

        if dist > self.pos_tolerance:
            return False

        yaw_err = wrap_angle(ttheta - self.current_yaw)
        if abs(yaw_err) > self.yaw_tolerance:
            return False

        self.waypoint_idx += 1
        if self.waypoint_idx >= len(WAYPOINTS):
            self.get_logger().info('Reached final waypoint. Restarting waypoint loop.')
            self.waypoint_idx = 0
        else:
            self.get_logger().info(f'Reached waypoint {self.waypoint_idx}/{len(WAYPOINTS)}')
        return True

    def publish_cmd_vel(self):

        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        if self.cmd_vel_paused:
            self.cmd_linear = 0.0
            self.cmd_angular = 0.0
            self._publish(0.0, 0.0)
            self.last_update_time = self.get_clock().now()
            return
        if self.publish_zero_counter < self.publish_zero_max:
            self.get_logger().info(f"Publishing zero cmd_vel for initialization ({self.publish_zero_counter}/{self.publish_zero_max})")
            self.cmd_linear = 0.0
            self.cmd_angular = 0.0
            self._publish(0.0, 0.0)
            self.publish_zero_counter += 1
            self.last_update_time = self.get_clock().now()
            return
        if dt <= 0.0:
            dt = 0.05

        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            self._publish(0.0, 0.0)
            return

        self._advance_waypoint_if_reached()

        tx, ty, ttheta = WAYPOINTS[self.waypoint_idx]
        dx = tx - self.current_x
        dy = ty - self.current_y
        dist = math.hypot(dx, dy)

        heading_to_target = math.atan2(dy, dx)
        heading_err = wrap_angle(heading_to_target - self.current_yaw)
        goal_yaw_err = wrap_angle(ttheta - self.current_yaw)

        if dist > self.pos_tolerance:
            v_des = self.max_linear_speed * math.tanh(self.kp_pos * dist)
            v_des *= max(0.0, math.cos(heading_err))
            w_des = clamp(self.kp_heading * heading_err, -self.max_angular_speed, self.max_angular_speed)
        else:
            v_des = 0.0
            w_des = clamp(self.kp_goal_yaw * goal_yaw_err, -self.max_angular_speed, self.max_angular_speed)

        dv_limit = self.max_linear_accel * dt
        dw_limit = self.max_angular_accel * dt
        self.cmd_linear += clamp(v_des - self.cmd_linear, -dv_limit, dv_limit)
        self.cmd_angular += clamp(w_des - self.cmd_angular, -dw_limit, dw_limit)

        self.cmd_linear = clamp(self.cmd_linear, -self.max_linear_speed, self.max_linear_speed)
        self.cmd_angular = clamp(self.cmd_angular, -self.max_angular_speed, self.max_angular_speed)

        self._publish(self.cmd_linear, self.cmd_angular)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVelPublisher()
    rclpy.spin(cmd_vel_publisher)
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()