import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
import numpy as np
from mini_bot.utils.pid import PID
import tf2_geometry_msgs  # noqa: F401 - registers geometry_msgs transforms with tf2
import tf2_ros

import math

from mini_bot.utils.utils import quaternion_to_euler


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Declare parameters
        self.declare_parameter('max_v', 0.5)
        self.declare_parameter('max_w', 3.14)
        self.declare_parameter('max_pwm', 255.0)
        self.declare_parameter('cmd_vel_timeout', 0.2)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('gain_pid', 0.5)
        self.declare_parameter('pid_v.kp', 1.0)
        self.declare_parameter('pid_v.ki', 0.0)
        self.declare_parameter('pid_v.kd', 0.2)
        self.declare_parameter('pid_v.integral_max', 0.1)
        self.declare_parameter('pid_w.kp', 1.0)
        self.declare_parameter('pid_w.ki', 0.0)
        self.declare_parameter('pid_w.kd', 0.2)
        self.declare_parameter('pid_w.integral_max', 0.1)
        self.declare_parameter('angle_pid.kp', 1.0)
        self.declare_parameter('angle_pid.ki', 0.1)
        self.declare_parameter('angle_pid.kd', 0.05)
        self.declare_parameter('angle_pid.integral_max', 0.2)
        self.declare_parameter('max_v_goal', 0.2)
        self.declare_parameter('max_w_goal', 1.0)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('goal_angle_tolerance', 0.3)

        self.declare_parameter(
            'velocity_to_pwm_lut_v',
            [0.0, 0.0, 0.01, 125.0, 0.25, 150.0, 0.4, 200.0, 0.5, 225.0, 0.6, 255.0],
        )
        self.declare_parameter(
            'velocity_to_pwm_lut_w',
            [0.0, 0.0, 0.01, 125.0, 0.85, 135.0, 2.3, 155.0, 4.0, 185.0],
        )

        # Get parameters
        self.max_v = abs(self.get_parameter('max_v').get_parameter_value().double_value)
        self.max_w = abs(self.get_parameter('max_w').get_parameter_value().double_value)
        self.max_pwm = self.get_parameter('max_pwm').get_parameter_value().double_value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.gain_pid = self.get_parameter('gain_pid').get_parameter_value().double_value

        pid_v_kp = self.get_parameter('pid_v.kp').get_parameter_value().double_value
        pid_v_ki = self.get_parameter('pid_v.ki').get_parameter_value().double_value
        pid_v_kd = self.get_parameter('pid_v.kd').get_parameter_value().double_value
        pid_v_integral_max = self.get_parameter('pid_v.integral_max').get_parameter_value().double_value
        pid_w_kp = self.get_parameter('pid_w.kp').get_parameter_value().double_value
        pid_w_ki = self.get_parameter('pid_w.ki').get_parameter_value().double_value
        pid_w_kd = self.get_parameter('pid_w.kd').get_parameter_value().double_value
        pid_w_integral_max = self.get_parameter('pid_w.integral_max').get_parameter_value().double_value
        angle_pid_kp = self.get_parameter('angle_pid.kp').get_parameter_value().double_value
        angle_pid_ki = self.get_parameter('angle_pid.ki').get_parameter_value().double_value
        angle_pid_kd = self.get_parameter('angle_pid.kd').get_parameter_value().double_value
        angle_pid_integral_max = self.get_parameter(
            'angle_pid.integral_max'
        ).get_parameter_value().double_value
        self.max_v_goal = abs(self.get_parameter('max_v_goal').get_parameter_value().double_value)
        self.max_w_goal = abs(self.get_parameter('max_w_goal').get_parameter_value().double_value)
        self.goal_tolerance = abs(self.get_parameter('goal_tolerance').get_parameter_value().double_value)
        self.goal_angle_tolerance = abs(
            self.get_parameter('goal_angle_tolerance').get_parameter_value().double_value
        )

        self.velocity_to_pwm_lut_v = self.load_lut('velocity_to_pwm_lut_v')
        self.velocity_to_pwm_lut_w = self.load_lut('velocity_to_pwm_lut_w')

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Store current odometry
        self.current_v = 0.0
        self.current_w = 0.0
        self.current_pose = None
        self.last_odom_time = None

        # Store desired velocities
        self.desired_v = 0.0
        self.desired_w = 0.0
        self.desired_last_time = self.get_clock().now()

        # Go-to-goal state
        self.goal_pose = None
        self.execute_goto = False

        # PID controllers
        self.pid_v = PID(kp=pid_v_kp, ki=pid_v_ki, kd=pid_v_kd, integral_max=pid_v_integral_max)
        self.pid_w = PID(kp=pid_w_kp, ki=pid_w_ki, kd=pid_w_kd, integral_max=pid_w_integral_max)
        self.angle_pid = PID(
            kp=angle_pid_kp,
            ki=angle_pid_ki,
            kd=angle_pid_kd,
            integral_max=angle_pid_integral_max,
        )
        self.last_pid_time = self.get_clock().now()
        self.last_goal_time = self.get_clock().now()

        # Publishers and subscribers
        self.pwm_pub = self.create_publisher(Int16MultiArray, 'pwm_setpoints', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # Timers
        self.create_timer(self.dt, self.goto_goal_controller)
        self.create_timer(self.dt, self.apply_velocity)

    def load_lut(self, parameter_name):
        lut_param = self.get_parameter(parameter_name).value
        lut = np.array(lut_param, dtype=float)
        if lut.size == 0 or lut.size % 2 != 0:
            raise ValueError(f'{parameter_name} must contain velocity/PWM pairs')

        lut = lut.reshape(-1, 2)
        if np.any(np.diff(lut[:, 0]) < 0.0):
            raise ValueError(f'{parameter_name} velocity values must be sorted')

        return lut

    def cmd_vel_callback(self, msg: Twist):
        if self.execute_goto:
            self.get_logger().info('Cancelling go-to-goal because cmd_vel was received.')
        self.execute_goto = False
        self.goal_pose = None
        self.desired_v = float(np.clip(msg.linear.x, -self.max_v, self.max_v))
        self.desired_w = float(np.clip(msg.angular.z, -self.max_w, self.max_w))
        self.desired_last_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z
        self.current_pose = msg.pose.pose
        self.last_odom_time = self.get_clock().now()

    def goal_pose_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.execute_goto = True
        self.set_desired_stop()
        self.last_goal_time = self.get_clock().now()
        self.reset_pid(self.angle_pid)
        self.get_logger().info(
            f'New goal received: {msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}'
        )

    def goto_goal_controller(self):
        if not self.execute_goto or self.goal_pose is None:
            return

        now = self.get_clock().now()
        dt = max((now - self.last_goal_time).nanoseconds / 1e9, 1e-6)
        self.last_goal_time = now

        odom_is_current = (
            self.current_pose is not None
            and self.last_odom_time is not None
            and (now - self.last_odom_time).nanoseconds / 1e9 <= self.cmd_vel_timeout
        )
        if not odom_is_current:
            self.set_desired_stop(now)
            return

        try:
            if self.goal_pose.header.frame_id in ('', 'odom'):
                goal_in_odom = self.goal_pose
            else:
                goal_in_odom = self.tf_buffer.transform(
                    self.goal_pose,
                    'odom',
                    timeout=Duration(seconds=0.5),
                )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform goal to odom frame: {e}')
            self.set_desired_stop(now)
            return

        dx = goal_in_odom.pose.position.x - self.current_pose.position.x
        dy = goal_in_odom.pose.position.y - self.current_pose.position.y
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        _, _, current_theta = quaternion_to_euler((
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        ))

        angle_error = self.normalize_angle(angle_to_goal - current_theta)

        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info('Goal reached!')
            self.execute_goto = False
            self.goal_pose = None
            self.set_desired_stop(now)
            self.reset_pid(self.angle_pid)
            return

        if abs(angle_error) > self.goal_angle_tolerance:
            self.desired_v = 0.0
        else:
            self.desired_v = min(self.max_v_goal, self.max_v)

        self.desired_w = self.angle_pid.update(angle_error, dt)
        self.desired_w = float(np.clip(self.desired_w, -self.max_w_goal, self.max_w_goal))
        self.desired_w = float(np.clip(self.desired_w, -self.max_w, self.max_w))
        self.desired_last_time = now

    def apply_velocity(self):
        now = self.get_clock().now()
        dt = max((now - self.last_pid_time).nanoseconds / 1e9, 1e-6)
        self.last_pid_time = now

        if (
            not self.execute_goto
            and (now - self.desired_last_time).nanoseconds / 1e9 > self.cmd_vel_timeout
        ):
            self.desired_v = 0.0
            self.desired_w = 0.0

        v_pid = 0.0
        w_pid = 0.0
        odom_is_current = (
            self.last_odom_time is not None
            and (now - self.last_odom_time).nanoseconds / 1e9 <= self.cmd_vel_timeout
        )
        if odom_is_current:
            v_error = self.desired_v - self.current_v
            w_error = self.desired_w - self.current_w
            v_pid = self.pid_v.update(v_error, dt)
            w_pid = self.pid_w.update(w_error, dt)
            v_pid = float(np.clip(v_pid, -self.max_v, self.max_v))
            w_pid = float(np.clip(w_pid, -self.max_w, self.max_w))

        v = self.desired_v + self.gain_pid * v_pid
        w = self.desired_w + self.gain_pid * w_pid
        v = float(np.clip(v, -self.max_v, self.max_v))
        w = float(np.clip(w, -self.max_w, self.max_w))

        left_pwm, right_pwm = self.velocity_to_wheel_pwm(v, w)

        left_pwm = float(np.clip(left_pwm, -self.max_pwm, self.max_pwm))
        right_pwm = float(np.clip(right_pwm, -self.max_pwm, self.max_pwm))

        self.send_pwm_command(round(left_pwm), round(right_pwm))

    def velocity_to_wheel_pwm(self, v, w):
        if v == 0.0 and w == 0.0:
            return 0.0, 0.0

        v_pwm = self.lookup_signed_pwm(v, self.velocity_to_pwm_lut_v)
        w_pwm = self.lookup_signed_pwm(w, self.velocity_to_pwm_lut_w)
        w_pwm = float(np.clip(w_pwm, -self.max_pwm, self.max_pwm))

        # Give angular velocity priority: keep |v_pwm| + |w_pwm| <= max_pwm.
        max_v_pwm = max(0.0, self.max_pwm - abs(w_pwm))
        v_pwm = float(np.clip(v_pwm, -max_v_pwm, max_v_pwm))

        left_pwm = v_pwm - w_pwm
        right_pwm = v_pwm + w_pwm
        return left_pwm, right_pwm

    def lookup_signed_pwm(self, velocity, lut):
        pwm = np.interp(abs(velocity), lut[:, 0], lut[:, 1])
        return float(np.sign(velocity) * pwm)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def reset_pid(self, pid):
        pid.previous_error = 0.0
        pid.integral = 0.0

    def set_desired_stop(self, now=None):
        self.desired_v = 0.0
        self.desired_w = 0.0
        self.desired_last_time = now if now is not None else self.get_clock().now()

    def send_pwm_command(self, left_pwm, right_pwm):
        pwm_msg = Int16MultiArray()
        pwm_msg.data = [int(left_pwm), int(right_pwm)]
        self.pwm_pub.publish(pwm_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
