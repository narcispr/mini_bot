import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
import numpy as np
from mini_bot.utils.pid import PID


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

        self.velocity_to_pwm_lut_v = self.load_lut('velocity_to_pwm_lut_v')
        self.velocity_to_pwm_lut_w = self.load_lut('velocity_to_pwm_lut_w')

        # Store current velocities
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_odom_time = None

        # Store desired velocities
        self.desired_v = 0.0
        self.desired_w = 0.0
        self.desired_last_time = self.get_clock().now()

        # PID controllers
        self.pid_v = PID(kp=pid_v_kp, ki=pid_v_ki, kd=pid_v_kd, integral_max=pid_v_integral_max)
        self.pid_w = PID(kp=pid_w_kp, ki=pid_w_ki, kd=pid_w_kd, integral_max=pid_w_integral_max)
        self.last_pid_time = self.get_clock().now()

        # Publishers and subscribers
        self.pwm_pub = self.create_publisher(Int16MultiArray, 'pwm_setpoints', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Timers
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
        self.desired_v = float(np.clip(msg.linear.x, -self.max_v, self.max_v))
        self.desired_w = float(np.clip(msg.angular.z, -self.max_w, self.max_w))
        self.desired_last_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z
        self.last_odom_time = self.get_clock().now()

    def apply_velocity(self):
        now = self.get_clock().now()
        dt = max((now - self.last_pid_time).nanoseconds / 1e9, 1e-6)
        self.last_pid_time = now

        if (now - self.desired_last_time).nanoseconds / 1e9 > self.cmd_vel_timeout:
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
