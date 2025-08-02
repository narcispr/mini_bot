import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import numpy as np
from mini_bot.utils.pid import PID
import time

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.1)
        self.declare_parameter('max_delta_pwm', 25.0)
        self.declare_parameter('v_pid.kp', 1.0)
        self.declare_parameter('v_pid.ki', 0.1)
        self.declare_parameter('v_pid.kd', 0.05)
        self.declare_parameter('v_pid.integral_max', 0.2)
        self.declare_parameter('w_pid.kp', 1.0)
        self.declare_parameter('w_pid.ki', 0.1)
        self.declare_parameter('w_pid.kd', 0.05)
        self.declare_parameter('w_pid.integral_max', 0.2)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('velocity_to_pwm_lut_left', 
            [-6.0, -255.0, -3.0, -225.0, -2.0, -200.0, -1.0, -100.0, 0.0, 0.0, 1.0, 100.0, 2.0, 200.0, 3.5, 225.0, 6.5, 255.0])
        self.declare_parameter('velocity_to_pwm_lut_right', 
            [-6.0, -255.0, -3.0, -225.0, -2.0, -200.0, -1.0, -100.0, 0.0, 0.0, 1.0, 100.0, 2.0, 200.0, 3.5, 225.0, 6.5, 255.0])

        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_delta_pwm = self.get_parameter('max_delta_pwm').get_parameter_value().double_value
        v_pid_kp = self.get_parameter('v_pid.kp').get_parameter_value().double_value
        v_pid_ki = self.get_parameter('v_pid.ki').get_parameter_value().double_value
        v_pid_kd = self.get_parameter('v_pid.kd').get_parameter_value().double_value
        v_pid_integral_max = self.get_parameter('v_pid.integral_max').get_parameter_value().double_value
        w_pid_kp = self.get_parameter('w_pid.kp').get_parameter_value().double_value
        w_pid_ki = self.get_parameter('w_pid.ki').get_parameter_value().double_value
        w_pid_kd = self.get_parameter('w_pid.kd').get_parameter_value().double_value
        w_pid_integral_max = self.get_parameter('w_pid.integral_max').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        
        lut_param_left = self.get_parameter('velocity_to_pwm_lut_left').get_parameter_value().double_array_value
        self.velocity_to_pwm_lut_left = np.array(lut_param_left).reshape(-1, 2)
        lut_param_right = self.get_parameter('velocity_to_pwm_lut_right').get_parameter_value().double_array_value
        self.velocity_to_pwm_lut_right = np.array(lut_param_right).reshape(-1, 2)

        # Store current velocities
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_velocity_time = self.get_clock().now()

        # Store desired velocities
        self.desired_v = 0.0
        self.desired_w = 0.0
        self.desired_last_time = self.get_clock().now()

        # PID controllers
        self.v_pid = PID(kp=v_pid_kp, ki=v_pid_ki, kd=v_pid_kd, integral_max=v_pid_integral_max)
        self.w_pid = PID(kp=w_pid_kp, ki=w_pid_ki, kd=w_pid_kd, integral_max=w_pid_integral_max)
        self.last_time = self.get_clock().now()

        # PWM smoothing
        self.last_left_pwm = 0.0
        self.last_right_pwm = 0.0

        # Publishers and subscribers
        self.pwm_pub = self.create_publisher(Int16MultiArray, 'pwm_setpoints', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Create a timer to apply velocities
        self.create_timer(self.dt, self.apply_velocity)

    def cmd_vel_callback(self, msg: Twist):
        # Log the received cmd_vel message
        self.desired_v = msg.linear.x
        self.desired_w = msg.angular.z
        self.desired_last_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        # Store current linear and angular velocities
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z
        self.last_velocity_time = self.get_clock().now()

    def apply_velocity(self):
        # self.get_logger().info(f'Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}')
      
        # Calculate time delta
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Get desired linear and angular velocities
        if (now - self.desired_last_time).nanoseconds / 1e9 > 1.0:
            # Desired velocity too old, stop the robot
            self.desired_v = 0.0
            self.desired_w = 0.0
        
        # set feed-forward velocities
        v_feed_forward = self.desired_v
        w_feed_forward = self.desired_w

        # Apply the PID controllers to the feed-forward velocities
        if (now - self.last_velocity_time).nanoseconds / 1e9 > 0.2:
            self.get_logger().warn('Last odometry message is too old, skipping PID control.')
            v_correction = 0.0
            w_correction = 0.0
        else:
            # Calculate errors
            v_error = v_feed_forward - self.current_v
            w_error = w_feed_forward - self.current_w

            # Get PID corrections
            v_correction = self.v_pid.update(v_error, dt)
            w_correction = self.w_pid.update(w_error, dt)

        # TODO: For now, we will not use PID corrections
        # v_correction = 0.0
        # w_correction = 0.0
        self.get_logger().debug(f'v_feed_forward: {v_feed_forward}, w_feed_forward: {w_feed_forward}, v_correction: {v_correction}, w_correction: {w_correction}')
        
        # Final velocities
        v = v_feed_forward + v_correction
        w = w_feed_forward + w_correction

        # Inverse kinematics for a differential drive robot
        vr_rads = ((2 * v) + (w * self.wheel_base)) / (2 * self.wheel_radius)
        vl_rads = ((2 * v) - (w * self.wheel_base)) / (2 * self.wheel_radius)

        # Interpolate PWM values from the LUT
        left_pwm = np.interp(vl_rads, self.velocity_to_pwm_lut_left[:, 0], self.velocity_to_pwm_lut_left[:, 1])
        right_pwm = np.interp(vr_rads, self.velocity_to_pwm_lut_right[:, 0], self.velocity_to_pwm_lut_right[:, 1])

        # TODO: For now, we will not smooth the PWM output
        # Smooth the PWM output
        # delta_left = left_pwm - self.last_left_pwm
        # if abs(delta_left) > self.max_delta_pwm:
        #     left_pwm = self.last_left_pwm + np.sign(delta_left) * self.max_delta_pwm

        # delta_right = right_pwm - self.last_right_pwm
        # if abs(delta_right) > self.max_delta_pwm:
        #     right_pwm = self.last_right_pwm + np.sign(delta_right) * self.max_delta_pwm
        
        # self.last_left_pwm = left_pwm
        # self.last_right_pwm = right_pwm

        # To avoind strange behavior when stopping the robot
        if self.desired_v == 0.0 and self.desired_w == 0.0:
            # If no desired velocity, stop the motors
            left_pwm = 0.0
            right_pwm = 0.0

        self.send_pwm_command(int(left_pwm), int(right_pwm))

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
