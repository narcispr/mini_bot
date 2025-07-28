import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
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
            [[-15.0, -255.0], [-10.0, -180.0], [-5.0, -100.0], [-1.0, -30.0], [0.0, 0.0], [1.0, 30.0], [5.0, 100.0], [10.0, 180.0], [15.0, 255.0]])
        self.declare_parameter('velocity_to_pwm_lut_right', 
            [[-15.0, -255.0], [-10.0, -180.0], [-5.0, -100.0], [-1.0, -30.0], [0.0, 0.0], [1.0, 30.0], [5.0, 100.0], [10.0, 180.0], [15.0, 255.0]])

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
        
        lut_param_left = self.get_parameter('velocity_to_pwm_lut_left').get_parameter_value().value
        self.velocity_to_pwm_lut_left = np.array(lut_param_left)
        lut_param_right = self.get_parameter('velocity_to_pwm_lut_right').get_parameter_value().value
        self.velocity_to_pwm_lut_right = np.array(lut_param_right)

        # Store current velocities
        self.current_v = 0.0
        self.current_w = 0.0

        # PID controllers
        self.v_pid = PID(kp=v_pid_kp, ki=v_pid_ki, kd=v_pid_kd, integral_max=v_pid_integral_max)
        self.w_pid = PID(kp=w_pid_kp, ki=w_pid_ki, kd=w_pid_kd, integral_max=w_pid_integral_max)
        self.last_time = self.get_clock().now()

        # PWM smoothing
        self.last_left_pwm = 0.0
        self.last_right_pwm = 0.0

        # Calibration variables
        self.calibrating = False
        self.calibration_velocities_left = []
        self.calibration_velocities_right = []

        # Publishers and subscribers
        self.pwm_pub = self.create_publisher(Int16MultiArray, 'pwm_setpoints', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        # Service server
        self.calibrate_service = self.create_service(Trigger, 'calibrate', self.calibrate_callback)

    def joint_state_callback(self, msg: JointState):
        if self.calibrating:
            # Assuming joint_state.velocity[0] is left and [1] is right
            self.calibration_velocities_left.append(msg.velocity[0])
            self.calibration_velocities_right.append(msg.velocity[1])

    def cmd_vel_callback(self, msg: Twist):
        if self.calibrating:
            return # Ignore cmd_vel during calibration

        # Get desired linear and angular velocities
        v_feed_forward = msg.linear.x
        w_feed_forward = msg.angular.z

        # Calculate time delta
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Calculate errors
        v_error = v_feed_forward - self.current_v
        w_error = w_feed_forward - self.current_w

        # Get PID corrections
        v_correction = self.v_pid.update(v_error, dt)
        w_correction = self.w_pid.update(w_error, dt)

        # Final velocities
        v = v_feed_forward + v_correction
        w = w_feed_forward + w_correction

        # Inverse kinematics for a differential drive robot
        # vr = (2*v + w*L) / (2*R)
        # vl = (2*v - w*L) / (2*R)
        
        vr_rads = ((2 * v) + (w * self.wheel_base)) / (2 * self.wheel_radius)
        vl_rads = ((2 * v) - (w * self.wheel_base)) / (2 * self.wheel_radius)

        # Interpolate PWM values from the LUT
        left_pwm = np.interp(vl_rads, self.velocity_to_pwm_lut_left[:, 0], self.velocity_to_pwm_lut_left[:, 1])
        right_pwm = np.interp(vr_rads, self.velocity_to_pwm_lut_right[:, 0], self.velocity_to_pwm_lut_right[:, 1])

        # Smooth the PWM output
        delta_left = left_pwm - self.last_left_pwm
        if abs(delta_left) > self.max_delta_pwm:
            left_pwm = self.last_left_pwm + np.sign(delta_left) * self.max_delta_pwm

        delta_right = right_pwm - self.last_right_pwm
        if abs(delta_right) > self.max_delta_pwm:
            right_pwm = self.last_right_pwm + np.sign(delta_right) * self.max_delta_pwm
        
        self.last_left_pwm = left_pwm
        self.last_right_pwm = right_pwm

        self.send_pwm_command(int(left_pwm), int(right_pwm))

    def odom_callback(self, msg: Odometry):
        # Store current linear and angular velocities
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z

    def send_pwm_command(self, left_pwm, right_pwm):
        pwm_msg = Int16MultiArray()
        pwm_msg.data = [int(left_pwm), int(right_pwm)]
        self.pwm_pub.publish(pwm_msg)

    def calibrate_callback(self, request, response):
        self.get_logger().info('Starting calibration...')
        self.calibrating = True

        new_lut_left = []
        new_lut_right = []

        # Calibration for positive PWMs
        for pwm in range(0, 256, 64):
            self.get_logger().info(f'Calibrating PWM: {pwm}')
            self.send_pwm_command(pwm, pwm)
            self.calibration_velocities_left = []
            self.calibration_velocities_right = []
            
            start_time = self.get_clock().now()
            while (self.get_clock().now() - start_time).nanoseconds / 1e9 < 3.0:
                rclpy.spin_once(self, timeout_sec=0.1) # Process incoming messages
                self.send_pwm_command(pwm, pwm)

            # Collect data for the last second
            num_samples_last_sec = int(1.0 / self.dt) # Using dt from parameters

            if len(self.calibration_velocities_left) < num_samples_last_sec or len(self.calibration_velocities_right) < num_samples_last_sec:
                self.get_logger().error(f'Not enough samples collected for PWM {pwm}. Aborting calibration for this step.')
                response.success = False
                response.message = "Calibration aborted due to insufficient samples."
                self.calibrating = False
                return response
            
            avg_vel_left = np.mean(self.calibration_velocities_left[-num_samples_last_sec:])
            avg_vel_right = np.mean(self.calibration_velocities_right[-num_samples_last_sec:])
            
            self.get_logger().info(f'PWM: {pwm}, Left Wheel: {avg_vel_left:.2f} rad/s ({avg_vel_left * 60 / (2 * np.pi):.2f} RPM), Right Wheel: {avg_vel_right:.2f} rad/s ({avg_vel_right * 60 / (2 * np.pi):.2f} RPM), Averaged measures: {num_samples_last_sec}')

            new_lut_left.append([avg_vel_left, float(pwm)])
            new_lut_right.append([avg_vel_right, float(pwm)])

        # Calibration for negative PWMs
        for pwm in range(0, -256, -64):
            self.get_logger().info(f'Calibrating PWM: {pwm}')
            self.send_pwm_command(pwm, pwm)
            self.calibration_velocities_left = []
            self.calibration_velocities_right = []
            
            start_time = self.get_clock().now()
            while (self.get_clock().now() - start_time).nanoseconds / 1e9 < 3.0:
                rclpy.spin_once(self, timeout_sec=0.1) # Process incoming messages
                self.send_pwm_command(pwm, pwm)

            # Collect data for the last second
            num_samples_last_sec = int(1.0 / self.dt) # Using dt from parameters
            
            if len(self.calibration_velocities_left) < 2*num_samples_last_sec or len(self.calibration_velocities_right) < 2*num_samples_last_sec:
                self.get_logger().error(f'Not enough samples collected for PWM {pwm}. Aborting calibration for this step.')
                response.success = False
                response.message = "Calibration aborted due to insufficient samples."
                self.calibrating = False
                return response

            avg_vel_left = np.mean(self.calibration_velocities_left[2*num_samples_last_sec:])
            avg_vel_right = np.mean(self.calibration_velocities_right[2*num_samples_last_sec:])

            self.get_logger().info(f'PWM: {pwm}, Left Wheel: {avg_vel_left:.2f} rad/s ({avg_vel_left * 60 / (2 * np.pi):.2f} RPM), Right Wheel: {avg_vel_right:.2f} rad/s ({avg_vel_right * 60 / (2 * np.pi):.2f} RPM), Averaged measures: {len(self.calibration_velocities_right[2*num_samples_last_sec:])}')
            
            new_lut_left.append([avg_vel_left, float(pwm)])
            new_lut_right.append([avg_vel_right, float(pwm)])

        # Stop motors
        self.send_pwm_command(0, 0)
        self.calibrating = False

        self.get_logger().info('Calibration complete.')
        self.get_logger().info('New Left Wheel LUT (velocity_rads, pwm):')
        for row in new_lut_left:
            self.get_logger().info(f'  {row}')
        self.get_logger().info('New Right Wheel LUT (velocity_rads, pwm):')
        for row in new_lut_right:
            self.get_logger().info(f'  {row}')

        self.get_logger().warn('Please manually update the `velocity_to_pwm_lut_left` and `velocity_to_pwm_lut_right` parameters in `config/mini_bot.yaml` with these new values.')

        response.success = True
        response.message = "Calibration completed. New LUTs printed to console. Please update config file manually."
        return response

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

    def cmd_vel_callback(self, msg: Twist):
        # Get desired linear and angular velocities
        v_feed_forward = msg.linear.x
        w_feed_forward = msg.angular.z

        # Calculate time delta
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Calculate errors
        v_error = v_feed_forward - self.current_v
        w_error = w_feed_forward - self.current_w

        # Get PID corrections
        v_correction = self.v_pid.update(v_error, dt)
        w_correction = self.w_pid.update(w_error, dt)

        # Final velocities
        v = v_feed_forward + v_correction
        w = w_feed_forward + w_correction

        # Inverse kinematics for a differential drive robot
        # vr = (2*v + w*L) / (2*R)
        # vl = (2*v - w*L) / (2*R)
        
        vr_rads = ((2 * v) + (w * self.wheel_base)) / (2 * self.wheel_radius)
        vl_rads = ((2 * v) - (w * self.wheel_base)) / (2 * self.wheel_radius)

        # Interpolate PWM values from the LUT
        left_pwm = np.interp(vl_rads, self.velocity_to_pwm_lut_left[:, 0], self.velocity_to_pwm_lut_left[:, 1])
        right_pwm = np.interp(vr_rads, self.velocity_to_pwm_lut_right[:, 0], self.velocity_to_pwm_lut_right[:, 1])

        # Smooth the PWM output
        delta_left = left_pwm - self.last_left_pwm
        if abs(delta_left) > self.max_delta_pwm:
            left_pwm = self.last_left_pwm + np.sign(delta_left) * self.max_delta_pwm

        delta_right = right_pwm - self.last_right_pwm
        if abs(delta_right) > self.max_delta_pwm:
            right_pwm = self.last_right_pwm + np.sign(delta_right) * self.max_delta_pwm
        
        self.last_left_pwm = left_pwm
        self.last_right_pwm = right_pwm

        self.send_pwm_command(int(left_pwm), int(right_pwm))

    def odom_callback(self, msg: Odometry):
        # Store current linear and angular velocities
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z

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
