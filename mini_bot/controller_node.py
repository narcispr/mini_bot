import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import Trigger
import numpy as np
from mini_bot.utils.pid import PID
import time
import tf2_ros
from mini_bot.utils.utils import quaternion_to_euler

import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.16)
        self.declare_parameter('max_delta_pwm', 25.0)
        self.declare_parameter('dt', 0.05)

        # PID parameters for velocity control
        self.declare_parameter('v_pid.kp', 1.0)
        self.declare_parameter('v_pid.ki', 0.1)
        self.declare_parameter('v_pid.kd', 0.05)
        self.declare_parameter('v_pid.integral_max', 0.2)
        self.declare_parameter('w_pid.kp', 1.0)
        self.declare_parameter('w_pid.ki', 0.1)
        self.declare_parameter('w_pid.kd', 0.05)
        self.declare_parameter('w_pid.integral_max', 0.2)

        # PID parameters for go-to-goal control
        self.declare_parameter('dist_pid.kp', 1.0)
        self.declare_parameter('dist_pid.ki', 0.1)
        self.declare_parameter('dist_pid.kd', 0.05)
        self.declare_parameter('dist_pid.integral_max', 0.2)
        self.declare_parameter('angle_pid.kp', 1.0)
        self.declare_parameter('angle_pid.ki', 0.1)
        self.declare_parameter('angle_pid.kd', 0.05)
        self.declare_parameter('angle_pid.integral_max', 0.2)
        self.declare_parameter('max_v_goal', 0.2)
        self.declare_parameter('max_w_goal', 1.0)

        self.declare_parameter('velocity_to_pwm_lut_left', 
            [-6.0, -255.0, -3.0, -225.0, -2.0, -200.0, -1.0, -100.0, 0.0, 0.0, 1.0, 100.0, 2.0, 200.0, 3.5, 225.0, 6.5, 255.0])
        self.declare_parameter('velocity_to_pwm_lut_right', 
            [-6.0, -255.0, -3.0, -225.0, -2.0, -200.0, -1.0, -100.0, 0.0, 0.0, 1.0, 100.0, 2.0, 200.0, 3.5, 225.0, 6.5, 255.0])

        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_delta_pwm = self.get_parameter('max_delta_pwm').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        
        v_pid_kp = self.get_parameter('v_pid.kp').get_parameter_value().double_value
        v_pid_ki = self.get_parameter('v_pid.ki').get_parameter_value().double_value
        v_pid_kd = self.get_parameter('v_pid.kd').get_parameter_value().double_value
        v_pid_integral_max = self.get_parameter('v_pid.integral_max').get_parameter_value().double_value
        w_pid_kp = self.get_parameter('w_pid.kp').get_parameter_value().double_value
        w_pid_ki = self.get_parameter('w_pid.ki').get_parameter_value().double_value
        w_pid_kd = self.get_parameter('w_pid.kd').get_parameter_value().double_value
        w_pid_integral_max = self.get_parameter('w_pid.integral_max').get_parameter_value().double_value

        dist_pid_kp = self.get_parameter('dist_pid.kp').get_parameter_value().double_value
        dist_pid_ki = self.get_parameter('dist_pid.ki').get_parameter_value().double_value
        dist_pid_kd = self.get_parameter('dist_pid.kd').get_parameter_value().double_value
        dist_pid_integral_max = self.get_parameter('dist_pid.integral_max').get_parameter_value().double_value
        angle_pid_kp = self.get_parameter('angle_pid.kp').get_parameter_value().double_value
        angle_pid_ki = self.get_parameter('angle_pid.ki').get_parameter_value().double_value
        angle_pid_kd = self.get_parameter('angle_pid.kd').get_parameter_value().double_value
        angle_pid_integral_max = self.get_parameter('angle_pid.integral_max').get_parameter_value().double_value
        self.max_v_goal = self.get_parameter('max_v_goal').get_parameter_value().double_value
        self.max_w_goal = self.get_parameter('max_w_goal').get_parameter_value().double_value

        lut_param_left = self.get_parameter('velocity_to_pwm_lut_left').get_parameter_value().double_array_value
        self.velocity_to_pwm_lut_left = np.array(lut_param_left).reshape(-1, 2)
        lut_param_right = self.get_parameter('velocity_to_pwm_lut_right').get_parameter_value().double_array_value
        self.velocity_to_pwm_lut_right = np.array(lut_param_right).reshape(-1, 2)

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Store current state
        self.current_v = 0.0
        self.current_w = 0.0
        self.current_pose = None
        self.last_velocity_time = self.get_clock().now()

        # Store desired velocities
        self.desired_v = 0.0
        self.desired_w = 0.0
        self.desired_last_time = self.get_clock().now()

        # Go-to-goal variables
        self.goal_pose = None
        self.execute_goto = False

        # PID controllers
        self.v_pid = PID(kp=v_pid_kp, ki=v_pid_ki, kd=v_pid_kd, integral_max=v_pid_integral_max)
        self.w_pid = PID(kp=w_pid_kp, ki=w_pid_ki, kd=w_pid_kd, integral_max=w_pid_integral_max)
        self.dist_pid = PID(kp=dist_pid_kp, ki=dist_pid_ki, kd=dist_pid_kd, integral_max=dist_pid_integral_max)
        self.angle_pid = PID(kp=angle_pid_kp, ki=angle_pid_ki, kd=angle_pid_kd, integral_max=angle_pid_integral_max)
        self.last_time = self.get_clock().now()

        # PWM smoothing
        # self.last_left_pwm = 0.0
        # self.last_right_pwm = 0.0

        # Publishers and subscribers
        self.pwm_pub = self.create_publisher(Int16MultiArray, 'pwm_setpoints', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        
        # Timers
        self.create_timer(self.dt, self.apply_velocity)
        self.create_timer(self.dt, self.goto_goal_controller)

    def goal_pose_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.execute_goto = True
        self.get_logger().info(f'New goal received: {msg.pose.position.x}, {msg.pose.position.y}')

    def cmd_vel_callback(self, msg: Twist):
        # Manual control has priority, so disable go-to-goal
        self.execute_goto = False
        self.desired_v = msg.linear.x
        self.desired_w = msg.angular.z
        self.desired_last_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z
        self.current_pose = msg.pose.pose
        self.last_velocity_time = self.get_clock().now()

    def goto_goal_controller(self):
        if not self.execute_goto or self.goal_pose is None or self.current_pose is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        # --- Transform goal to odom frame if necessary ---
        try:
            if self.goal_pose.header.frame_id != 'odom':
                goal_in_odom = self.tf_buffer.transform(self.goal_pose, 'odom', rclpy.duration.Duration(seconds=0.5))
            else:
                goal_in_odom = self.goal_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform goal to odom frame: {e}')
            return

        # --- Calculate distance and angle to goal ---
        dx = goal_in_odom.pose.position.x - self.current_pose.position.x
        dy = goal_in_odom.pose.position.y - self.current_pose.position.y
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        # --- Get current robot orientation ---
        _, _, current_theta = quaternion_to_euler((
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ))

        # --- Calculate angle error ---
        angle_error = angle_to_goal - current_theta
        # Normalize angle to [-pi, pi]
        angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

        # --- Check if goal is reached ---
        if distance_to_goal < 0.05: # 5 cm tolerance
            self.get_logger().info('Goal reached!')
            self.execute_goto = False
            self.goal_pose = None
            self.desired_v = 0.0
            self.desired_w = 0.0
            # self.dist_pid.reset()
            # self.angle_pid.reset()
            return

        # --- Use PID controllers to get desired velocities ---
        # If the angle error is large, prioritize turning
        if abs(angle_error) > 0.3: # ~17 degrees
            self.desired_v = 0.0
        else:
            # Use PID for distance to get linear velocity
            self.desired_v = 0.2 # self.dist_pid.update(distance_to_goal, dt)
            # Clamp the velocity
            self.desired_v = np.clip(self.desired_v, -self.max_v_goal, self.max_v_goal)

        # Use PID for angle to get angular velocity
        self.desired_w = self.angle_pid.update(angle_error, dt)
        # Clamp the velocity
        self.desired_w = np.clip(self.desired_w, -self.max_w_goal, self.max_w_goal)
        self.get_logger().info(f'Goto goal: v={self.desired_v}, w={self.desired_w}, distance={distance_to_goal}, angle_error={angle_error}')
        self.desired_last_time = now


    def apply_velocity(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # If not in go-to-goal mode, check for old cmd_vel
        if not self.execute_goto and (now - self.desired_last_time).nanoseconds / 1e9 > 1.0:
            self.desired_v = 0.0
            self.desired_w = 0.0
        
        v_feed_forward = self.desired_v
        w_feed_forward = self.desired_w

        # Apply the PID controllers for motor velocity
        if (now - self.last_velocity_time).nanoseconds / 1e9 > 0.2:
            self.get_logger().warn('Last odometry message is too old, skipping PID correction.')
            v_correction = 0.0
            w_correction = 0.0
        else:
            v_error = v_feed_forward - self.current_v
            w_error = w_feed_forward - self.current_w
            v_correction = self.v_pid.update(v_error, dt)
            w_correction = self.w_pid.update(w_error, dt)

        v = v_feed_forward + v_correction
        w = w_feed_forward + w_correction

        # Inverse kinematics
        vr_rads = ((2 * v) + (w * self.wheel_base)) / (2 * self.wheel_radius)
        vl_rads = ((2 * v) - (w * self.wheel_base)) / (2 * self.wheel_radius)

        # Interpolate PWM
        left_pwm = np.interp(vl_rads, self.velocity_to_pwm_lut_left[:, 0], self.velocity_to_pwm_lut_left[:, 1])
        right_pwm = np.interp(vr_rads, self.velocity_to_pwm_lut_right[:, 0], self.velocity_to_pwm_lut_right[:, 1])

        # Stop motors if desired velocity is zero
        if self.desired_v == 0.0 and self.desired_w == 0.0:
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