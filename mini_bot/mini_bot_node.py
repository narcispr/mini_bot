import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_msgs.msg import Header
from std_msgs.msg import UInt8MultiArray

import math
import serial
import threading

import numpy as np
import mini_bot.utils.bot_comms as coms

class MiniBotNode(Node):
    def __init__(self):
        super().__init__('mini_bot_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('pulses_window', 5)
        self.declare_parameter('pulses_per_revolution', 20)

        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.pulses_window = self.get_parameter('pulses_window').get_parameter_value().integer_value
        self.pulses_per_revolution = self.get_parameter('pulses_per_revolution').get_parameter_value().integer_value

        # Serial port for Arduino communications
        self.ser = serial.Serial(serial_port, 115200, timeout=1)
        self.serial_lock = threading.Lock()

        # Global vars
        self.all_pulses = np.zeros((2, self.pulses_window), dtype=np.uint8)
        self.pulses_idx = 0
        self.left_pwm = 0
        self.right_pwm = 0
        self.l_dir = 1
        self.r_dir = 1
        self.last_pwm_cmd = self.get_clock().now()

        # ROS interfaces
        self.range_pub = self.create_publisher(Range, 'range', 10)
        self.pc_pub = self.create_publisher(PointCloud2, 'range_pointcloud', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.angle_pub = self.create_publisher(Int32, 'compass_angle', 10)
        self.create_subscription(UInt8MultiArray, 'pwm_setpoints',  self.pwm_callback, 10)
        
        # Timers
        self.create_timer(self.dt, self.write_motors)
        # Start a separate thread for reading sensors
        self.read_sensors_thread = threading.Thread(target=self.read_sensors_loop, daemon=True)
        self.read_sensors_thread.start()

       
    def pwm_callback(self, msg: UInt8MultiArray):
        # Entrada original
        self.left_pwm = msg.data[0]
        self.right_pwm = msg.data[1]
        self.last_pwm_cmd = self.get_clock().now()

    def read_sensors_loop(self):
        while rclpy.ok():
            try:
                self.read_sensors()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                break
            except Exception as e:
                self.get_logger().error(f'Error reading sensors: {e}')
            rclpy.spin_once(self, timeout_sec=self.dt/10.0)
    
    def read_sensors(self):
        with self.serial_lock:
            data = coms.read_message(self.ser)

        if data:
            msg_id, payload = data
            if msg_id == coms.ID_SENSOR_RANGE and len(payload) == 2:
                value = payload[0] | (payload[1] << 8)
                self.publish_range(value)
            if msg_id == coms.ID_SENSOR_ENCODERS and len(payload) == 2:
                self.all_pulses[0, self.pulses_idx] = payload[0]
                self.all_pulses[1, self.pulses_idx] = payload[1]
                self.pulses_idx = (self.pulses_idx + 1) % self.pulses_window
                self.publish_joint_state()
            if msg_id == coms.ID_SENSOR_COMPASS and len(payload) == 2:
                angle = payload[0] | (payload[1] << 8)
                angle_msg = Int32()
                angle_msg.data = int(angle)
                self.angle_pub.publish(angle_msg)
                # self.get_logger().info(f'Published compass angle: {angle_msg.data} degrees')

    def write_motors(self):
        # If no PWM command has been sent in the last second, stop the motors
        now = self.get_clock().now()
        elapsed = (now - self.last_pwm_cmd).nanoseconds / 1e9
        if elapsed > 1.0:
            self.left_pwm = 0
            self.right_pwm = 0

        # Send PWM values to the motors
        msg_bytes, self.l_dir, self.r_dir = coms.build_pwm_message(self.left_pwm, self.right_pwm)
        if self.l_dir == 0:
            self.l_dir = -1
        if self.r_dir == 0:
            self.r_dir = -1
        with self.serial_lock:
            self.ser.write(msg_bytes)
            # self.get_logger().info(f'Sent PWM: left={self.left_pwm}, right={self.right_pwm}')

    def publish_joint_state(self):
        # Publish joint state velocities (rad/s) to /joint_states
        # Calculate RPMs
        rpm_left = np.sum(self.all_pulses[0]) * (60.0 / (self.pulses_window * self.dt)) / self.pulses_per_revolution
        rpm_right = np.sum(self.all_pulses[1]) * (60.0 / (self.pulses_window * self.dt)) / self.pulses_per_revolution

        # Convert RPM to rad/s: rad/s = RPM * 2*pi / 60
        vel_left = self.l_dir * rpm_left * 2 * np.pi / 60.0
        vel_right = self.r_dir * rpm_right * 2 * np.pi / 60.0

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.velocity = [vel_left, vel_right]
        self.joint_state_pub.publish(joint_state)
        # self.get_logger().info(f'Published joint velocities: left={vel_left:.3f} rad/s, right={vel_right:.3f} rad/s')

    def publish_range(self, dist_cm):
        now = self.get_clock().now().to_msg()

        # --- 1. Publish the original Range message
        range_msg = Range()
        range_msg.header.stamp = now
        range_msg.header.frame_id = "range_link"
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.349  # ~20º in radians
        range_msg.min_range = 0.02
        range_msg.max_range = 3.0
        range_msg.range = dist_cm / 100.0
        self.range_pub.publish(range_msg)
        # self.get_logger().info(f'Published range: {range_msg.range:.2f} m')


        # --- 2. Simulate a 20º fan of points at given distance
        distance_m = dist_cm / 100.0
        if distance_m < range_msg.min_range or distance_m > range_msg.max_range:
            return  # don't publish if out of bounds

        num_points = 21
        fov_deg = 20.0
        fov_rad = math.radians(fov_deg)
        angle_min = -fov_rad / 2
        angle_max = fov_rad / 2

        points = []
        for i in range(num_points):
            angle = angle_min + i * (angle_max - angle_min) / (num_points - 1)
            x = distance_m * math.cos(angle)
            y = distance_m * math.sin(angle)
            z = 0.0
            points.append([x, y, z])

        header = Header()
        header.stamp = now
        header.frame_id = "range_link"

        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MiniBotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
