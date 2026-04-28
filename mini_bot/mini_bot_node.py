import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, PointCloud2, JointState, MagneticField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Int32, Header, Int16MultiArray

import math
import serial
import threading
import struct
import time

import numpy as np
import mini_bot.utils.bot_comms as coms

class MiniBotNode(Node):
    def __init__(self):
        super().__init__('mini_bot_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_timeout', 0.1)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('pulses_window', 5)
        self.declare_parameter('pulses_per_revolution', 20)
        self.declare_parameter('serial_no_data_warn_sec', 2.0)
        self.declare_parameter('diagnostics_period_sec', 5.0)

        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        serial_timeout = self.get_parameter('serial_timeout').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.pulses_window = self.get_parameter('pulses_window').get_parameter_value().integer_value
        self.pulses_per_revolution = self.get_parameter('pulses_per_revolution').get_parameter_value().integer_value
        self.serial_no_data_warn_sec = self.get_parameter('serial_no_data_warn_sec').get_parameter_value().double_value
        diagnostics_period = self.get_parameter('diagnostics_period_sec').get_parameter_value().double_value

        # Serial port for Arduino communications
        self.ser = serial.Serial(serial_port, 115200, timeout=serial_timeout, write_timeout=serial_timeout)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.serial_lock = threading.Lock()

        # Global vars
        self.all_pulses = np.zeros((2, self.pulses_window), dtype=np.uint8)
        self.pulses_idx = 0
        self.left_pwm = 0
        self.right_pwm = 0
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        
        self.last_pwm_cmd = self.get_clock().now()
        self.last_sensor_msg_monotonic = time.monotonic()
        self.last_no_data_warn_monotonic = 0.0
        self.rx_total = 0
        self.rx_by_id = {
            coms.ID_SENSOR_RANGE: 0,
            coms.ID_SENSOR_ENCODERS: 0,
            coms.ID_SENSOR_COMPASS: 0,
        }

        # ROS interfaces
        self.range_pub = self.create_publisher(Range, 'range', 10)
        self.pc_pub = self.create_publisher(PointCloud2, 'range_pointcloud', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.angle_pub = self.create_publisher(Int32, 'compass_angle', 10)
        self.create_subscription(Int16MultiArray, 'pwm_setpoints',  self.pwm_callback, 10)
        
        # Timers
        self.create_timer(self.dt, self.write_motors)
        if diagnostics_period > 0.0:
            self.create_timer(diagnostics_period, self.log_diagnostics)

        self.get_logger().info(
            f'Serial opened on {serial_port} @115200 (timeout={serial_timeout:.3f}s).'
        )
        # Start a separate thread for reading sensors
        self.read_sensors_thread = threading.Thread(target=self.read_sensors_loop, daemon=True)
        self.read_sensors_thread.start()

       
    def pwm_callback(self, msg: Int16MultiArray):
        # Entrada original
        self.left_pwm = np.clip(msg.data[0], -255, 255)
        self.right_pwm = np.clip(msg.data[1], -255, 255)
        self.last_pwm_cmd = self.get_clock().now()

    def read_sensors_loop(self):
        self.get_logger().info('Sensor read thread started.')
        while rclpy.ok():
            try:
                # Drain a burst of queued serial messages to avoid RX backlog.
                msgs_read = 0
                max_msgs_per_cycle = 50
                while msgs_read < max_msgs_per_cycle and self.read_sensors():
                    msgs_read += 1

                if msgs_read == max_msgs_per_cycle:
                    self.get_logger().warn('Serial backlog detected: reached max messages per cycle.')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                break
            except Exception as e:
                self.get_logger().error(f'Error reading sensors: {e}')
            # Avoid nested spinning from a worker thread; main thread already spins the node.
            time.sleep(max(0.001, self.dt / 10.0))

        self.get_logger().warn('Sensor read thread stopped.')
    
    def read_sensors(self):
        data = coms.read_message(self.ser, lock=self.serial_lock)

        if data:
            msg_id, payload = data
            self.last_sensor_msg_monotonic = time.monotonic()
            self.rx_total += 1
            if msg_id in self.rx_by_id:
                self.rx_by_id[msg_id] += 1

            if msg_id == coms.ID_SENSOR_RANGE and len(payload) == 2:
                value = payload[0] | (payload[1] << 8)
                self.publish_range(value)
            if msg_id == coms.ID_SENSOR_ENCODERS and len(payload) == 2:
                pulses_left = payload[0]
                pulses_right = payload[1]

                # Update wheel angles
                angle_increment_left = np.sign(self.left_pwm) * pulses_left * (2 * np.pi / self.pulses_per_revolution)
                angle_increment_right = np.sign(self.right_pwm) * pulses_right * (2 * np.pi / self.pulses_per_revolution)
                self.left_wheel_angle += angle_increment_left
                self.right_wheel_angle += angle_increment_right

                self.all_pulses[0, self.pulses_idx] = pulses_left
                self.all_pulses[1, self.pulses_idx] = pulses_right
                self.pulses_idx = (self.pulses_idx + 1) % self.pulses_window
                self.publish_joint_state()
            if msg_id == coms.ID_SENSOR_COMPASS and len(payload) == 6:
                self._process_compass_data(payload)
            return True
        else:
            now = time.monotonic()
            no_data_for = now - self.last_sensor_msg_monotonic
            should_warn = (
                no_data_for >= self.serial_no_data_warn_sec
                and (now - self.last_no_data_warn_monotonic) >= self.serial_no_data_warn_sec
            )
            if should_warn:
                self.last_no_data_warn_monotonic = now
                self.get_logger().warn(
                    f'No complete serial frame for {no_data_for:.2f}s '
                    f'(in_waiting={self.ser.in_waiting}, pwm=({self.left_pwm}, {self.right_pwm})).'
                )
            return False

    def write_motors(self):
        # If no PWM command has been sent in the last second, stop the motors
        now = self.get_clock().now()
        elapsed = (now - self.last_pwm_cmd).nanoseconds / 1e9
        if elapsed > 1.0:
            self.left_pwm = 0
            self.right_pwm = 0

        # Send PWM values to the motors
        msg_bytes = coms.build_pwm_message(self.left_pwm, self.right_pwm)
        
        try:
            with self.serial_lock:
                self.ser.write(msg_bytes)
        except serial.SerialTimeoutException:
            self.get_logger().warn('Serial write timeout while sending motor PWM command.')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def log_diagnostics(self):
        no_data_for = time.monotonic() - self.last_sensor_msg_monotonic
        self.get_logger().info(
            'Serial diagnostics: '
            f'no_data_for={no_data_for:.2f}s, '
            f'in_waiting={self.ser.in_waiting}, '
            f'rx_total={self.rx_total}, '
            f'range={self.rx_by_id.get(coms.ID_SENSOR_RANGE, 0)}, '
            f'encoders={self.rx_by_id.get(coms.ID_SENSOR_ENCODERS, 0)}, '
            f'compass={self.rx_by_id.get(coms.ID_SENSOR_COMPASS, 0)}, '
            f'pwm=({self.left_pwm}, {self.right_pwm})'
        )

    def publish_joint_state(self):
        # Publish joint state velocities (rad/s) to /joint_states
        # Calculate RPMs
        rpm_left = np.sum(self.all_pulses[0]) * (60.0 / (self.pulses_window * self.dt)) / self.pulses_per_revolution
        rpm_right = np.sum(self.all_pulses[1]) * (60.0 / (self.pulses_window * self.dt)) / self.pulses_per_revolution

        # Convert RPM to rad/s: rad/s = RPM * 2*pi / 60
        vel_left = np.sign(self.left_pwm) * rpm_left * 2 * np.pi / 60.0
        vel_right = np.sign(self.right_pwm) * rpm_right * 2 * np.pi / 60.0

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_angle, self.right_wheel_angle]
        joint_state.velocity = [vel_left, vel_right]
        self.joint_state_pub.publish(joint_state)

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

    def _process_compass_data(self, payload):
        # Check for error code from Arduino (all 0xFF)
        if all(p == 0xFF for p in payload):
            self.get_logger().warn('Invalid compass reading received from Arduino.')
            return

        # Unpack 3 signed 16-bit integers (little-endian)
        mag_x_raw, mag_y_raw, mag_z_raw = struct.unpack('<hhh', bytearray(payload))

        # Convert back to float and to Teslas (from micro-Teslas * 100)
        # The sensor reports in micro-Teslas (uT)
        mag_x = float(mag_x_raw) / 100.0 * 1e-6 # Convert from scaled uT to T
        mag_y = float(mag_y_raw) / 100.0 * 1e-6 # Convert from scaled uT to T
        mag_z = float(mag_z_raw) / 100.0 * 1e-6 # Convert from scaled uT to T

        # Publish MagneticField message
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = 'imu_link' # Or your appropriate frame
        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z
        # We don't have covariance data, so we leave it as zeros
        self.mag_pub.publish(mag_msg)

        # --- Calculate and publish yaw angle for backward compatibility ---
        # Calculate heading in degrees from the raw magnetometer data (in uT)
        heading_rad = math.atan2(mag_y_raw, mag_x_raw)
        heading_deg = math.degrees(heading_rad)
        if heading_deg < 0:
            heading_deg += 360

        angle_msg = Int32()
        angle_msg.data = int(heading_deg)
        self.angle_pub.publish(angle_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MiniBotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.ser.close()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()
