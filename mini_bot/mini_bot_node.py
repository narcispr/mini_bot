import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Range, PointCloud2, JointState, MagneticField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Int32, Header, Int16MultiArray
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

import math
from pathlib import Path
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
        self.declare_float_parameter('serial_timeout', 0.1)
        self.declare_float_parameter('dt', 0.05)
        self.declare_parameter('pulses_window', 5)
        self.declare_parameter('pulses_per_revolution', 20)
        self.declare_float_parameter('serial_no_data_warn_sec', 2.0)
        self.declare_float_parameter('diagnostics_period_sec', 5.0)
        self.declare_float_parameter('mag_x_offset_uT', 0.0)
        self.declare_float_parameter('mag_y_offset_uT', 0.0)
        self.declare_float_parameter('mag_x_scale', 1.0)
        self.declare_float_parameter('mag_y_scale', 1.0)
        self.declare_float_parameter('mag_heading_offset_deg', 0.0)
        self.declare_float_parameter('mag_calibration_angular_velocity', 1.0)
        self.declare_float_parameter('mag_calibration_duration_sec', 10.0)
        self.declare_parameter('mag_calibration_config_file', '')

        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        serial_timeout = self.get_float_parameter('serial_timeout')
        self.dt = self.get_float_parameter('dt')
        self.pulses_window = self.get_parameter('pulses_window').get_parameter_value().integer_value
        self.pulses_per_revolution = self.get_parameter('pulses_per_revolution').get_parameter_value().integer_value
        self.serial_no_data_warn_sec = self.get_float_parameter('serial_no_data_warn_sec')
        diagnostics_period = self.get_float_parameter('diagnostics_period_sec')
        self.mag_x_offset_uT = self.get_float_parameter('mag_x_offset_uT')
        self.mag_y_offset_uT = self.get_float_parameter('mag_y_offset_uT')
        self.mag_x_scale = self.get_float_parameter('mag_x_scale')
        self.mag_y_scale = self.get_float_parameter('mag_y_scale')
        self.mag_heading_offset_deg = self.get_float_parameter('mag_heading_offset_deg')
        self.mag_calibration_angular_velocity = self.get_float_parameter(
            'mag_calibration_angular_velocity'
        )
        self.mag_calibration_duration_sec = self.get_float_parameter(
            'mag_calibration_duration_sec'
        )
        self.mag_calibration_config_file = self.get_parameter(
            'mag_calibration_config_file'
        ).get_parameter_value().string_value

        # Serial port for Arduino communications
        self.ser = serial.Serial(serial_port, 115200, timeout=serial_timeout, write_timeout=serial_timeout)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.serial_lock = threading.Lock()
        self.mag_calibration_lock = threading.Lock()
        self.mag_calibration_active = False
        self.mag_calibration_samples = None

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
        self.calibration_cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Int16MultiArray, 'pwm_setpoints',  self.pwm_callback, 10)
        self.calibration_service_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            Trigger,
            'calibrate_magnetometer',
            self.calibrate_magnetometer_callback,
            callback_group=self.calibration_service_group,
        )
        
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

       
    def declare_float_parameter(self, name, default_value):
        self.declare_parameter(
            name,
            float(default_value),
            ParameterDescriptor(dynamic_typing=True),
        )

    def get_float_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name} must be numeric, got {value!r}')
        return float(value)

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

    def calibrate_magnetometer_callback(self, request, response):
        del request

        with self.mag_calibration_lock:
            if self.mag_calibration_active:
                response.success = False
                response.message = 'Magnetometer calibration is already running.'
                return response

            self.mag_calibration_active = True
            self.mag_calibration_samples = {
                'count': 0,
                'x_min': float('inf'),
                'x_max': float('-inf'),
                'y_min': float('inf'),
                'y_max': float('-inf'),
                'z_min': float('inf'),
                'z_max': float('-inf'),
            }

        duration_sec = max(1.0, self.mag_calibration_duration_sec)
        angular_velocity = self.mag_calibration_angular_velocity
        self.get_logger().info(
            f'Starting magnetometer calibration: rotating at {angular_velocity:.2f} rad/s '
            f'for {duration_sec:.1f}s.'
        )

        try:
            end_time = time.monotonic() + duration_sec
            while time.monotonic() < end_time and rclpy.ok():
                self.publish_calibration_cmd(angular_velocity)
                time.sleep(0.05)

            self.publish_calibration_cmd(0.0)
            time.sleep(0.1)
            self.publish_calibration_cmd(0.0)

            with self.mag_calibration_lock:
                samples = dict(self.mag_calibration_samples)
                self.mag_calibration_active = False
                self.mag_calibration_samples = None

            calibration = self.calculate_magnetometer_calibration(samples)
            self.apply_magnetometer_calibration(calibration)
            config_path = self.save_magnetometer_calibration(calibration)

            response.success = True
            response.message = (
                f'Magnetometer calibration saved to {config_path}. '
                f'samples={samples["count"]}, '
                f'x=[{samples["x_min"]:.3f}, {samples["x_max"]:.3f}] uT, '
                f'y=[{samples["y_min"]:.3f}, {samples["y_max"]:.3f}] uT, '
                f'z=[{samples["z_min"]:.3f}, {samples["z_max"]:.3f}] uT, '
                f'z_span={samples["z_max"] - samples["z_min"]:.3f} uT.'
            )
            return response
        except Exception as exc:
            self.publish_calibration_cmd(0.0)
            with self.mag_calibration_lock:
                self.mag_calibration_active = False
                self.mag_calibration_samples = None
            response.success = False
            response.message = f'Magnetometer calibration failed: {exc}'
            self.get_logger().error(response.message)
            return response

    def publish_calibration_cmd(self, angular_velocity):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = float(angular_velocity)
        self.calibration_cmd_pub.publish(msg)

    def record_magnetometer_calibration_sample(self, mag_x_uT, mag_y_uT, mag_z_uT):
        with self.mag_calibration_lock:
            if not self.mag_calibration_active or self.mag_calibration_samples is None:
                return

            samples = self.mag_calibration_samples
            samples['count'] += 1
            samples['x_min'] = min(samples['x_min'], mag_x_uT)
            samples['x_max'] = max(samples['x_max'], mag_x_uT)
            samples['y_min'] = min(samples['y_min'], mag_y_uT)
            samples['y_max'] = max(samples['y_max'], mag_y_uT)
            samples['z_min'] = min(samples['z_min'], mag_z_uT)
            samples['z_max'] = max(samples['z_max'], mag_z_uT)

    def calculate_magnetometer_calibration(self, samples):
        if samples['count'] < 10:
            raise ValueError(f'not enough magnetometer samples ({samples["count"]})')

        x_radius = (samples['x_max'] - samples['x_min']) / 2.0
        y_radius = (samples['y_max'] - samples['y_min']) / 2.0
        if x_radius <= 1e-6 or y_radius <= 1e-6:
            raise ValueError(
                f'invalid magnetometer span: x_radius={x_radius:.6f}, y_radius={y_radius:.6f}'
            )

        avg_radius = (x_radius + y_radius) / 2.0
        return {
            'mag_x_offset_uT': (samples['x_max'] + samples['x_min']) / 2.0,
            'mag_y_offset_uT': (samples['y_max'] + samples['y_min']) / 2.0,
            'mag_x_scale': avg_radius / x_radius,
            'mag_y_scale': avg_radius / y_radius,
        }

    def apply_magnetometer_calibration(self, calibration):
        self.mag_x_offset_uT = calibration['mag_x_offset_uT']
        self.mag_y_offset_uT = calibration['mag_y_offset_uT']
        self.mag_x_scale = calibration['mag_x_scale']
        self.mag_y_scale = calibration['mag_y_scale']
        self.set_parameters([
            Parameter('mag_x_offset_uT', Parameter.Type.DOUBLE, self.mag_x_offset_uT),
            Parameter('mag_y_offset_uT', Parameter.Type.DOUBLE, self.mag_y_offset_uT),
            Parameter('mag_x_scale', Parameter.Type.DOUBLE, self.mag_x_scale),
            Parameter('mag_y_scale', Parameter.Type.DOUBLE, self.mag_y_scale),
        ])

    def save_magnetometer_calibration(self, calibration):
        config_path = self.resolve_magnetometer_config_file()
        self.update_yaml_parameters(config_path, 'mini_bot_node', calibration)
        return str(config_path)

    def resolve_magnetometer_config_file(self):
        configured_path = self.mag_calibration_config_file.strip()
        if configured_path:
            config_path = Path(configured_path).expanduser()
            if not config_path.is_absolute():
                config_path = Path.cwd() / config_path
            return config_path

        candidates = [
            Path.cwd() / 'config' / 'mini_bot.yaml',
            Path.cwd() / 'src' / 'mini_bot' / 'config' / 'mini_bot.yaml',
            Path('/home/narcis/ros2_ws/src/mini_bot/config/mini_bot.yaml'),
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate

        raise FileNotFoundError(
            'mag_calibration_config_file is empty and config/mini_bot.yaml could not be found'
        )

    def update_yaml_parameters(self, config_path, node_name, parameter_values):
        if not config_path.exists():
            raise FileNotFoundError(f'{config_path} does not exist')

        lines = config_path.read_text().splitlines()
        node_start = self.find_yaml_key_line(lines, node_name, 0, 0, len(lines))
        if node_start is None:
            raise ValueError(f'could not find {node_name}: in {config_path}')

        node_end = self.find_yaml_block_end(lines, node_start, 0)
        params_start = self.find_yaml_key_line(lines, 'ros__parameters', 2, node_start + 1, node_end)
        if params_start is None:
            raise ValueError(f'could not find {node_name}.ros__parameters in {config_path}')

        params_end = self.find_yaml_block_end(lines, params_start, 2)
        param_indent = ' ' * 4
        insert_at = params_start + 1

        for key, value in parameter_values.items():
            replacement = f'{param_indent}{key}: {self.format_yaml_float(value)}'
            line_idx = self.find_yaml_key_line(lines, key, 4, params_start + 1, params_end)
            if line_idx is None:
                lines.insert(insert_at, replacement)
                insert_at += 1
                params_end += 1
                node_end += 1
            else:
                lines[line_idx] = replacement

        config_path.write_text('\n'.join(lines) + '\n')

    def find_yaml_key_line(self, lines, key, indent, start, end):
        prefix = ' ' * indent + key + ':'
        for idx in range(start, end):
            if lines[idx].startswith(prefix):
                return idx
        return None

    def find_yaml_block_end(self, lines, start, indent):
        for idx in range(start + 1, len(lines)):
            line = lines[idx]
            if line.strip() == '' or line.lstrip().startswith('#'):
                continue
            current_indent = len(line) - len(line.lstrip(' '))
            if current_indent <= indent:
                return idx
        return len(lines)

    def format_yaml_float(self, value):
        formatted = f'{float(value):.6g}'
        if 'e' in formatted or 'E' in formatted:
            mantissa, exponent = formatted.lower().split('e', 1)
            if '.' not in mantissa:
                mantissa += '.0'
            return f'{mantissa}e{exponent}'
        if '.' not in formatted:
            formatted += '.0'
        return formatted

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
        self.record_magnetometer_calibration_sample(
            float(mag_x_raw) / 100.0,
            float(mag_y_raw) / 100.0,
            float(mag_z_raw) / 100.0,
        )

        # Publish MagneticField message
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = 'imu_link' # Or your appropriate frame
        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z
        # We don't have covariance data, so we leave it as zeros
        self.mag_pub.publish(mag_msg)

        heading_deg = self.calculate_compass_heading_deg(mag_x_raw, mag_y_raw)

        angle_msg = Int32()
        angle_msg.data = int(round(heading_deg)) % 360
        self.angle_pub.publish(angle_msg)

    def calculate_compass_heading_deg(self, mag_x_raw, mag_y_raw):
        mag_x_uT = float(mag_x_raw) / 100.0
        mag_y_uT = float(mag_y_raw) / 100.0

        mag_x_cal = (mag_x_uT - self.mag_x_offset_uT) * self.mag_x_scale
        mag_y_cal = (mag_y_uT - self.mag_y_offset_uT) * self.mag_y_scale

        heading_deg = math.degrees(math.atan2(mag_y_cal, mag_x_cal))
        return (heading_deg + self.mag_heading_offset_deg) % 360.0


def main(args=None):
    rclpy.init(args=args)
    node = MiniBotNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    try:
        node.ser.close()
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()
