import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from rclpy.duration import Duration
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import math
import serial
import threading
import time
import numpy as np

from mini_bot.utils.differential_drive import DifferentialWheel
import mini_bot.utils.bot_comms as coms


class MiniBotNode(Node):
    def __init__(self):
        super().__init__('mono_bot_node')

        # Paràmetres físics del robot
        self.max_v = 0.25      # m/s
        self.max_w = 4.5       # rad/s
        self.min_v = 0.05      # m/s
        self.min_w = 1.5       # rad/s
        self.max_a_v = 0.2    # m/s²
        self.max_a_w = 3.0     # rad/s²
        self.dt = 0.1          # s

        # Paràmetres de conversió wl/wr -> PWM
        self.pwm_lut = {
            'left':  {'min_pwm': 120, 'min_w': 1.0, 'max_pwm': 255, 'max_w': 6.0},
            'right': {'min_pwm': 100, 'min_w': 1.0, 'max_pwm': 255, 'max_w': 6.0},
        }

        # Wheel PWM command
        self.left_pwm = 0
        self.right_pwm = 0

        # Last desired speed command
        self.cmd_vel = (0.0, 0.0)
        self.last_cmd_vel = (0.0, 0.0)
        self.cmd_vel_stamp = self.get_clock().now()
        
        # Differential drive class
        self.robot = DifferentialWheel(dt=0.1, length=0.124, radius=0.0337, max_v=self.max_v, max_w=self.max_w)
        self.robot_lock = threading.Lock()

        # Serial port for Arduino communications
        self.serial_lock = threading.Lock()
        self.ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)

        # ROS interfaces
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.range_pub = self.create_publisher(Range, '/range', 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/range_pointcloud', 10)

        # Timers
        self.create_timer(self.dt, self.loop)

        # Threads
        threading.Thread(target=self.read_sensor_loop, daemon=True).start()

    def __compute_pwm__(self, w, side):
            lut = self.pwm_lut[side]
            min_pwm, min_w = lut['min_pwm'], lut['min_w']
            max_pwm, max_w = lut['max_pwm'], lut['max_w']
            w_abs = abs(w)

            if w_abs < min_w:
                if w_abs != 0.0:
                    print("Warning! Allowing too small speeds!")
                return 0

            pwm = min_pwm + (w_abs - min_w) * (max_pwm - min_pwm) / (max_w - min_w)
            pwm = np.clip(pwm, min_pwm, max_pwm)
            return int(np.sign(w) * pwm)

    def cmd_vel_callback(self, msg: Twist):
        # Entrada original
        v_d = msg.linear.x
        w_d = -1.0*msg.angular.z
        # print(f"-------->  [INPUT] v_d: {v_d}, w_d: {w_d}")
        self.cmd_vel = (v_d, w_d)
        self.cmd_vel_stamp = self.get_clock().now()
        self.__compute_feasible_speed__(v_d, w_d)
       
    def __compute_feasible_speed__(self, v_d, w_d):   
        # print(f"[Desired] v_d: {v_d}, w_d: {w_d}")
       
        # -------------------------------
        # 1. Prioritat a la rotació
        # -------------------------------
        w_ratio = min(abs(w_d) / self.max_w, 1.0)
        v_d = np.clip(v_d, -self.max_v * (1.0 - w_ratio), self.max_v * (1.0 - w_ratio))
        w_d = np.clip(w_d, -self.max_w, self.max_w)

       
        # -------------------------------
        # 2. Limitació dynàmica
        # -------------------------------
        v, w = self.last_cmd_vel
        a_v = (v_d - v) / self.dt
        a_w = (w_d - w) / self.dt
        a_v = np.clip(a_v, -self.max_a_v, self.max_a_v)
        a_w = np.clip(a_w, -self.max_a_w, self.max_a_w)
        v_d = v + a_v * self.dt
        w_d = w + a_w * self.dt
        
        self.last_cmd_vel = (v_d, w_d)

        
        # -------------------------------
        # 2. Aplica umbrals mínims (evita moviments petits)
        # -------------------------------
        if abs(v_d) < self.min_v:
            v_d = 0.0
        if abs(w_d) < self.min_w:
            w_d = 0.0
        
        with self.robot_lock:
            self.robot.move((v_d, w_d))

        # print(f"[Feasible desired vel] v_d: {v_d}, w_d: {w_d}")

        # -------------------------------
        # 4. Obté wl i wr
        # -------------------------------
        wl, wr = self.robot.get_inv_velocity((v_d, w_d))
        # print(f"[Wheel speeds] wl: {wl:.2f}, wr: {wr:.2f}")

        # -------------------------------
        # 5. Transforma a PWM per cada roda
        # -------------------------------
        self.left_pwm = self.__compute_pwm__(wl, 'left')
        self.right_pwm = self.__compute_pwm__(wr, 'right')
        # print(f"[PWM] left: {self.left_pwm}, right: {self.right_pwm}")


    def send_motors_message(self, left_pwm, right_pwm):
        # print(f"left_pwm: {left_pwm}, right_pwm: {right_pwm}")
        
        # Envia per sèriellll
        msg_bytes = coms.build_pwm_message(left_pwm, right_pwm)
        with self.serial_lock:
            self.ser.write(msg_bytes)

    def read_sensor_loop(self):
        while rclpy.ok():
            with self.serial_lock:
                data = coms.read_message(self.ser)

            if data:
                msg_id, payload = data
                if msg_id == coms.ID_SENSOR_RANGE and len(payload) == 2:
                    value = payload[0] | (payload[1] << 8)
                    self.publish_range(value)

            time.sleep(0.025)

    def loop(self):
        # Send message to motor
        self.send_motors_message(self.left_pwm, self.right_pwm)
        
        # Check if last motor command is older than 1s
        if self.cmd_vel_stamp < (self.get_clock().now() - Duration(seconds=1.0)):
            self.cmd_vel = (0.0, 0.0) 

        self.__compute_feasible_speed__(self.cmd_vel[0], self.cmd_vel[1])
          
        # Publish odometry
        self.publish_odometry()

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

    def publish_odometry(self):
        with self.robot_lock:
            x, y, theta = self.robot.get_pose()
            v, w = self.robot.get_velocity()

        # ---------------- Odometry Message ----------------
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(np.sin(theta / 2.0))
        msg.pose.pose.orientation.w = float(np.cos(theta / 2.0))

        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.angular.z = float(w)

        self.odom_pub.publish(msg)

        # ---------------- TF: odom -> base_link ----------------
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'

        tf_msg.transform.translation.x = float(x)
        tf_msg.transform.translation.y = float(y)
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = float(np.sin(theta / 2.0))
        tf_msg.transform.rotation.w = float(np.cos(theta / 2.0))

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MiniBotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
