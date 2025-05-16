import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Header

import serial
import threading
import time
import numpy as np

from mini_bot.utils.differential_drive import DifferentialWheel
import mini_bot.utils.bot_comms as coms


class MiniBotNode(Node):
    def __init__(self):
        super().__init__('mono_bot_node')

        self.robot = DifferentialWheel(dt=0.1, length=0.124, radius=0.0337, max_v=0.3, max_w=1.57)
        self.serial_lock = threading.Lock()
        self.ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)

        # ROS interfaces
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.range_pub = self.create_publisher(Range, '/range', 10)

        # Timers
        self.create_timer(0.1, self.publish_odometry)

        # Threads
        threading.Thread(target=self.read_sensor_loop, daemon=True).start()

    def cmd_vel_callback(self, msg: Twist):
        print("cmd_vel_callback")
        v = msg.linear.x
        w = msg.angular.z
        print(f"v: {v}, w: {w}")

        # Calcula velocitats individuals
        wl, wr = self.robot.get_inv_velocity((v, w))
        print(f"wl: {wl}, wr: {wr}")
        # Escala a PWM segons el teu mètode
        left_pwm = wl * self.robot.radius * 155 / self.robot.max_v
        right_pwm = wr * self.robot.radius * 155 / self.robot.max_v
        left_pwm = int(left_pwm + np.sign(left_pwm)*100)
        right_pwm = int(right_pwm + np.sign(right_pwm)*100)
        np.clip(left_pwm, -255, 255)
        np.clip(right_pwm, -255, 255)
        print(f"left_pwm: {left_pwm}, right_pwm: {right_pwm}")
        
        # Envia per sèrie
        msg_bytes = coms.build_pwm_message(left_pwm, right_pwm)
        with self.serial_lock:
            self.ser.write(msg_bytes)

        # Mou el robot internament (per odometria)
        self.robot.move((v, w))


    def read_sensor_loop(self):
        while rclpy.ok():
            with self.serial_lock:
                data = coms.read_message(self.ser)

            if data:
                msg_id, payload = data
                if msg_id == coms.ID_SENSOR_RANGE and len(payload) == 2:
                    value = payload[0] | (payload[1] << 8)
                    self.publish_range(value)

            time.sleep(0.05)

    def publish_range(self, dist_cm):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "range_link"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.2
        msg.min_range = 0.02
        msg.max_range = 3.0
        msg.range = dist_cm / 100.0
        self.range_pub.publish(msg)

    def publish_odometry(self):
        x, y, theta = self.robot.get_pose()
        v, w = self.robot.get_velocity()
        # print(f"pose: ({x}, {y}, {theta}), vel: ({v}, {w})")

        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = np.sin(theta / 2.0)
        msg.pose.pose.orientation.w = np.cos(theta / 2.0)

        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.angular.z = float(w)

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MiniBotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
