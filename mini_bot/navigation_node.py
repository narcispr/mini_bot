import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np


from mini_bot.utils.utils import euler_to_quaternion


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Declare parameters
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.1)
        self.declare_parameter('use_external_theta', True)
        self.declare_parameter(
            'external_theta_calibration_lut',
            [0.0, 0.0, 90.0, 90.0, 180.0, 180.0, 270.0, 270.0],
        )

        # Get parameters
        self.radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.l = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.use_external_theta = self.get_parameter('use_external_theta').get_parameter_value().bool_value
        self.external_theta_calibration_lut = self.load_external_theta_calibration_lut()

        # Initialize the differential drive robot state
        self.pose = Pose2D()
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()
        self.last_theta_time = self.get_clock().now()

        # Init publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
       
        self.subscription_js = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        if self.use_external_theta:
            self.subscription_angle = self.create_subscription(
                Int32,
                'compass_angle',
                self.orientation_callback,
                10
            )
        
        
    def joint_state_callback(self, msg):
        # Assuming wheel order: [left, right]
        vl = (msg.velocity[0] * (self.radius)) 
        vr = (msg.velocity[1] * (self.radius))
        
        # Wheel velocity to robot linear and angular velocity 
        self.v = (vl + vr)/2
        self.w = (vr - vl)/(self.l)
        
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # check last_theta_time
        time_since_theta = (self.get_clock().now() - self.last_theta_time).nanoseconds / 1e9
        if time_since_theta > 0.2 and self.use_external_theta:
            self.get_logger().error('Compass heading message is older than 0.2s!')
       
        # Integrate position
        self.pose.x += self.v * dt * float(np.cos(self.pose.theta))
        self.pose.y += self.v * dt * float(np.sin(self.pose.theta))
        if not self.use_external_theta:
            self.pose.theta += self.w * dt
            if self.pose.theta > 2 * np.pi:
                self.pose.theta -= 2 * np.pi

        self.publish_odometry()

    def load_external_theta_calibration_lut(self):
        lut_param = self.get_parameter('external_theta_calibration_lut').value
        lut = np.array(lut_param, dtype=float)
        if lut.size == 0:
            return np.array([[0.0, 0.0], [360.0, 360.0]], dtype=float)
        if lut.size % 2 != 0:
            raise ValueError('external_theta_calibration_lut must contain measured/corrected degree pairs')

        lut = lut.reshape(-1, 2)
        lut[:, 0] = np.mod(lut[:, 0], 360.0)
        lut[:, 1] = np.mod(lut[:, 1], 360.0)
        lut = lut[np.argsort(lut[:, 0])]

        if np.any(np.diff(lut[:, 0]) <= 0.0):
            raise ValueError('external_theta_calibration_lut measured degree values must be unique')

        return lut

    def calibrate_external_theta(self, measured_deg):
        if self.external_theta_calibration_lut.shape[0] < 2:
            return measured_deg % 360.0

        lut = self.external_theta_calibration_lut
        measured = measured_deg % 360.0
        measured_points = lut[:, 0]
        corrected_points = np.rad2deg(np.unwrap(np.deg2rad(lut[:, 1])))

        if measured < measured_points[0]:
            measured += 360.0

        measured_points = np.append(measured_points, measured_points[0] + 360.0)
        corrected_points = np.append(corrected_points, corrected_points[0] + 360.0)

        return float(np.interp(measured, measured_points, corrected_points) % 360.0)

    def orientation_callback(self, msg):
        corrected_theta_deg = self.calibrate_external_theta(float(msg.data))
        self.pose.theta = math.radians(corrected_theta_deg)
        self.last_theta_time = self.get_clock().now()

    def publish_odometry(self):
        now_msg = self.get_clock().now().to_msg()
        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now_msg
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        quat = euler_to_quaternion(0, 0, self.pose.theta)

        # Set pose
        odom_msg.pose.pose.position.x = self.pose.x
        odom_msg.pose.pose.position.y = self.pose.y
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        # self.get_logger().info(f'Publishing odometry: x={self.pose.x}, y={self.pose.y}, theta={self.pose.theta}')
        
        # Set twist
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.w

        # Publish Odometry message
        self.odom_pub.publish(odom_msg)

        # Broadcast the transform
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pose.x
        t.transform.translation.y = self.pose.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
