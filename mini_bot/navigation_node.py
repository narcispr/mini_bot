import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import tf_transformations


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Declare parameters
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.1)
        self.declare_parameter('use_external_theta', True)

        # Get parameters
        self.radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.l = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.use_external_theta = self.get_parameter('use_external_theta').get_parameter_value().bool_value

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
        self.w = (vl - vr)/(self.l)
        
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # check last_theta_time
        time_since_theta = (self.get_clock().now() - self.last_theta_time).nanoseconds / 1e9
        if time_since_theta > 0.2:
            self.get_logger().error('Compass heading message is older than 0.2s!')
       
        # Integrate position
        self.pose.x += self.v * dt * float(np.cos(self.pose.theta))
        self.pose.y += self.v * dt * float(np.sin(self.pose.theta))
        if not self.use_externa_theta:
            self.pose.theta += self.w * dt
            if self.pose.theta > 2 * np.pi:
                self.pose.theta -= 2 * np.pi

        self.publish_odometry()

    def orientation_callback(self, msg):
        # Force heading to measured orientation (degrees)
        self.pose.theta = rclpy.math.radians(msg.data)
        self.last_theta_time = self.get_clock().now()

    def publish_odometry(self):
        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set pose
        odom_msg.pose.pose.position.x = self.pose.x
        odom_msg.pose.pose.position.y = self.pose.y
        odom_msg.pose.pose.orientation = tf_transformations.quaternion_from_euler(0, 0, self.pose.theta)

        # Set twist
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.w

        # Publish Odometry message
        self.odom_pub.publish(odom_msg)

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pose.x
        t.transform.translation.y = self.pose.y
        t.transform.translation.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0, 0, self.pose.theta)
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