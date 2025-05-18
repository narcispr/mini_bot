from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node principal del robot
        Node(
            package='mini_bot',
            executable='mini_bot_node',
            name='mini_bot',
            output='screen'
        ),

        # Transformació estàtica: base_link → range_link (5 cm davant)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='range_static_tf',
            arguments=['0.05', '0.0', '0.0',  # x, y, z
                       '0', '0', '0',         # roll, pitch, yaw
                       'base_link', 'range_link'],
            output='screen'
        ),

        # Transformació estàtica: base_link → range_link (5 cm davant)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            arguments=['0.05', '0.0', '0.05',  # x, y, z
                       '0', '0', '0',         # roll, pitch, yaw
                       'base_link', 'camera_link'],
            output='screen'
        ),

        # Node de càmera USB
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 320,
                'image_height': 240,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'camera_link'
            }]
        ),
    ])
