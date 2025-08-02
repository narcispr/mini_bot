from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('mini_bot'),
        'config',
        'mini_bot.yaml'
    )

    return LaunchDescription([
        Node(
            package='mini_bot',
            executable='mini_bot_node',
            name='mini_bot_node',
            parameters=[config_path],
            output='screen'
        ),
        Node(
            package='mini_bot',
            executable='navigation_node',
            name='navigation_node',
            parameters=[config_path],
            output='screen'
        ),
        Node(
            package='mini_bot',
            executable='controller_node',
            name='controller_node',
            parameters=[config_path],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='range_static_tf',
            arguments=['0.05', '0.0', '0.0', '0', '0', '0', 'base_link', 'range_link'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            arguments=['0.05', '0.0', '0.05', '0', '0', '0', 'base_link', 'camera_link'],
            output='screen'
        ),
        # Node(
        #     package='usb_cam',
        #     executable='usb_cam_node_exe',
        #     name='usb_cam',
        #     output='screen',
        #     parameters=[{
        #         'video_device': '/dev/video0',
        #         'image_width': 320,
        #         'image_height': 240,
        #         'pixel_format': 'yuyv',
        #         'camera_frame_id': 'camera_link'
        #     }]
        # ),
    ])
