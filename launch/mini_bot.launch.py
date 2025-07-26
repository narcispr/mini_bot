from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('mini_bot'),
        'config',
        'mini_bot.yaml'
    )

    with open(config_path, 'r') as f:
        config_params = yaml.safe_load(f)

    # Common parameters for all nodes that need them
    mini_bot_params = config_params['mini_bot_params']

    # Ensure all parameters are either str, int, bool, or launch.Substitution
    # Convert any float arguments in static_transform_publisher to strings
    return LaunchDescription([
        Node(
            package='mini_bot',
            executable='mini_bot_node',
            name='mini_bot_node',
            parameters=[mini_bot_params],
            output='screen'
        ),
        # Node(
        #     package='mini_bot',
        #     executable='navigation_node',
        #     name='navigation_node',
        #     parameters=[mini_bot_params, config_params['navigation']],
        #     output='screen'
        # ),
        # Node(
        #     package='mini_bot',
        #     executable='controller_node',
        #     name='controller_node',
        #     parameters=[mini_bot_params, config_params['controller']],
        #     output='screen'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='range_static_tf',
        #     arguments=['0.05', '0.0', '0.0', '0', '0', '0', 'base_link', 'range_link'],
        #     output='screen'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_static_tf',
        #     arguments=['0.05', '0.0', '0.05', '0', '0', '0', 'base_link', 'camera_link'],
        #     output='screen'
        # ),
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
