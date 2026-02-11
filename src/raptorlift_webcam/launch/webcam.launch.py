"""Launch USB webcam with image viewer for RaptorLift forklift."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('raptorlift_webcam')
    config_file = os.path.join(pkg_dir, 'config', 'webcam.yaml')

    # -- Launch arguments --
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='webcam',
        description='Camera namespace',
    )
    declare_video_device = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path (e.g. /dev/video0, /dev/video2)',
    )
    declare_show_image = DeclareLaunchArgument(
        'show_image',
        default_value='true',
        description='Launch rqt_image_view automatically',
    )

    # -- USB camera driver --
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            config_file,
            {'video_device': LaunchConfiguration('video_device')},
        ],
        output='screen',
    )

    # -- Static TF: base_link -> webcam_link --
    # Adjust to actual mount position on forklift.
    webcam_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='webcam_static_tf',
        arguments=[
            '--x', '0.50',
            '--y', '0.0',
            '--z', '1.10',
            '--roll', '0.0',
            '--pitch', '0.20',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'webcam_link',
        ],
        output='screen',
    )

    # -- Image viewer --
    image_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='webcam_viewer',
        arguments=['/webcam/image_raw'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_image')),
    )

    return LaunchDescription([
        declare_namespace,
        declare_video_device,
        declare_show_image,
        usb_cam_node,
        webcam_static_tf,
        image_viewer,
    ])
