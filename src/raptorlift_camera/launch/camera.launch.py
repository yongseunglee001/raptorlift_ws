"""Launch Basler pylon camera for RaptorLift forklift."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_raptorlift_camera = get_package_share_directory('raptorlift_camera')
    pkg_pylon_wrapper = get_package_share_directory('pylon_ros2_camera_wrapper')

    config_file = os.path.join(pkg_raptorlift_camera, 'config', 'camera.yaml')

    # -- Launch arguments --
    declare_camera_id = DeclareLaunchArgument(
        'camera_id',
        default_value='front_camera',
        description='Namespace / id of the camera',
    )
    declare_respawn = DeclareLaunchArgument(
        'respawn',
        default_value='true',
        description='Respawn camera node on crash',
    )

    # -- Pylon driver (upstream launch) --
    pylon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pylon_wrapper, 'launch', 'pylon_ros2_camera.launch.py')
        ),
        launch_arguments={
            'node_name': 'pylon_camera_node',
            'camera_id': LaunchConfiguration('camera_id'),
            'config_file': config_file,
            'mtu_size': '8192',
            'startup_user_set': 'CurrentSetting',
            'enable_status_publisher': 'true',
            'enable_current_params_publisher': 'true',
            'respawn': LaunchConfiguration('respawn'),
        }.items(),
    )

    # -- Static TF: base_link -> camera_link --
    # Adjust xyz / rpy to match actual camera mount on the forklift.
    # Default: front-center, 1.2m high, tilted 15 deg down.
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', '0.60',
            '--y', '0.0',
            '--z', '1.20',
            '--roll', '0.0',
            '--pitch', '0.26',   # ~15 deg downward
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'raptorlift_camera_link',
        ],
        output='screen',
    )

    # -- Image viewer (auto-opens window) --
    image_viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_viewer',
        arguments=['/front_camera/image_raw'],
        output='screen',
    )

    return LaunchDescription([
        declare_camera_id,
        declare_respawn,
        pylon_launch,
        camera_static_tf,
        image_viewer,
    ])
