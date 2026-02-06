# Copyright 2026 Kandins
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('raptorlift_hardware_bridge')
    default_config = os.path.join(pkg_dir, 'config', 'hardware_bridge.yaml')

    return LaunchDescription([
        # ── Launch Arguments ──────────────────────────────────────────────
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to hardware bridge YAML config file',
        ),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='true',
            description='Run in simulation mode (true) or real hardware (false)',
        ),
        DeclareLaunchArgument(
            'control_rate',
            default_value='100.0',
            description='Control loop frequency in Hz',
        ),
        DeclareLaunchArgument(
            'modbus_ip',
            default_value='192.168.2.1',
            description='PLC Modbus TCP IP address',
        ),
        DeclareLaunchArgument(
            'modbus_port',
            default_value='502',
            description='PLC Modbus TCP port',
        ),
        DeclareLaunchArgument(
            'detailed_logging',
            default_value='false',
            description='Enable verbose logging',
        ),

        # ── Node ──────────────────────────────────────────────────────────
        Node(
            package='raptorlift_hardware_bridge',
            executable='hardware_bridge',
            name='hardware_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'simulation_mode': LaunchConfiguration('simulation_mode'),
                    'control_rate': LaunchConfiguration('control_rate'),
                    'modbus_ip': LaunchConfiguration('modbus_ip'),
                    'modbus_port': LaunchConfiguration('modbus_port'),
                    'detailed_logging': LaunchConfiguration('detailed_logging'),
                },
            ],
        ),
    ])
