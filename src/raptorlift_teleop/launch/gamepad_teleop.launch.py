#!/usr/bin/env python3
# Copyright 2026 Kandins
# SPDX-License-Identifier: Apache-2.0

"""Launch file for RaptorLift Xbox gamepad teleoperation."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for gamepad teleop."""

    # Get package share directory
    pkg_share = FindPackageShare('raptorlift_teleop')

    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'gamepad_config.yaml']),
        description='Path to the teleop configuration file'
    )

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for nodes'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }
        ],
        output='screen',
    )

    # Gamepad teleop node
    gamepad_teleop_node = Node(
        package='raptorlift_teleop',
        executable='gamepad_teleop',
        name='gamepad_teleop',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
        remappings=[
            ('joy', 'joy'),
            ('cmd_vel', '/teleop/cmd_vel'),
            ('lift_command', 'lift_command'),
            ('teleop_state', 'teleop_state'),
        ],
    )

    return LaunchDescription([
        config_file_arg,
        joy_dev_arg,
        namespace_arg,
        use_sim_time_arg,
        joy_node,
        gamepad_teleop_node,
    ])
