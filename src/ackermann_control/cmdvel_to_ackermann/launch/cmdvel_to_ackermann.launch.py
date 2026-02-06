# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_period_ms', default_value='20',
            description='Publishing period in milliseconds'),
        DeclareLaunchArgument(
            'wheelbase', default_value='1.0',
            description='Distance between front and rear axles (m)'),
        DeclareLaunchArgument(
            'acceleration', default_value='0.0',
            description='Acceleration limit, 0 = unlimited (m/s^2)'),
        DeclareLaunchArgument(
            'steering_velocity', default_value='0.0',
            description='Steering angle rate limit, 0 = unlimited (rad/s)'),
        DeclareLaunchArgument(
            'cmd_timeout', default_value='0.5',
            description='Command timeout before sending zero (s)'),

        Node(
            package='cmdvel_to_ackermann',
            executable='cmdvel_to_ackermann.py',
            name='cmdvel_to_ackermann',
            output='screen',
            parameters=[{
                'publish_period_ms': LaunchConfiguration('publish_period_ms'),
                'wheelbase': LaunchConfiguration('wheelbase'),
                'acceleration': LaunchConfiguration('acceleration'),
                'steering_velocity': LaunchConfiguration('steering_velocity'),
                'cmd_timeout': LaunchConfiguration('cmd_timeout'),
            }]
        ),
    ])
