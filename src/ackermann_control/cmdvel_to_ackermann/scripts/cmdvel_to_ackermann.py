#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class CmdvelToAckermann(Node):
    """Convert TwistStamped to AckermannDriveStamped for rear-steer Ackermann robot.

    RaptorLift is a rear-steer forklift: the rear wheels steer while the front
    wheels provide traction. The steering angle sign is negated compared to
    standard front-steer Ackermann because the steering axle is behind the
    traction axle.
    """

    def __init__(self):
        super().__init__('cmdvel_to_ackermann')

        self.declare_parameter('publish_period_ms', 20)
        self.declare_parameter('wheelbase', 1.0)
        self.declare_parameter('acceleration', 0.0)
        self.declare_parameter('steering_velocity', 0.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self._cmd_vel_sub = self.create_subscription(
            TwistStamped, 'cmd_vel', self._cmd_vel_callback, 1)
        self._ackermann_pub = self.create_publisher(
            AckermannDriveStamped, 'ackermann_cmd', 1)

        publish_period_s = self.get_parameter('publish_period_ms').value / 1000.0
        self.create_timer(publish_period_s, self._timer_callback)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.acceleration = self.get_parameter('acceleration').value
        self.steering_velocity = self.get_parameter('steering_velocity').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.get_logger().info(
            f"wheelbase: {self.wheelbase}, "
            f"acceleration: {self.acceleration}, "
            f"cmd_timeout: {self.cmd_timeout}")

        self._ackermann_msg = None
        self._last_cmd_time = self.get_clock().now()

    def _convert_to_rear_steering_angle(self, v, omega):
        """Compute rear-steer Ackermann steering angle from v and omega.

        For rear-steer: steering_angle = -atan(wheelbase * omega / v)
        The negative sign accounts for the rear steering geometry.
        """
        if omega == 0.0 or v == 0.0:
            return 0.0

        return -math.atan(self.wheelbase * omega / v)

    def _cmd_vel_callback(self, msg):
        self._last_cmd_time = self.get_clock().now()

        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.drive.speed = msg.twist.linear.x
        ack_msg.drive.steering_angle = self._convert_to_rear_steering_angle(
            msg.twist.linear.x, msg.twist.angular.z)
        ack_msg.drive.acceleration = self.acceleration
        ack_msg.drive.steering_angle_velocity = self.steering_velocity

        self._ackermann_msg = ack_msg

    def _timer_callback(self):
        if self._ackermann_msg is None:
            return

        # Stop publishing if no cmd_vel received within timeout
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self._ackermann_msg = None
            zero_msg = AckermannDriveStamped()
            zero_msg.header.stamp = self.get_clock().now().to_msg()
            self._ackermann_pub.publish(zero_msg)
            return

        self._ackermann_pub.publish(self._ackermann_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdvelToAckermann()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
