// Copyright 2026 Kandins
// SPDX-License-Identifier: Apache-2.0

#include "raptorlift_teleop/velocity_scaler.hpp"

#include <algorithm>
#include <cmath>

namespace raptorlift_teleop
{

VelocityScaler::VelocityScaler(const rclcpp::NodeOptions & options)
: Node("velocity_scaler", options),
  current_gear_(10),
  input_linear_x_(0.0),
  input_angular_z_(0.0)
{
  // Declare parameters
  this->declare_parameter("min_linear_vel", 0.1);
  this->declare_parameter("max_linear_vel", 2.0);
  this->declare_parameter("max_angular_vel", 1.0);
  this->declare_parameter("num_gears", 20);
  this->declare_parameter("default_gear", 10);
  this->declare_parameter("input_max_linear", 1.0);
  this->declare_parameter("input_max_angular", 1.0);
  this->declare_parameter("publish_rate", 50.0);
  this->declare_parameter("cmd_timeout", 0.5);
  this->declare_parameter("frame_id", "base_link");

  // Get parameters
  min_linear_vel_ = this->get_parameter("min_linear_vel").as_double();
  max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
  max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
  num_gears_ = this->get_parameter("num_gears").as_int();
  default_gear_ = this->get_parameter("default_gear").as_int();
  input_max_linear_ = this->get_parameter("input_max_linear").as_double();
  input_max_angular_ = this->get_parameter("input_max_angular").as_double();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
  frame_id_ = this->get_parameter("frame_id").as_string();

  // Initialize state
  current_gear_ = default_gear_;

  // Create subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_in", rclcpp::QoS(10),
    std::bind(&VelocityScaler::cmd_vel_callback, this, std::placeholders::_1));

  gear_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "gear_state", rclcpp::QoS(10),
    std::bind(&VelocityScaler::gear_callback, this, std::placeholders::_1));

  // Create publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel_out", rclcpp::QoS(10));

  // Create timer
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&VelocityScaler::publish_timer_callback, this));

  // Initialize timing
  last_cmd_time_ = this->now();
  last_gear_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "VelocityScaler initialized");
  RCLCPP_INFO(this->get_logger(), "  Gear range: 1-%d (current: %d)", num_gears_, current_gear_);
  RCLCPP_INFO(this->get_logger(), "  Velocity range: %.2f - %.2f m/s", min_linear_vel_, max_linear_vel_);
  RCLCPP_INFO(this->get_logger(), "  Input normalization: linear=%.1f, angular=%.1f",
    input_max_linear_, input_max_angular_);
}

void VelocityScaler::cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  last_cmd_time_ = this->now();

  // Normalize input to [-1, 1] range (guard against zero divisor)
  double lin_scale = (input_max_linear_ > 0.0) ? input_max_linear_ : 1.0;
  double ang_scale = (input_max_angular_ > 0.0) ? input_max_angular_ : 1.0;
  input_linear_x_ = std::clamp(msg->twist.linear.x / lin_scale, -1.0, 1.0);
  input_angular_z_ = std::clamp(msg->twist.angular.z / ang_scale, -1.0, 1.0);
}

void VelocityScaler::gear_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  last_gear_time_ = this->now();

  int new_gear = std::clamp(msg->data, 1, num_gears_);
  if (new_gear != current_gear_) {
    current_gear_ = new_gear;
    RCLCPP_INFO(this->get_logger(), "Gear changed: %d (max vel: %.2f m/s)",
      current_gear_, calculate_gear_velocity());
  }
}

double VelocityScaler::calculate_gear_velocity() const
{
  if (num_gears_ <= 1) {
    return max_linear_vel_;
  }
  // Linear interpolation: gear 1 = min_vel, gear N = max_vel
  return min_linear_vel_ +
         (max_linear_vel_ - min_linear_vel_) *
         static_cast<double>(current_gear_ - 1) /
         static_cast<double>(num_gears_ - 1);
}

void VelocityScaler::publish_timer_callback()
{
  double time_since_cmd = (this->now() - last_cmd_time_).seconds();

  double output_linear_x = 0.0;
  double output_angular_z = 0.0;

  if (time_since_cmd <= cmd_timeout_) {
    // Apply gear scaling
    double gear_max_vel = calculate_gear_velocity();
    output_linear_x = input_linear_x_ * gear_max_vel;
    // Scale angular velocity proportionally with gear velocity
    // At low gears, steering angle limits constrain achievable omega
    // omega_max = v * tan(max_steer) / wheelbase, but we use simple linear scaling
    double gear_angular_scale = (max_linear_vel_ > 0.0) ? (gear_max_vel / max_linear_vel_) : 1.0;
    double scaled_max_angular = max_angular_vel_ * gear_angular_scale;
    output_angular_z = input_angular_z_ * scaled_max_angular;
  } else {
    // Timeout - stop
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "No cmd_vel for %.1f seconds, stopping", time_since_cmd);
  }

  // Publish TwistStamped
  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.stamp = this->now();
  twist_msg.header.frame_id = frame_id_;
  twist_msg.twist.linear.x = output_linear_x;
  twist_msg.twist.angular.z = output_angular_z;
  cmd_vel_pub_->publish(twist_msg);
}

}  // namespace raptorlift_teleop

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<raptorlift_teleop::VelocityScaler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
