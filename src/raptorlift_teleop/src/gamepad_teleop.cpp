// Copyright 2026 Kandins
// SPDX-License-Identifier: Apache-2.0

#include "raptorlift_teleop/gamepad_teleop.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace raptorlift_teleop
{

GamepadTeleop::GamepadTeleop(const rclcpp::NodeOptions & options)
: Node("gamepad_teleop", options),
  current_gear_(10),
  emergency_stop_(false),
  teleop_priority_(false),
  target_linear_x_(0.0),
  target_angular_z_(0.0),
  current_lift_cmd_(LiftCommand::STOP),
  first_joy_msg_(true)
{
  // Declare parameters
  this->declare_parameter("min_linear_vel", 0.1);
  this->declare_parameter("max_linear_vel", 2.0);
  this->declare_parameter("max_angular_vel", 1.0);
  this->declare_parameter("deadzone", 0.1);
  this->declare_parameter("num_gears", 20);
  this->declare_parameter("publish_rate", 50.0);
  this->declare_parameter("joy_timeout", 0.5);
  this->declare_parameter("frame_id", "base_link");

  // Get parameters
  min_linear_vel_ = this->get_parameter("min_linear_vel").as_double();
  max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
  max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
  deadzone_ = this->get_parameter("deadzone").as_double();
  num_gears_ = this->get_parameter("num_gears").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  joy_timeout_ = this->get_parameter("joy_timeout").as_double();
  frame_id_ = this->get_parameter("frame_id").as_string();

  // Validate
  if (num_gears_ < 1) {
    num_gears_ = 1;
  }
  current_gear_ = std::max(1, std::min(num_gears_ / 2, num_gears_));

  // Create subscriber
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS(),
    std::bind(&GamepadTeleop::joy_callback, this, std::placeholders::_1));

  // Create publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", rclcpp::QoS(10));

  gear_pub_ = this->create_publisher<std_msgs::msg::Int32>(
    "gear_state", rclcpp::QoS(10).reliable());

  lift_pub_ = this->create_publisher<std_msgs::msg::Int8>(
    "lift_command", rclcpp::QoS(10));

  state_pub_ = this->create_publisher<std_msgs::msg::String>(
    "teleop_state", rclcpp::QoS(10));

  // Create timer
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GamepadTeleop::publish_cmd_vel, this));

  // Initialize timing
  last_joy_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "GamepadTeleop initialized (gear-scaled output for twist_mux)");
  RCLCPP_INFO(this->get_logger(), "  Gears: %d (current: %d)", num_gears_, current_gear_);
  RCLCPP_INFO(this->get_logger(), "  Velocity range: %.2f - %.2f m/s (gear-scaled output)",
    min_linear_vel_, max_linear_vel_);
  RCLCPP_INFO(this->get_logger(), "  Output: gear-scaled cmd_vel (m/s) + gear_state");
}

void GamepadTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (first_joy_msg_) {
    prev_buttons_ = msg->buttons;
    first_joy_msg_ = false;
    RCLCPP_INFO(this->get_logger(), "Gamepad connected (axes: %zu, buttons: %zu)",
      msg->axes.size(), msg->buttons.size());
  }

  last_joy_time_ = this->now();

  process_buttons(msg);
  process_axes(msg);

  prev_buttons_ = msg->buttons;
}

void GamepadTeleop::process_buttons(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons.size() < 12) {
    return;
  }

  auto is_pressed = [&](int button) -> bool {
    if (button >= static_cast<int>(msg->buttons.size()) ||
        button >= static_cast<int>(prev_buttons_.size())) {
      return false;
    }
    return msg->buttons[button] == 1 && prev_buttons_[button] == 0;
  };

  auto is_held = [&](int button) -> bool {
    if (button >= static_cast<int>(msg->buttons.size())) {
      return false;
    }
    return msg->buttons[button] == 1;
  };

  // Emergency stop toggle
  if (is_pressed(XboxButtons::ESTOP)) {
    emergency_stop_ = !emergency_stop_;
    if (emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP ACTIVATED");
      target_linear_x_ = 0.0;
      target_angular_z_ = 0.0;
    } else {
      RCLCPP_INFO(this->get_logger(), "Emergency stop released");
    }
  }

  // Mode toggle
  if (is_pressed(XboxButtons::MODE)) {
    teleop_priority_ = !teleop_priority_;
    RCLCPP_INFO(this->get_logger(), "Mode: %s",
      teleop_priority_ ? "TELEOP PRIORITY" : "SHARED");
  }

  // Gear control
  if (is_pressed(XboxButtons::L1)) {
    current_gear_ = std::max(1, current_gear_ - 1);
    RCLCPP_INFO(this->get_logger(), "Gear DOWN: %d (max vel: %.2f m/s)",
      current_gear_, calculate_gear_velocity());
  }
  if (is_pressed(XboxButtons::R1)) {
    current_gear_ = std::min(num_gears_, current_gear_ + 1);
    RCLCPP_INFO(this->get_logger(), "Gear UP: %d (max vel: %.2f m/s)",
      current_gear_, calculate_gear_velocity());
  }

  // Lift control
  if (is_held(XboxButtons::X)) {
    current_lift_cmd_ = LiftCommand::UP;
  } else if (is_held(XboxButtons::A)) {
    current_lift_cmd_ = LiftCommand::DOWN;
  } else if (is_pressed(XboxButtons::Y)) {
    current_lift_cmd_ = LiftCommand::HOME;
  } else {
    current_lift_cmd_ = LiftCommand::STOP;
  }

  // Publish lift command
  auto lift_msg = std_msgs::msg::Int8();
  lift_msg.data = static_cast<int8_t>(current_lift_cmd_);
  lift_pub_->publish(lift_msg);
}

void GamepadTeleop::process_axes(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->axes.size() < 6) {
    return;
  }

  if (emergency_stop_) {
    target_linear_x_ = 0.0;
    target_angular_z_ = 0.0;
    return;
  }

  // Get normalized axis values [-1, 1]
  double left_stick_y = apply_deadzone(msg->axes[XboxAxes::L_STICK_Y]);
  double right_stick_x = apply_deadzone(msg->axes[XboxAxes::R_STICK_X]);

  // Get trigger values for speed modifiers
  double lt_value = msg->axes[XboxAxes::LT];
  double rt_value = msg->axes[XboxAxes::RT];

  // Normalize triggers (0.0 = released, 1.0 = fully pressed)
  double lt_normalized = (1.0 - lt_value) / 2.0;
  double rt_normalized = (1.0 - rt_value) / 2.0;

  // Speed modifier: LT slows down, RT speeds up
  double speed_modifier = (1.0 - lt_normalized * 0.7) * (1.0 + rt_normalized * 0.5);

  // Output normalized values [-1, 1] (gear scaling applied at publish stage)
  target_linear_x_ = left_stick_y * speed_modifier;
  target_angular_z_ = right_stick_x * speed_modifier;

  // Clamp to [-1, 1]
  target_linear_x_ = std::clamp(target_linear_x_, -1.0, 1.0);
  target_angular_z_ = std::clamp(target_angular_z_, -1.0, 1.0);

  // D-pad for precise movement (diagonal supported).
  // Note: Xbox D-pad hat switch may ghost one axis when transitioning from
  // diagonal to cardinal. Workaround: fully release D-pad before re-pressing.
  if (msg->axes.size() >= 8) {
    double dpad_y = msg->axes[XboxAxes::DPAD_Y];
    double dpad_x = msg->axes[XboxAxes::DPAD_X];

    if (std::abs(dpad_y) > 0.5 || std::abs(dpad_x) > 0.5) {
      target_linear_x_ = dpad_y * 0.2;
      target_angular_z_ = -dpad_x * 0.3;
    }
  }
}

double GamepadTeleop::apply_deadzone(double value) const
{
  if (std::abs(value) < deadzone_) {
    return 0.0;
  }
  double sign = value > 0 ? 1.0 : -1.0;
  return sign * (std::abs(value) - deadzone_) / (1.0 - deadzone_);
}

double GamepadTeleop::calculate_gear_velocity() const
{
  if (num_gears_ <= 1) {
    return max_linear_vel_;
  }
  return min_linear_vel_ +
         (max_linear_vel_ - min_linear_vel_) *
         static_cast<double>(current_gear_ - 1) /
         static_cast<double>(num_gears_ - 1);
}

void GamepadTeleop::publish_cmd_vel()
{
  double time_since_joy = (this->now() - last_joy_time_).seconds();

  double output_linear = 0.0;
  double output_angular = 0.0;

  if (time_since_joy <= joy_timeout_) {
    // Apply gear-based velocity scaling (outputs real m/s, not normalized)
    double gear_max_vel = calculate_gear_velocity();
    output_linear = target_linear_x_ * gear_max_vel;
    // Scale angular velocity proportionally with gear
    double gear_angular_scale = (max_linear_vel_ > 0.0) ? (gear_max_vel / max_linear_vel_) : 1.0;
    output_angular = target_angular_z_ * max_angular_vel_ * gear_angular_scale;
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "No joy messages for %.1f seconds", time_since_joy);
  }

  // Publish gear-scaled cmd_vel (TwistStamped)
  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.stamp = this->now();
  twist_msg.header.frame_id = frame_id_;
  twist_msg.twist.linear.x = output_linear;
  twist_msg.twist.angular.z = output_angular;
  cmd_vel_pub_->publish(twist_msg);

  // Always publish gear state
  auto gear_msg = std_msgs::msg::Int32();
  gear_msg.data = current_gear_;
  gear_pub_->publish(gear_msg);

  publish_state();
}

void GamepadTeleop::publish_state()
{
  if (++state_publish_counter_ < 50) {
    return;
  }
  state_publish_counter_ = 0;

  std::ostringstream oss;
  oss << "gear=" << current_gear_
      << ",estop=" << (emergency_stop_ ? "ON" : "OFF")
      << ",priority=" << (teleop_priority_ ? "TELEOP" : "SHARED")
      << ",lin=" << std::fixed << std::setprecision(2) << target_linear_x_
      << ",ang=" << std::fixed << std::setprecision(2) << target_angular_z_;

  auto state_msg = std_msgs::msg::String();
  state_msg.data = oss.str();
  state_pub_->publish(state_msg);
}

}  // namespace raptorlift_teleop

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<raptorlift_teleop::GamepadTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
