// Copyright 2026 Kandins
// SPDX-License-Identifier: Apache-2.0

#ifndef RAPTORLIFT_TELEOP__VELOCITY_SCALER_HPP_
#define RAPTORLIFT_TELEOP__VELOCITY_SCALER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

namespace raptorlift_teleop
{

/**
 * @brief VelocityScaler node - applies gear-based velocity scaling
 *
 * This node receives cmd_vel from twist_mux and gear state from gamepad_teleop,
 * then outputs scaled velocity commands to the ackermann_steering_controller.
 *
 * Subscribes:
 *   - cmd_vel_in (TwistStamped): Input velocity from twist_mux
 *   - gear_state (Int32): Current gear from gamepad_teleop
 *
 * Publishes:
 *   - cmd_vel_out (TwistStamped): Scaled velocity to ackermann_controller
 */
class VelocityScaler : public rclcpp::Node
{
public:
  explicit VelocityScaler(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VelocityScaler() override = default;

private:
  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void gear_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void publish_timer_callback();
  double calculate_gear_velocity() const;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gear_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Parameters
  double min_linear_vel_;       // Minimum linear velocity at gear 1 (m/s)
  double max_linear_vel_;       // Maximum linear velocity at max gear (m/s)
  double max_angular_vel_;      // Maximum angular velocity (rad/s)
  int num_gears_;               // Total number of gears
  int default_gear_;            // Default gear if no gear_state received
  double input_max_linear_;     // Expected max linear input for normalization
  double input_max_angular_;    // Expected max angular input for normalization
  double publish_rate_;         // Publishing rate (Hz)
  double cmd_timeout_;          // Command timeout (seconds)
  std::string frame_id_;        // Frame ID for TwistStamped

  // State
  int current_gear_;
  double input_linear_x_;
  double input_angular_z_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_gear_time_;
};

}  // namespace raptorlift_teleop

#endif  // RAPTORLIFT_TELEOP__VELOCITY_SCALER_HPP_
