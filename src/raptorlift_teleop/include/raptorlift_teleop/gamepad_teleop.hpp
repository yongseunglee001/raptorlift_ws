// Copyright 2026 Kandins
// SPDX-License-Identifier: Apache-2.0

#ifndef RAPTORLIFT_TELEOP__GAMEPAD_TELEOP_HPP_
#define RAPTORLIFT_TELEOP__GAMEPAD_TELEOP_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace raptorlift_teleop
{

/**
 * @brief Xbox controller button indices (Xbox Series X Controller)
 */
struct XboxButtons
{
  static constexpr int A = 0;           // Lift down
  static constexpr int B = 1;           // Lift stop
  static constexpr int X = 2;           // Lift up
  static constexpr int Y = 3;           // Lift home
  static constexpr int LB = 4;          // (unused)
  static constexpr int RB = 5;          // (unused)
  static constexpr int L1 = 6;          // Gear down
  static constexpr int R1 = 7;          // Gear up
  static constexpr int XBOX = 8;        // Xbox button (unused)
  static constexpr int L_STICK = 9;     // Left stick press (unused)
  static constexpr int MODE = 10;       // Teleop/Autonomous mode toggle
  static constexpr int ESTOP = 11;      // Emergency stop toggle
};

/**
 * @brief Xbox controller axes indices
 */
struct XboxAxes
{
  static constexpr int L_STICK_X = 0;   // Left stick horizontal
  static constexpr int L_STICK_Y = 1;   // Left stick vertical (forward/back)
  static constexpr int LT = 2;          // Left trigger (slow down)
  static constexpr int R_STICK_X = 3;   // Right stick horizontal (rotation)
  static constexpr int R_STICK_Y = 4;   // Right stick vertical
  static constexpr int RT = 5;          // Right trigger (turbo)
  static constexpr int DPAD_X = 6;      // D-pad horizontal
  static constexpr int DPAD_Y = 7;      // D-pad vertical
};

/**
 * @brief Lift command values
 */
enum class LiftCommand : int8_t
{
  DOWN = -1,
  STOP = 0,
  UP = 1,
  HOME = 2
};

/**
 * @brief GamepadTeleop node - Gamepad to cmd_vel + gear state
 *
 * Publishes:
 *   - cmd_vel (TwistStamped): Normalized velocity [-1, 1] for twist_mux
 *   - gear_state (Int32): Current gear for velocity_scaler
 *   - lift_command (Int8): Lift control
 *   - teleop_state (String): Status info
 */
class GamepadTeleop : public rclcpp::Node
{
public:
  explicit GamepadTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GamepadTeleop() override = default;

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void publish_cmd_vel();
  void process_buttons(const sensor_msgs::msg::Joy::SharedPtr msg);
  void process_axes(const sensor_msgs::msg::Joy::SharedPtr msg);
  double apply_deadzone(double value) const;
  void publish_state();

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gear_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr lift_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Parameters
  double max_angular_vel_;     // Maximum angular velocity (rad/s)
  double deadzone_;            // Joystick deadzone
  int num_gears_;              // Number of gear steps
  double publish_rate_;        // Publishing rate (Hz)
  double joy_timeout_;         // Joy message timeout (seconds)
  std::string frame_id_;       // Frame ID for TwistStamped header

  // State
  int current_gear_;
  bool emergency_stop_;
  bool teleop_priority_;
  double target_linear_x_;     // Normalized [-1, 1]
  double target_angular_z_;    // Normalized [-1, 1]
  LiftCommand current_lift_cmd_;

  // Button state tracking
  std::vector<int> prev_buttons_;
  bool first_joy_msg_;

  // Timing
  rclcpp::Time last_joy_time_;
  int state_publish_counter_{0};
};

}  // namespace raptorlift_teleop

#endif  // RAPTORLIFT_TELEOP__GAMEPAD_TELEOP_HPP_
