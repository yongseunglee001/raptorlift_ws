// Copyright 2026 Kandins
// SPDX-License-Identifier: Apache-2.0

#ifndef RAPTORLIFT_HARDWARE_BRIDGE__HARDWARE_BRIDGE_HPP_
#define RAPTORLIFT_HARDWARE_BRIDGE__HARDWARE_BRIDGE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

#include "raptorlift_hardware_bridge/pid_controller.hpp"

#ifdef HAS_MODBUS
#include "raptorlift_hardware_bridge/modbus_driver.hpp"
#endif

namespace raptorlift_hardware_bridge
{

/**
 * @brief Per-axis joint data (matches axis indices FR=0, FL=1, RR=2, RL=3)
 */
struct JointData
{
  std::string name;
  int axis_index{-1};       // Index into driver's axis array
  bool is_traction{false};   // true=traction(velocity), false=steering(position)

  double cmd_position{0.0};  // steering: rad
  double cmd_velocity{0.0};  // traction: m/s (wheel surface speed)
  double state_position{0.0}; // traction: meters, steering: rad
  double state_velocity{0.0}; // traction: m/s, steering: rad/s
  double state_effort{0.0};
  PIDController pid;

  // PLC register equivalents (computed every cycle for logging)
  uint32_t plc_speed_limit{0};   // traction: |cmd_vel| × 100000
  int32_t  plc_pos_raw{0};       // steering: cmd_pos × motor_dir × 10000
  int32_t  plc_torque_raw{0};    // all: torque × motor_dir × 10
};

/**
 * @brief Virtual PLC simulator for simulation mode
 *
 * Simulates 4-axis PLC behavior: 2 traction + 2 steering.
 * Uses proper physics model matching raptorlift_hardware.cpp:
 *   acceleration = (torque - damping * velocity) / inertia
 *
 * Traction axes operate in rad/s internally; the bridge converts to m/s
 * at the boundary. Steering axes operate in rad directly.
 */
class VirtualPLCSimulator
{
public:
  static constexpr int NUM_AXES = 4;
  static constexpr double MAX_STEERING_ANGLE = 0.7;

  /**
   * @brief Construct with physics parameters
   * @param steering_inertia  kg*m^2 (default 0.5)
   * @param traction_inertia  kg*m^2 (default 0.08)
   * @param steering_damping  N*m*s/rad (default 0.5)
   * @param traction_damping  N*m*s/rad (default 1.0)
   */
  explicit VirtualPLCSimulator(
    double steering_inertia = 0.5,
    double traction_inertia = 0.08,
    double steering_damping = 0.5,
    double traction_damping = 1.0)
  : steering_inertia_(steering_inertia)
  , traction_inertia_(traction_inertia)
  , steering_damping_(steering_damping)
  , traction_damping_(traction_damping)
  {
  }

  void setTractionTorque(int axis_index, double torque)
  {
    if (axis_index >= 0 && axis_index < NUM_AXES) {
      traction_torques_[axis_index] = torque;
    }
  }

  void setSteeringTorque(int axis_index, double torque)
  {
    if (axis_index >= 0 && axis_index < NUM_AXES) {
      steering_torques_[axis_index] = torque;
    }
  }

  void update(double dt)
  {
    if (dt <= 0.0) {
      return;
    }

    // Traction axes (FR=0, FL=1): torque -> velocity (rad/s internally)
    for (int i = 0; i < 2; i++) {
      double alpha = (traction_torques_[i] - traction_damping_ * velocities_[i])
                     / traction_inertia_;
      velocities_[i] += alpha * dt;
      positions_[i] += velocities_[i] * dt;
      // No wrapping -- position accumulates (bridge converts to meters externally)
    }

    // Steering axes (RR=2, RL=3): torque -> position (rad)
    for (int i = 2; i < NUM_AXES; i++) {
      double alpha = (steering_torques_[i] - steering_damping_ * velocities_[i])
                     / steering_inertia_;
      velocities_[i] += alpha * dt;
      positions_[i] += velocities_[i] * dt;
      positions_[i] = std::clamp(
        positions_[i], -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
      // Zero velocity at hard stops to prevent snap-back overshoot
      if (std::abs(positions_[i]) >= MAX_STEERING_ANGLE) {
        velocities_[i] = 0.0;
      }
    }
  }

  double getPosition(int axis_index) const
  {
    return (axis_index >= 0 && axis_index < NUM_AXES) ? positions_[axis_index] : 0.0;
  }

  double getVelocity(int axis_index) const
  {
    return (axis_index >= 0 && axis_index < NUM_AXES) ? velocities_[axis_index] : 0.0;
  }

  void logState(rclcpp::Logger logger, rclcpp::Clock & clock) const
  {
    RCLCPP_INFO_THROTTLE(logger, clock, 2000,
      "[VirtualPLC] FR(vel=%.3f) FL(vel=%.3f) RR(pos=%.4f) RL(pos=%.4f)",
      velocities_[0], velocities_[1], positions_[2], positions_[3]);
  }

private:
  // Physics parameters
  double steering_inertia_{0.5};    // kg*m^2
  double traction_inertia_{0.08};   // kg*m^2
  double steering_damping_{0.5};    // N*m*s/rad
  double traction_damping_{1.0};    // N*m*s/rad

  std::array<double, NUM_AXES> positions_{};
  std::array<double, NUM_AXES> velocities_{};
  std::array<double, NUM_AXES> traction_torques_{};
  std::array<double, NUM_AXES> steering_torques_{};
};

/**
 * @brief Hardware bridge node for RaptorLift robot
 *
 * 4-axis Compy-style architecture:
 *   Axis 0 (FR): traction - velocity -> torque
 *   Axis 1 (FL): traction - velocity -> torque
 *   Axis 2 (RR): steering - position -> torque
 *   Axis 3 (RL): steering - position -> torque
 */
class HardwareBridge : public rclcpp::Node
{
public:
  explicit HardwareBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HardwareBridge() override = default;

private:
  void joint_commands_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void control_loop();
  void publish_joint_states();
  void initialize_joints();
  bool initialize_modbus();
  bool write_to_hardware();
  bool read_from_hardware(double dt);
  void checkCommunicationTimeout();
  void handleEmergencyStop();
  bool attemptReconnect();
  void logControlPipeline(double dt);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_commands_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Joint data: ordered as FR, FL, RR, RL (matching axis indices)
  std::vector<JointData> joints_;

#ifdef HAS_MODBUS
  std::unique_ptr<RaptorLiftModbusDriver> modbus_driver_;
#endif

  std::unique_ptr<VirtualPLCSimulator> virtual_plc_;

  // Parameters
  bool simulation_mode_;
  double control_rate_;
  std::string modbus_ip_;
  int modbus_port_;
  bool detailed_logging_;

  // Topic names
  std::string topic_joint_commands_;
  std::string topic_joint_states_;

  // Joint names ordered: [FR, FL, RR, RL]
  std::vector<std::string> traction_joint_names_;
  std::vector<std::string> steering_joint_names_;

  // PID gains
  double steering_kp_, steering_ki_, steering_kd_;
  double traction_kp_, traction_ki_, traction_kd_;
  double max_steering_torque_;
  double max_traction_torque_;

  // Wheel radius for m/s ↔ rad/s conversion at PLC boundary
  double wheel_radius_{0.1715};  // meters

  // Motor mounting directions: FR=+1, FL=-1, RR=+1, RL=-1 (must match modbus_driver)
  static constexpr int MOTOR_DIRS[4] = {+1, -1, +1, -1};

  // PLC scale factors (must match modbus_driver.hpp)
  static constexpr double PLC_POSITION_SCALE = 10000.0;   // rad → int32
  static constexpr double PLC_TORQUE_SCALE = 10.0;        // Nm → ×0.1% rated
  static constexpr double PLC_SPEED_LIMIT_SCALE = 100000.0;

  // Gear state
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gear_state_sub_;
  int current_gear_{0};

  // Communication watchdog (Compy pattern)
  double comm_timeout_;
  double reconnect_interval_;
  std::chrono::steady_clock::time_point last_comm_time_;
  std::chrono::steady_clock::time_point last_reconnect_attempt_;
  bool emergency_stop_active_{false};
  int reconnect_count_{0};

  // Timing
  rclcpp::Time last_control_time_;
  bool modbus_connected_{false};
  bool first_read_done_{false};   // Seed PID measurement after first hardware read
};

}  // namespace raptorlift_hardware_bridge

#endif  // RAPTORLIFT_HARDWARE_BRIDGE__HARDWARE_BRIDGE_HPP_
