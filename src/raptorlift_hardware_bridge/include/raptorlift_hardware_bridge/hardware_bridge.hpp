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

#ifdef HAS_MODBUS
#include "raptorlift_hardware_bridge/modbus_driver.hpp"
#endif

namespace raptorlift_hardware_bridge
{

/**
 * @brief Simple PID controller for motor control
 */
class PIDController
{
public:
  PIDController() = default;

  void setGains(double kp, double ki, double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void setLimits(double min_output, double max_output)
  {
    min_output_ = min_output;
    max_output_ = max_output;
  }

  /**
   * @brief Compute PID output
   * @param error Current error (setpoint - measurement)
   * @param measurement Current measurement (for derivative-on-measurement to avoid kick)
   * @param dt Time step
   */
  double compute(double error, double measurement, double dt)
  {
    if (dt <= 0.0) {
      return 0.0;
    }

    integral_ += error * dt;

    // Anti-windup
    integral_ = std::clamp(integral_, -max_integral_, max_integral_);

    // Derivative on measurement (not error) to avoid derivative kick
    double derivative = -(measurement - prev_measurement_) / dt;
    prev_measurement_ = measurement;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    return std::clamp(output, min_output_, max_output_);
  }

  // Legacy overload for backward compatibility
  double compute(double error, double dt)
  {
    return compute(error, -error, dt);  // approximation when measurement unavailable
  }

  void reset()
  {
    integral_ = 0.0;
    prev_measurement_ = 0.0;
  }

private:
  double kp_{0.0};
  double ki_{0.0};
  double kd_{0.0};
  double integral_{0.0};
  double prev_measurement_{0.0};
  double min_output_{-100.0};
  double max_output_{100.0};
  double max_integral_{50.0};
};

/**
 * @brief Per-axis joint data (matches axis indices FR=0, FL=1, RR=2, RL=3)
 */
struct JointData
{
  std::string name;
  int axis_index{-1};       // Index into driver's axis array
  bool is_traction{false};   // true=traction(velocity), false=steering(position)

  double cmd_position{0.0};
  double cmd_velocity{0.0};
  double state_position{0.0};
  double state_velocity{0.0};
  double state_effort{0.0};
  PIDController pid;
};

/**
 * @brief Virtual PLC simulator for simulation mode
 *
 * Simulates 4-axis PLC behavior: 2 traction + 2 steering.
 */
class VirtualPLCSimulator
{
public:
  static constexpr int NUM_AXES = 4;
  static constexpr double MAX_STEERING_ANGLE = 0.7;

  VirtualPLCSimulator() = default;

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

    // Traction axes (FR=0, FL=1): torque -> velocity
    for (int i = 0; i < 2; i++) {
      double acceleration = traction_torques_[i] * 0.1;
      velocities_[i] += acceleration * dt;
      velocities_[i] *= 0.98;  // Damping
      positions_[i] += velocities_[i] * dt;
      // Wrap position
      while (positions_[i] > M_PI) { positions_[i] -= 2 * M_PI; }
      while (positions_[i] < -M_PI) { positions_[i] += 2 * M_PI; }
    }

    // Steering axes (RR=2, RL=3): torque -> position
    for (int i = 2; i < NUM_AXES; i++) {
      double acceleration = steering_torques_[i] * 0.01;
      velocities_[i] += acceleration * dt;
      velocities_[i] *= 0.95;  // Damping
      positions_[i] += velocities_[i] * dt;
      positions_[i] = std::clamp(
        positions_[i], -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
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
  bool read_from_hardware();
  void checkCommunicationTimeout();
  void handleEmergencyStop();
  bool attemptReconnect();

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
};

}  // namespace raptorlift_hardware_bridge

#endif  // RAPTORLIFT_HARDWARE_BRIDGE__HARDWARE_BRIDGE_HPP_
