// Copyright 2024 RaptorLift
// Licensed under the Apache License, Version 2.0

#include "raptorlift_hardware/raptorlift_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace raptorlift_hardware
{

double RaptorLiftHardware::getParameter(
  const hardware_interface::HardwareInfo & info,
  const std::string & name,
  double default_value) const
{
  auto it = info.hardware_parameters.find(name);
  if (it != info.hardware_parameters.end()) {
    try {
      return std::stod(it->second);
    } catch (...) {
      return default_value;
    }
  }
  return default_value;
}

hardware_interface::CallbackReturn RaptorLiftHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load parameters from URDF
  simulation_mode_ = (info.hardware_parameters.count("simulation_mode") > 0) &&
    (info.hardware_parameters.at("simulation_mode") == "true");

  steering_kp_ = getParameter(info, "steering_kp", 50.0);
  steering_ki_ = getParameter(info, "steering_ki", 0.0);
  steering_kd_ = getParameter(info, "steering_kd", 5.0);
  traction_kp_ = getParameter(info, "traction_kp", 10.0);
  traction_ki_ = getParameter(info, "traction_ki", 5.0);
  traction_kd_ = getParameter(info, "traction_kd", 0.0);
  max_steering_torque_ = getParameter(info, "max_steering_torque", 100.0);
  max_traction_torque_ = getParameter(info, "max_traction_torque", 150.0);

  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
    "Simulation mode: %s", simulation_mode_ ? "true" : "false");
  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
    "Steering PID: kp=%.2f, ki=%.2f, kd=%.2f", steering_kp_, steering_ki_, steering_kd_);
  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
    "Traction PID: kp=%.2f, ki=%.2f, kd=%.2f", traction_kp_, traction_ki_, traction_kd_);

  // Parse joints from URDF
  for (const auto & joint : info.joints) {
    bool has_position_cmd = false;
    bool has_velocity_cmd = false;

    // Check command interfaces to determine joint type
    for (const auto & cmd_if : joint.command_interfaces) {
      if (cmd_if.name == hardware_interface::HW_IF_POSITION) {
        has_position_cmd = true;
      }
      if (cmd_if.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_cmd = true;
      }
    }

    // Steering joints have position command (may also have velocity for controller compatibility)
    // Traction joints have only velocity command (no position)
    // Passive joints have no command interfaces (state only)
    if (has_position_cmd) {
      SteeringJoint sj;
      sj.name = joint.name;
      sj.pid.setGains(steering_kp_, steering_ki_, steering_kd_);
      sj.pid.setOutputLimits(-max_steering_torque_, max_steering_torque_);
      steering_joints_.push_back(sj);
      RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
        "Added steering joint: %s (pos+vel cmd)", joint.name.c_str());
    } else if (has_velocity_cmd) {
      TractionJoint tj;
      tj.name = joint.name;
      tj.pid.setGains(traction_kp_, traction_ki_, traction_kd_);
      tj.pid.setOutputLimits(-max_traction_torque_, max_traction_torque_);
      traction_joints_.push_back(tj);
      RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
        "Added traction joint: %s (vel cmd)", joint.name.c_str());
    }
  }

  // Load simulation dynamics parameters from URDF (optional)
  steering_inertia_ = getParameter(info, "steering_inertia", 0.5);
  traction_inertia_ = getParameter(info, "traction_inertia", 0.08);
  steering_damping_ = getParameter(info, "steering_damping", 0.5);
  traction_damping_ = getParameter(info, "traction_damping", 1.0);
  max_steering_angle_ = getParameter(info, "max_steering_angle", 0.7);

  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
    "Total: %zu steering, %zu traction joints",
    steering_joints_.size(), traction_joints_.size());

  // Allocate torque buffers for sim dynamics
  steering_torques_.resize(steering_joints_.size(), 0.0);
  traction_torques_.resize(traction_joints_.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RaptorLiftHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"), "Configuring...");

  // Reset all states
  for (auto & joint : steering_joints_) {
    joint.cmd_position = 0.0;
    joint.cmd_velocity = 0.0;
    joint.state_position = 0.0;
    joint.state_velocity = 0.0;
    joint.state_effort = 0.0;
    joint.pid.reset();
  }

  for (auto & joint : traction_joints_) {
    joint.cmd_velocity = 0.0;
    joint.state_position = 0.0;
    joint.state_velocity = 0.0;
    joint.state_effort = 0.0;
    joint.pid.reset();
  }

  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"), "Successfully configured.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RaptorLiftHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"), "Activating...");

  // Initialize commands to current states (prevent jumps)
  for (auto & joint : steering_joints_) {
    joint.cmd_position = joint.state_position;
    joint.cmd_velocity = 0.0;
  }

  for (auto & joint : traction_joints_) {
    joint.cmd_velocity = 0.0;  // Start with zero velocity
  }

  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"), "Successfully activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RaptorLiftHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"), "Deactivating...");

  // Stop all motion
  for (auto & joint : traction_joints_) {
    joint.cmd_velocity = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"), "Successfully deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RaptorLiftHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Steering joints
  for (auto & joint : steering_joints_) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.state_position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.state_velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &joint.state_effort));
  }

  // Traction joints
  for (auto & joint : traction_joints_) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.state_position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.state_velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &joint.state_effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RaptorLiftHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Steering joints: position + velocity command (for ackermann_steering_controller)
  for (auto & joint : steering_joints_) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.cmd_position));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.cmd_velocity));
  }

  // Traction joints: velocity command
  for (auto & joint : traction_joints_) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &joint.cmd_velocity));
  }

  return command_interfaces;
}

hardware_interface::return_type RaptorLiftHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!simulation_mode_) {
    // TODO: Read encoder values from real hardware (Modbus/EtherCAT)
    return hardware_interface::return_type::OK;
  }

  // Simulation: apply torques computed in previous write() to update states
  double dt = period.seconds();
  if (dt <= 0.0) {
    dt = 0.01;
  }

  // Steering joints: torque -> dynamics -> position/velocity
  for (size_t i = 0; i < steering_joints_.size(); i++) {
    auto & joint = steering_joints_[i];
    double torque = steering_torques_[i];

    double alpha = (torque - steering_damping_ * joint.state_velocity) / steering_inertia_;
    joint.state_velocity += alpha * dt;
    joint.state_position += joint.state_velocity * dt;

    joint.state_position = std::clamp(
      joint.state_position, -max_steering_angle_, max_steering_angle_);
    if (std::abs(joint.state_position) >= max_steering_angle_) {
      joint.state_velocity = 0.0;
    }
  }

  // Traction joints: torque -> dynamics -> velocity/position
  for (size_t i = 0; i < traction_joints_.size(); i++) {
    auto & joint = traction_joints_[i];
    double torque = traction_torques_[i];

    double alpha = (torque - traction_damping_ * joint.state_velocity) / traction_inertia_;
    joint.state_velocity += alpha * dt;
    joint.state_position += joint.state_velocity * dt;

    // Wrap position to [-pi, pi] for continuous joint
    while (joint.state_position > M_PI) {
      joint.state_position -= 2.0 * M_PI;
    }
    while (joint.state_position < -M_PI) {
      joint.state_position += 2.0 * M_PI;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RaptorLiftHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) {
    dt = 0.01;
  }

  // Steering: position error -> PID -> torque (derivative on measurement)
  for (size_t i = 0; i < steering_joints_.size(); i++) {
    auto & joint = steering_joints_[i];
    double pos_error = joint.cmd_position - joint.state_position;
    double torque = joint.pid.compute(pos_error, joint.state_position, dt);
    joint.state_effort = torque;
    steering_torques_[i] = torque;
  }

  // Traction: velocity error -> PID -> torque (derivative on measurement)
  for (size_t i = 0; i < traction_joints_.size(); i++) {
    auto & joint = traction_joints_[i];
    double vel_error = joint.cmd_velocity - joint.state_velocity;
    double torque = joint.pid.compute(vel_error, joint.state_velocity, dt);
    joint.state_effort = torque;
    traction_torques_[i] = torque;
  }

  if (!simulation_mode_) {
    // TODO: Send torque commands to real servos via Modbus/EtherCAT
  }

  return hardware_interface::return_type::OK;
}

}  // namespace raptorlift_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  raptorlift_hardware::RaptorLiftHardware,
  hardware_interface::SystemInterface)
