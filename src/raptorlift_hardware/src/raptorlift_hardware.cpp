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

int RaptorLiftHardware::getAxisIndex(const std::string & joint_name)
{
  // Map joint name to PLC axis index: FR=0, FL=1, RR=2, RL=3
  if (joint_name.find("front_right") != std::string::npos) {
    return 0;
  }
  if (joint_name.find("front_left") != std::string::npos) {
    return 1;
  }
  if (joint_name.find("rear_right") != std::string::npos) {
    return 2;
  }
  if (joint_name.find("rear_left") != std::string::npos) {
    return 3;
  }
  return -1;  // Unknown joint
}

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

  // Load Modbus connection parameters (used when simulation_mode=false)
  auto ip_it = info.hardware_parameters.find("modbus_ip");
  if (ip_it != info.hardware_parameters.end()) {
    modbus_ip_ = ip_it->second;
  }
  auto port_it = info.hardware_parameters.find("modbus_port");
  if (port_it != info.hardware_parameters.end()) {
    try {
      modbus_port_ = std::stoi(port_it->second);
    } catch (...) {
      modbus_port_ = 502;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
    "Simulation mode: %s", simulation_mode_ ? "true" : "false");
  if (!simulation_mode_) {
    RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
      "Modbus PLC: %s:%d", modbus_ip_.c_str(), modbus_port_);
  }
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
      sj.axis_index = getAxisIndex(joint.name);
      sj.pid.setGains(steering_kp_, steering_ki_, steering_kd_);
      sj.pid.setOutputLimits(-max_steering_torque_, max_steering_torque_);
      steering_joints_.push_back(sj);
      RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
        "Added steering joint: %s (pos+vel cmd, axis=%d)", joint.name.c_str(), sj.axis_index);
    } else if (has_velocity_cmd) {
      TractionJoint tj;
      tj.name = joint.name;
      tj.axis_index = getAxisIndex(joint.name);
      tj.pid.setGains(traction_kp_, traction_ki_, traction_kd_);
      tj.pid.setOutputLimits(-max_traction_torque_, max_traction_torque_);
      traction_joints_.push_back(tj);
      RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
        "Added traction joint: %s (vel cmd, axis=%d)", joint.name.c_str(), tj.axis_index);
    }
  }

  // Load simulation dynamics parameters from URDF (optional)
  steering_inertia_ = getParameter(info, "steering_inertia", 0.5);
  traction_inertia_ = getParameter(info, "traction_inertia", 0.08);
  steering_damping_ = getParameter(info, "steering_damping", 0.5);
  traction_damping_ = getParameter(info, "traction_damping", 1.0);
  max_steering_angle_ = getParameter(info, "max_steering_angle", 0.7);
  wheel_radius_ = getParameter(info, "wheel_radius", 0.1715);

  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
    "Wheel radius: %.4f m (joint velocity in m/s, PLC in rad/s)", wheel_radius_);
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
  // Also seed PID prev_measurement_ to avoid derivative spike on first cycle
  for (auto & joint : steering_joints_) {
    joint.cmd_position = joint.state_position;
    joint.cmd_velocity = 0.0;
    joint.pid.reset();
    joint.pid.seedMeasurement(joint.state_position);
  }

  for (auto & joint : traction_joints_) {
    joint.cmd_velocity = 0.0;  // Start with zero velocity
    joint.pid.reset();
    joint.pid.seedMeasurement(joint.state_velocity);
  }

  // Connect to PLC via Modbus (real hardware only)
  if (!simulation_mode_) {
#ifdef HAS_MODBUS
    modbus_driver_ = std::make_unique<raptorlift_hardware_bridge::RaptorLiftModbusDriver>();
    if (!modbus_driver_->connect(modbus_ip_, modbus_port_)) {
      RCLCPP_ERROR(rclcpp::get_logger("RaptorLiftHardware"),
        "Failed to connect to PLC at %s:%d", modbus_ip_.c_str(), modbus_port_);
      modbus_driver_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
      "Connected to PLC at %s:%d (reset pulse + motors enabled)",
      modbus_ip_.c_str(), modbus_port_);
#else
    RCLCPP_ERROR(rclcpp::get_logger("RaptorLiftHardware"),
      "Built without libmodbus support - cannot use real hardware mode. "
      "Install libmodbus-dev and rebuild.");
    return hardware_interface::CallbackReturn::ERROR;
#endif
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

  // Disconnect from PLC (real hardware only)
#ifdef HAS_MODBUS
  if (modbus_driver_) {
    RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"),
      "Emergency stop + disconnecting from PLC...");
    modbus_driver_->emergencyStop();
    modbus_driver_->disconnect();
    modbus_driver_.reset();
  }
#endif

  RCLCPP_INFO(rclcpp::get_logger("RaptorLiftHardware"), "Successfully deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RaptorLiftHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(rclcpp::get_logger("RaptorLiftHardware"),
    "Error state entered - performing emergency cleanup");

  // Stop all motion
  for (auto & joint : traction_joints_) {
    joint.cmd_velocity = 0.0;
  }

  // Emergency stop and disconnect from PLC
#ifdef HAS_MODBUS
  if (modbus_driver_) {
    modbus_driver_->emergencyStop();
    modbus_driver_->disconnect();
    modbus_driver_.reset();
    RCLCPP_WARN(rclcpp::get_logger("RaptorLiftHardware"),
      "PLC emergency stop sent, motors disabled, connection closed");
  }
#endif

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
    // Real hardware: read encoder feedback from PLC via Modbus
    double dt = period.seconds();
    if (dt <= 0.0) {
      dt = 0.01;
    }

#ifdef HAS_MODBUS
    if (!modbus_driver_ || !modbus_driver_->readFeedback()) {
      static rclcpp::Clock steady_clock_read(RCL_STEADY_TIME);
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("RaptorLiftHardware"),
        steady_clock_read, 2000, "Failed to read feedback from PLC");
      return hardware_interface::return_type::ERROR;
    }

    // Traction: PLC returns rad/s; convert to m/s at boundary
    for (auto & joint : traction_joints_) {
      joint.state_velocity =
        modbus_driver_->getTractionVelocity(joint.axis_index) * wheel_radius_;
      joint.state_position += joint.state_velocity * dt;
    }

    // Steering: PLC provides actual position, differentiate for velocity
    for (auto & joint : steering_joints_) {
      double prev_position = joint.state_position;
      joint.state_position =
        modbus_driver_->getSteeringPosition(joint.axis_index);
      if (dt > 0.0) {
        joint.state_velocity =
          (joint.state_position - prev_position) / dt;
      }
    }
#else
    static rclcpp::Clock steady_clock_nomod(RCL_STEADY_TIME);
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("RaptorLiftHardware"),
      steady_clock_nomod, 5000,
      "No libmodbus support - cannot read from hardware");
    return hardware_interface::return_type::ERROR;
#endif
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
  // State velocity/position are in m/s and meters; physics uses rad/s internally
  for (size_t i = 0; i < traction_joints_.size(); i++) {
    auto & joint = traction_joints_[i];
    double torque = traction_torques_[i];

    // Convert m/s -> rad/s for physics simulation
    double vel_rad = joint.state_velocity / wheel_radius_;
    double alpha = (torque - traction_damping_ * vel_rad) / traction_inertia_;
    vel_rad += alpha * dt;
    // Convert back to m/s
    joint.state_velocity = vel_rad * wheel_radius_;
    joint.state_position += joint.state_velocity * dt;
    // No wrapping — position is meters (distance traveled), not angle
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
  // Negate cmd_position for rear-steer: ackermann_steering_controller assumes
  // front-steer geometry. For rear-steer, +controller_angle must become
  // -actual_angle to maintain ROS convention (+angular.z = LEFT turn).
  for (size_t i = 0; i < steering_joints_.size(); i++) {
    auto & joint = steering_joints_[i];
    double pos_error = (-joint.cmd_position) - joint.state_position;
    double torque = joint.pid.compute(pos_error, joint.state_position, dt);
    joint.state_effort = torque;
    steering_torques_[i] = torque;
  }

  // Traction: velocity error -> PID -> torque (derivative on measurement)
  // Convert m/s -> rad/s before PID to preserve tuned gains
  for (size_t i = 0; i < traction_joints_.size(); i++) {
    auto & joint = traction_joints_[i];
    double vel_error_rad = (joint.cmd_velocity - joint.state_velocity) / wheel_radius_;
    double measurement_rad = joint.state_velocity / wheel_radius_;
    double torque = joint.pid.compute(vel_error_rad, measurement_rad, dt);
    joint.state_effort = torque;
    traction_torques_[i] = torque;
  }

  if (!simulation_mode_) {
    // Real hardware: send PID torque commands to PLC via Modbus
#ifdef HAS_MODBUS
    if (modbus_driver_) {
      // Traction: speed_limit (convert m/s -> rad/s) + torque
      for (size_t i = 0; i < traction_joints_.size(); i++) {
        double speed_raw =
          std::abs(traction_joints_[i].cmd_velocity / wheel_radius_) * PLC_SPEED_LIMIT_SCALE;
        uint32_t speed_limit = static_cast<uint32_t>(
          std::min(speed_raw, static_cast<double>(std::numeric_limits<uint32_t>::max())));
        modbus_driver_->setTractionCommand(
          traction_joints_[i].axis_index, speed_limit, traction_torques_[i]);
      }

      // Steering: target position + torque (axis mapped by joint name)
      // Send negated position to PLC (rear-steer inversion)
      for (size_t i = 0; i < steering_joints_.size(); i++) {
        modbus_driver_->setSteeringCommand(
          steering_joints_[i].axis_index,
          -steering_joints_[i].cmd_position,
          steering_torques_[i]);
      }

      if (!modbus_driver_->sendPacket()) {
        static rclcpp::Clock steady_clock_write(RCL_STEADY_TIME);
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("RaptorLiftHardware"),
          steady_clock_write, 2000, "Failed to send commands to PLC");
        return hardware_interface::return_type::ERROR;
      }
    }
#endif
  }

  return hardware_interface::return_type::OK;
}

}  // namespace raptorlift_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  raptorlift_hardware::RaptorLiftHardware,
  hardware_interface::SystemInterface)
