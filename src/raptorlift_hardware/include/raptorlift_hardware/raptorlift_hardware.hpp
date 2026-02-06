// Copyright 2024 RaptorLift
// Licensed under the Apache License, Version 2.0

#ifndef RAPTORLIFT_HARDWARE__RAPTORLIFT_HARDWARE_HPP_
#define RAPTORLIFT_HARDWARE__RAPTORLIFT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "raptorlift_hardware/pid_controller.hpp"
#include "raptorlift_hardware/visibility_control.h"

namespace raptorlift_hardware
{

class RaptorLiftHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RaptorLiftHardware)

  RAPTORLIFT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  RAPTORLIFT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  RAPTORLIFT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  RAPTORLIFT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  RAPTORLIFT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  RAPTORLIFT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  RAPTORLIFT_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  RAPTORLIFT_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Steering joint data (Position + Velocity command -> Torque)
  struct SteeringJoint
  {
    std::string name;
    double cmd_position{0.0};
    double cmd_velocity{0.0};  // For ackermann_steering_controller compatibility
    double state_position{0.0};
    double state_velocity{0.0};
    double state_effort{0.0};
    PIDController pid;
  };

  // Traction joint data (Velocity command -> Torque)
  struct TractionJoint
  {
    std::string name;
    double cmd_velocity{0.0};
    double state_position{0.0};
    double state_velocity{0.0};
    double state_effort{0.0};
    PIDController pid;
  };

  std::vector<SteeringJoint> steering_joints_;
  std::vector<TractionJoint> traction_joints_;

  // PID gains (loaded from parameters)
  double steering_kp_{50.0};
  double steering_ki_{0.0};
  double steering_kd_{5.0};
  double traction_kp_{10.0};
  double traction_ki_{5.0};
  double traction_kd_{0.0};

  // Torque limits
  double max_steering_torque_{100.0};
  double max_traction_torque_{150.0};

  // Simulation mode flag
  bool simulation_mode_{true};

  // Simulation parameters (loadable from URDF)
  double steering_inertia_{0.5};   // kg*m^2
  double traction_inertia_{0.08};  // kg*m^2 (wheel moment of inertia)
  double steering_damping_{0.5};
  double traction_damping_{1.0};
  double max_steering_angle_{0.7}; // rad (±40°)

  // Torques computed in write(), applied as dynamics in read()
  // (follows ros2_control convention: read→controller→write)
  std::vector<double> steering_torques_;
  std::vector<double> traction_torques_;

  // Helper function to get parameter with default
  double getParameter(
    const hardware_interface::HardwareInfo & info,
    const std::string & name,
    double default_value) const;
};

}  // namespace raptorlift_hardware

#endif  // RAPTORLIFT_HARDWARE__RAPTORLIFT_HARDWARE_HPP_
