// Copyright 2024 RaptorLift
// Licensed under the Apache License, Version 2.0
//
// Unified PID controller - canonical implementation in raptorlift_hardware_bridge.
// This header provides a namespace alias so existing code using
// raptorlift_hardware::PIDController continues to compile unchanged.

#ifndef RAPTORLIFT_HARDWARE__PID_CONTROLLER_HPP_
#define RAPTORLIFT_HARDWARE__PID_CONTROLLER_HPP_

#include "raptorlift_hardware_bridge/pid_controller.hpp"

namespace raptorlift_hardware
{
  using PIDController = raptorlift_hardware_bridge::PIDController;
}  // namespace raptorlift_hardware

#endif  // RAPTORLIFT_HARDWARE__PID_CONTROLLER_HPP_
