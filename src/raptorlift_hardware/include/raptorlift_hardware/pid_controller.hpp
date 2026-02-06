// Copyright 2024 RaptorLift
// Licensed under the Apache License, Version 2.0

#ifndef RAPTORLIFT_HARDWARE__PID_CONTROLLER_HPP_
#define RAPTORLIFT_HARDWARE__PID_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

namespace raptorlift_hardware
{

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

  void setOutputLimits(double min_output, double max_output)
  {
    min_output_ = min_output;
    max_output_ = max_output;
  }

  void setIntegralLimits(double max_integral)
  {
    max_integral_ = max_integral;
  }

  /**
   * @brief Compute PID output with derivative-on-measurement to avoid kick
   * @param error Current error (setpoint - measurement)
   * @param measurement Current measurement value
   * @param dt Time step in seconds
   */
  double compute(double error, double measurement, double dt)
  {
    if (dt <= 0.0) {
      return 0.0;
    }

    double p_term = kp_ * error;

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -max_integral_, max_integral_);
    double i_term = ki_ * integral_;

    // Derivative on measurement (not error) to avoid derivative kick
    double derivative = -(measurement - prev_measurement_) / dt;
    double d_term = kd_ * derivative;
    prev_measurement_ = measurement;

    double output = p_term + i_term + d_term;
    return std::clamp(output, min_output_, max_output_);
  }

  // Legacy overload
  double compute(double error, double dt)
  {
    return compute(error, -error, dt);
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

  double min_output_{-1000.0};
  double max_output_{1000.0};
  double max_integral_{100.0};
};

}  // namespace raptorlift_hardware

#endif  // RAPTORLIFT_HARDWARE__PID_CONTROLLER_HPP_
