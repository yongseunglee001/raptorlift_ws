// Copyright 2026 Kandins
// SPDX-License-Identifier: Apache-2.0

#ifndef RAPTORLIFT_HARDWARE_BRIDGE__MODBUS_DRIVER_HPP_
#define RAPTORLIFT_HARDWARE_BRIDGE__MODBUS_DRIVER_HPP_

#include <modbus/modbus.h>
#include <vector>
#include <string>
#include <mutex>
#include <memory>
#include <cstdint>
#include <atomic>
#include <array>
#include <chrono>

namespace raptorlift_hardware_bridge
{

/**
 * @brief Axis type for RaptorLift's 4-wheel configuration
 */
enum class AxisType
{
  TRACTION,  // FR, FL - velocity -> torque
  STEERING   // RR, RL - position -> torque
};

/**
 * @brief Modbus driver for RaptorLift 4-wheel Ackermann robot
 *
 * Compy-style unified AxisData for 4 axes:
 *   Axis 0 (FR): Traction - D0,D1=speed_limit  D2,D3=target_torque  D16,D17=rotation_speed
 *   Axis 1 (FL): Traction - D4,D5=speed_limit  D6,D7=target_torque  D18,D19=rotation_speed
 *   Axis 2 (RR): Steering - D8,D9=target_pos   D10,D11=target_torque D20,D21=actual_pos
 *   Axis 3 (RL): Steering - D12,D13=target_pos  D14,D15=target_torque D22,D23=actual_pos
 *
 * Motor enable coils: M1(FR), M2(FL), M3(RR), M4(RL)
 * PLC reset: M0 pulse (high -> 10ms -> low)
 */
class RaptorLiftModbusDriver
{
public:
  static constexpr int NUM_AXES = 4;

  // Axis indices
  static constexpr int AXIS_FR = 0;  // Front Right (traction)
  static constexpr int AXIS_FL = 1;  // Front Left  (traction)
  static constexpr int AXIS_RR = 2;  // Rear Right  (steering)
  static constexpr int AXIS_RL = 3;  // Rear Left   (steering)

  // Axis types
  static constexpr AxisType axis_types[NUM_AXES] = {
    AxisType::TRACTION,  // FR
    AxisType::TRACTION,  // FL
    AxisType::STEERING,  // RR
    AxisType::STEERING   // RL
  };

  // Motor mounting directions: FR,RR same (+1); FL,RL same (-1); FL opposite to FR
  static constexpr int motor_dirs[NUM_AXES] = {+1, -1, +1, -1};

  // Scale factors
  static constexpr double POSITION_SCALE = 10000.0;  // rad -> int32
  static constexpr double VELOCITY_SCALE = 10000.0;  // rad/s -> int32
  static constexpr double TORQUE_SCALE = 10.0;       // Nm -> 0.1% rated

  // Emergency stop
  static constexpr double BRAKE_TORQUE = 300.0;

  // Throttled logging interval
  static constexpr int LOG_INTERVAL_MS = 2000;

  /**
   * @brief Per-axis data (Compy pattern)
   *
   * For traction axes: command_1 = speed_limit, command_2 = target_torque
   * For steering axes: command_1 = target_position (encoded as uint32), command_2 = target_torque
   * feedback: traction = rotation_speed, steering = actual_position
   */
  struct AxisData
  {
    uint32_t command_1{0};     // speed_limit (traction) or target_position (steering)
    int32_t  command_2{0};     // target_torque (both types)
    int32_t  feedback{0};      // rotation_speed (traction) or actual_position (steering)

    void init_safe()
    {
      command_1 = 0;
      command_2 = 0;
      feedback = 0;
    }
  };

  /**
   * @brief Complete data packet for 4 axes (Compy pattern)
   */
  struct ModbusDataPacket
  {
    AxisData axes[NUM_AXES];  // 4 axes: FR, FL, RR, RL

    void init_safe()
    {
      for (int i = 0; i < NUM_AXES; i++) {
        axes[i].init_safe();
      }
    }
  };

  RaptorLiftModbusDriver();
  ~RaptorLiftModbusDriver();

  // ==================== CONNECTION ====================

  bool connect(const std::string & ip = "192.168.2.1", int port = 502);
  void disconnect();
  bool isConnected() const;

  // ==================== COMMAND INTERFACE ====================

  /**
   * @brief Set traction axis commands (FR, FL)
   * @param axis_index AXIS_FR or AXIS_FL
   * @param speed_limit Speed limit in PLC units
   * @param torque Target torque (Nm, before motor_dir)
   */
  void setTractionCommand(int axis_index, uint32_t speed_limit, double torque);

  /**
   * @brief Set steering axis commands (RR, RL)
   * @param axis_index AXIS_RR or AXIS_RL
   * @param position Target position (rad)
   * @param torque Target torque (Nm, before motor_dir)
   */
  void setSteeringCommand(int axis_index, double position, double torque);

  /**
   * @brief Send all axis commands to PLC in one Modbus write (D0-D15)
   *
   * Applies motor_dirs to torques before sending.
   * @return true if successful
   */
  bool sendPacket();

  /**
   * @brief Read feedback from PLC (D16-D23)
   * @return true if successful
   */
  bool readFeedback();

  // ==================== FEEDBACK INTERFACE ====================

  /**
   * @brief Get traction axis feedback velocity
   * @param axis_index AXIS_FR or AXIS_FL
   * @return velocity in rad/s (motor_dir compensated)
   */
  double getTractionVelocity(int axis_index) const;

  /**
   * @brief Get steering axis feedback position
   * @param axis_index AXIS_RR or AXIS_RL
   * @return position in rad (motor_dir compensated)
   */
  double getSteeringPosition(int axis_index) const;

  /**
   * @brief Get raw feedback value for any axis
   */
  int32_t getRawFeedback(int axis_index) const;

  // ==================== SAFETY ====================

  /**
   * @brief Emergency stop: zero speed + brake torque (Compy pattern)
   */
  bool emergencyStop();

  /**
   * @brief Enable/disable all motors (thread-safe)
   */
  bool setMotorEnable(bool enable);

  /**
   * @brief Get direct access to packet for debugging
   */
  const ModbusDataPacket & getPacket() const { return pkt_; }

  /**
   * @brief Log current axis status
   */
  void logAxisStatus() const;

private:
  // PLC register addresses
  static constexpr int D_REGISTERS_START = 0;
  static constexpr int M_REGISTERS_START = 8192;

  // Command registers: 4 regs per axis, 4 axes = 16 regs (D0-D15)
  static constexpr int NUM_COMMAND_REGS = NUM_AXES * 4;

  // Feedback registers: 2 regs per axis, 4 axes = 8 regs (D16-D23)
  static constexpr int FEEDBACK_REG_START = 16;
  static constexpr int NUM_FEEDBACK_REGS = NUM_AXES * 2;

  // PLC coils
  static constexpr int M_PLC_RESET = M_REGISTERS_START + 0;       // M0
  static constexpr int M_MOTOR_BASE = M_REGISTERS_START + 1;      // M1-M4

  // Internal methods
  void sendPLCResetPulse();
  bool setMotorEnableUnlocked(bool enable);

  // Conversion helpers
  int32_t positionToRaw(double position) const;
  double rawToPosition(int32_t raw) const;
  int32_t velocityToRaw(double velocity) const;
  double rawToVelocity(int32_t raw) const;
  int32_t torqueToRaw(double torque) const;

  // Internal unlocked send (must hold mutex_)
  bool sendPacketUnlocked();

  modbus_t * ctx_;
  std::atomic<bool> connected_;
  mutable std::mutex mutex_;

  // Unified data packet (Compy pattern)
  ModbusDataPacket pkt_;

  // Throttled logging
  std::chrono::steady_clock::time_point last_log_time_;
};

}  // namespace raptorlift_hardware_bridge

#endif  // RAPTORLIFT_HARDWARE_BRIDGE__MODBUS_DRIVER_HPP_
