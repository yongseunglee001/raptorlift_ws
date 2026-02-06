// Copyright 2026 Kandins
// SPDX-License-Identifier: Apache-2.0

#include "raptorlift_hardware_bridge/modbus_driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <chrono>
#include <thread>

namespace raptorlift_hardware_bridge
{

RaptorLiftModbusDriver::RaptorLiftModbusDriver()
: ctx_(nullptr), connected_(false), last_log_time_(std::chrono::steady_clock::now())
{
  pkt_.init_safe();
}

RaptorLiftModbusDriver::~RaptorLiftModbusDriver()
{
  disconnect();
}

// ==================== CONNECTION ====================

bool RaptorLiftModbusDriver::connect(const std::string & ip, int port)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (connected_.load()) {
    return true;
  }

  auto logger = rclcpp::get_logger("RaptorLiftModbusDriver");

  ctx_ = modbus_new_tcp(ip.c_str(), port);
  if (!ctx_) {
    RCLCPP_ERROR(logger, "Failed to create Modbus context for %s:%d", ip.c_str(), port);
    return false;
  }

  if (modbus_set_slave(ctx_, 1) == -1) {
    RCLCPP_WARN(logger, "Failed to set Modbus slave ID: %s", modbus_strerror(errno));
  }

  modbus_set_response_timeout(ctx_, 0, 500000);

  if (modbus_connect(ctx_) == -1) {
    RCLCPP_ERROR(logger, "Modbus connection failed: %s", modbus_strerror(errno));
    modbus_free(ctx_);
    ctx_ = nullptr;
    return false;
  }

  RCLCPP_INFO(logger, "Connected to PLC at %s:%d", ip.c_str(), port);

  // PLC reset pulse: M0 high -> 10ms -> M0 low (Compy pattern)
  sendPLCResetPulse();

  // Enable per-motor drives: M1-M4
  setMotorEnableUnlocked(true);

  connected_.store(true);
  pkt_.init_safe();

  return true;
}

void RaptorLiftModbusDriver::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!ctx_) {
    connected_.store(false);
    return;
  }

  auto logger = rclcpp::get_logger("RaptorLiftModbusDriver");

  // Step 1: Zero all commands (Compy pattern: init_safe + write)
  pkt_.init_safe();

  // Write zero packet directly (already holding mutex_)
  std::vector<uint16_t> regs(NUM_COMMAND_REGS, 0);
  modbus_write_registers(ctx_, D_REGISTERS_START, NUM_COMMAND_REGS, regs.data());
  RCLCPP_INFO(logger, "Sent zero commands before disconnect");

  // Step 2: Disable motors M1-M4
  setMotorEnableUnlocked(false);

  // Step 3: Close connection
  modbus_close(ctx_);
  modbus_free(ctx_);
  ctx_ = nullptr;
  connected_.store(false);
}

bool RaptorLiftModbusDriver::isConnected() const
{
  return connected_.load();
}

// ==================== COMMAND INTERFACE ====================

void RaptorLiftModbusDriver::setTractionCommand(
  int axis_index, uint32_t speed_limit, double torque)
{
  if (axis_index < 0 || axis_index >= NUM_AXES) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  pkt_.axes[axis_index].command_1 = speed_limit;
  pkt_.axes[axis_index].command_2 = torqueToRaw(torque * motor_dirs[axis_index]);
}

void RaptorLiftModbusDriver::setSteeringCommand(
  int axis_index, double position, double torque)
{
  if (axis_index < 0 || axis_index >= NUM_AXES) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  // Position stored as int32 reinterpreted through uint32 register
  int32_t raw_pos = positionToRaw(position * motor_dirs[axis_index]);
  pkt_.axes[axis_index].command_1 = static_cast<uint32_t>(raw_pos);
  pkt_.axes[axis_index].command_2 = torqueToRaw(torque * motor_dirs[axis_index]);
}

bool RaptorLiftModbusDriver::sendPacket()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return sendPacketUnlocked();
}

bool RaptorLiftModbusDriver::sendPacketUnlocked()
{
  if (!ctx_ || !connected_.load()) {
    return false;
  }

  auto logger = rclcpp::get_logger("RaptorLiftModbusDriver");

  // Build register array: 4 axes * 4 registers = 16 (D0-D15)
  std::vector<uint16_t> regs(NUM_COMMAND_REGS);

  for (int i = 0; i < NUM_AXES; i++) {
    int offset = i * 4;

    // command_1: speed_limit (uint32) or target_position (int32 as uint32)
    regs[offset + 0] = pkt_.axes[i].command_1 & 0xFFFF;
    regs[offset + 1] = (pkt_.axes[i].command_1 >> 16) & 0xFFFF;

    // command_2: target_torque (int32)
    uint32_t torque_raw = static_cast<uint32_t>(pkt_.axes[i].command_2);
    regs[offset + 2] = torque_raw & 0xFFFF;
    regs[offset + 3] = (torque_raw >> 16) & 0xFFFF;
  }

  int rc = modbus_write_registers(ctx_, D_REGISTERS_START, NUM_COMMAND_REGS, regs.data());
  if (rc != NUM_COMMAND_REGS) {
    RCLCPP_ERROR(logger, "Failed to write D0-D%d: %s",
      NUM_COMMAND_REGS - 1, modbus_strerror(errno));
    return false;
  }

  // Throttled diagnostic logging (Compy pattern: every 2 seconds)
  auto now = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_log_time_).count();
  if (elapsed_ms >= LOG_INTERVAL_MS) {
    logAxisStatus();
    last_log_time_ = now;
  }

  return true;
}

bool RaptorLiftModbusDriver::readFeedback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!ctx_ || !connected_.load()) {
    return false;
  }

  auto logger = rclcpp::get_logger("RaptorLiftModbusDriver");

  // Read feedback registers: D16-D23 (8 registers, 2 per axis)
  uint16_t regs[NUM_FEEDBACK_REGS];
  int rc = modbus_read_registers(ctx_, FEEDBACK_REG_START, NUM_FEEDBACK_REGS, regs);
  if (rc != NUM_FEEDBACK_REGS) {
    RCLCPP_ERROR(logger, "Failed to read D%d-D%d: %s",
      FEEDBACK_REG_START,
      FEEDBACK_REG_START + NUM_FEEDBACK_REGS - 1,
      modbus_strerror(errno));
    return false;
  }

  // Parse feedback: 2 registers per axis
  for (int i = 0; i < NUM_AXES; i++) {
    int offset = i * 2;
    pkt_.axes[i].feedback = static_cast<int32_t>(
      (regs[offset + 1] << 16) | regs[offset]);
  }

  return true;
}

// ==================== FEEDBACK INTERFACE ====================

double RaptorLiftModbusDriver::getTractionVelocity(int axis_index) const
{
  if (axis_index < 0 || axis_index >= NUM_AXES) {
    return 0.0;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  // Compensate motor direction on readback
  return rawToVelocity(pkt_.axes[axis_index].feedback) * motor_dirs[axis_index];
}

double RaptorLiftModbusDriver::getSteeringPosition(int axis_index) const
{
  if (axis_index < 0 || axis_index >= NUM_AXES) {
    return 0.0;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  // Compensate motor direction on readback
  return rawToPosition(pkt_.axes[axis_index].feedback) * motor_dirs[axis_index];
}

int32_t RaptorLiftModbusDriver::getRawFeedback(int axis_index) const
{
  if (axis_index < 0 || axis_index >= NUM_AXES) {
    return 0;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return pkt_.axes[axis_index].feedback;
}

// ==================== SAFETY ====================

bool RaptorLiftModbusDriver::emergencyStop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto logger = rclcpp::get_logger("RaptorLiftModbusDriver");

  // Compy pattern: speed_limit=0, torque=BRAKE_TORQUE for traction; zero for steering
  for (int i = 0; i < NUM_AXES; i++) {
    pkt_.axes[i].command_1 = 0;
    if (axis_types[i] == AxisType::TRACTION) {
      pkt_.axes[i].command_2 = torqueToRaw(BRAKE_TORQUE * motor_dirs[i]);
    } else {
      pkt_.axes[i].command_2 = 0;
    }
  }

  if (connected_.load() && ctx_) {
    bool success = sendPacketUnlocked();
    if (success) {
      // Disable motor drives after braking
      setMotorEnableUnlocked(false);
      RCLCPP_WARN(logger, "Emergency stop: brake torque=%.1f, motors disabled", BRAKE_TORQUE);
    }
    return success;
  }
  return true;
}

bool RaptorLiftModbusDriver::setMotorEnable(bool enable)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return setMotorEnableUnlocked(enable);
}

void RaptorLiftModbusDriver::logAxisStatus() const
{
  auto logger = rclcpp::get_logger("RaptorLiftModbusDriver");

  RCLCPP_INFO(logger,
    "[PLC] FR(cmd1=%u trq=%d fb=%d) FL(cmd1=%u trq=%d fb=%d) "
    "RR(cmd1=%u trq=%d fb=%d) RL(cmd1=%u trq=%d fb=%d)",
    pkt_.axes[0].command_1, pkt_.axes[0].command_2, pkt_.axes[0].feedback,
    pkt_.axes[1].command_1, pkt_.axes[1].command_2, pkt_.axes[1].feedback,
    pkt_.axes[2].command_1, pkt_.axes[2].command_2, pkt_.axes[2].feedback,
    pkt_.axes[3].command_1, pkt_.axes[3].command_2, pkt_.axes[3].feedback);
}

// ==================== PRIVATE ====================

void RaptorLiftModbusDriver::sendPLCResetPulse()
{
  if (!ctx_) {
    return;
  }

  auto logger = rclcpp::get_logger("RaptorLiftModbusDriver");

  // Compy pattern: M0 high -> 10ms -> M0 low
  int rc = modbus_write_bit(ctx_, M_PLC_RESET, 1);
  if (rc == -1) {
    RCLCPP_WARN(logger, "Failed to toggle PLC reset (M0 high): %s", modbus_strerror(errno));
    return;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (modbus_write_bit(ctx_, M_PLC_RESET, 0) == -1) {
    RCLCPP_WARN(logger, "Failed to toggle PLC reset (M0 low): %s", modbus_strerror(errno));
    return;
  }

  RCLCPP_INFO(logger, "PLC reset pulse sent (M0: 1 -> 0)");
}

bool RaptorLiftModbusDriver::setMotorEnableUnlocked(bool enable)
{
  if (!ctx_) {
    return false;
  }

  auto logger = rclcpp::get_logger("RaptorLiftModbusDriver");
  bool success = true;

  // Per-axis motor enable: M1-M4 (Compy pattern)
  for (int i = 0; i < NUM_AXES; i++) {
    int coil_addr = M_MOTOR_BASE + i;  // M1, M2, M3, M4
    int rc = modbus_write_bit(ctx_, coil_addr, enable ? 1 : 0);
    if (rc == -1) {
      RCLCPP_WARN(logger, "Failed to %s motor M%d: %s",
        enable ? "enable" : "disable", i + 1, modbus_strerror(errno));
      success = false;
    }
  }

  if (success) {
    RCLCPP_INFO(logger, "Motors M1-M4 %s", enable ? "enabled" : "disabled");
  }

  return success;
}

// ==================== CONVERSION HELPERS ====================

int32_t RaptorLiftModbusDriver::positionToRaw(double position) const
{
  return static_cast<int32_t>(std::llround(position * POSITION_SCALE));
}

double RaptorLiftModbusDriver::rawToPosition(int32_t raw) const
{
  return static_cast<double>(raw) / POSITION_SCALE;
}

int32_t RaptorLiftModbusDriver::velocityToRaw(double velocity) const
{
  return static_cast<int32_t>(std::llround(velocity * VELOCITY_SCALE));
}

double RaptorLiftModbusDriver::rawToVelocity(int32_t raw) const
{
  return static_cast<double>(raw) / VELOCITY_SCALE;
}

int32_t RaptorLiftModbusDriver::torqueToRaw(double torque) const
{
  return static_cast<int32_t>(std::llround(torque * TORQUE_SCALE));
}

}  // namespace raptorlift_hardware_bridge
