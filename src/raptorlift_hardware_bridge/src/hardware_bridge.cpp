// Copyright 2026 Kandins
// SPDX-License-Identifier: Apache-2.0

#include "raptorlift_hardware_bridge/hardware_bridge.hpp"

namespace raptorlift_hardware_bridge
{

HardwareBridge::HardwareBridge(const rclcpp::NodeOptions & options)
: Node("hardware_bridge", options)
{
  // Declare parameters
  this->declare_parameter("simulation_mode", true);
  this->declare_parameter("control_rate", 100.0);
  this->declare_parameter("detailed_logging", false);

  // Topic parameters
  this->declare_parameter("topics.joint_commands", "/raptorlift/joint_commands");
  this->declare_parameter("topics.joint_states", "/raptorlift/joint_states");

  // Joint names: traction [FR, FL], steering [RR, RL]
  this->declare_parameter("traction_joints",
    std::vector<std::string>{"front_right_wheel_joint", "front_left_wheel_joint"});
  this->declare_parameter("steering_joints",
    std::vector<std::string>{"rear_right_steering_joint", "rear_left_steering_joint"});

  // Modbus parameters
  this->declare_parameter("modbus_ip", "192.168.2.1");
  this->declare_parameter("modbus_port", 502);

  // PID: steering (position control)
  this->declare_parameter("steering_kp", 50.0);
  this->declare_parameter("steering_ki", 0.0);
  this->declare_parameter("steering_kd", 5.0);
  this->declare_parameter("max_steering_torque", 100.0);

  // PID: traction (velocity control)
  this->declare_parameter("traction_kp", 10.0);
  this->declare_parameter("traction_ki", 5.0);
  this->declare_parameter("traction_kd", 0.0);
  this->declare_parameter("max_traction_torque", 150.0);

  // Communication watchdog (Compy pattern)
  this->declare_parameter("comm_timeout", 1.0);
  this->declare_parameter("reconnect_interval", 5.0);

  // Get parameters
  simulation_mode_ = this->get_parameter("simulation_mode").as_bool();
  control_rate_ = this->get_parameter("control_rate").as_double();
  detailed_logging_ = this->get_parameter("detailed_logging").as_bool();

  topic_joint_commands_ = this->get_parameter("topics.joint_commands").as_string();
  topic_joint_states_ = this->get_parameter("topics.joint_states").as_string();

  traction_joint_names_ = this->get_parameter("traction_joints").as_string_array();
  steering_joint_names_ = this->get_parameter("steering_joints").as_string_array();

  modbus_ip_ = this->get_parameter("modbus_ip").as_string();
  modbus_port_ = this->get_parameter("modbus_port").as_int();

  steering_kp_ = this->get_parameter("steering_kp").as_double();
  steering_ki_ = this->get_parameter("steering_ki").as_double();
  steering_kd_ = this->get_parameter("steering_kd").as_double();
  max_steering_torque_ = this->get_parameter("max_steering_torque").as_double();

  traction_kp_ = this->get_parameter("traction_kp").as_double();
  traction_ki_ = this->get_parameter("traction_ki").as_double();
  traction_kd_ = this->get_parameter("traction_kd").as_double();
  max_traction_torque_ = this->get_parameter("max_traction_torque").as_double();

  comm_timeout_ = this->get_parameter("comm_timeout").as_double();
  reconnect_interval_ = this->get_parameter("reconnect_interval").as_double();

  // Initialize timing
  last_comm_time_ = std::chrono::steady_clock::now();
  last_reconnect_attempt_ = std::chrono::steady_clock::now();

  // Initialize joints in axis order: FR(0), FL(1), RR(2), RL(3)
  initialize_joints();

  // Initialize communication
  if (simulation_mode_) {
    virtual_plc_ = std::make_unique<VirtualPLCSimulator>();
    RCLCPP_INFO(this->get_logger(), "Virtual PLC simulator initialized");
  } else {
#ifdef HAS_MODBUS
    if (!initialize_modbus()) {
      RCLCPP_WARN(this->get_logger(),
        "Failed to connect to Modbus - falling back to simulation mode");
      simulation_mode_ = true;
      virtual_plc_ = std::make_unique<VirtualPLCSimulator>();
    }
#else
    RCLCPP_WARN(this->get_logger(),
      "Built without libmodbus support - using simulation mode");
    simulation_mode_ = true;
    virtual_plc_ = std::make_unique<VirtualPLCSimulator>();
#endif
  }

  // Create subscriber/publisher (RELIABLE QoS for TopicBasedSystem)
  auto reliable_qos = rclcpp::QoS(10).reliable().durability_volatile();
  joint_commands_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    topic_joint_commands_, reliable_qos,
    std::bind(&HardwareBridge::joint_commands_callback, this, std::placeholders::_1));

  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    topic_joint_states_, reliable_qos);

  // Gear state subscriber (from gamepad_teleop)
  gear_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/gear_state", 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      current_gear_ = msg->data;
    });

  // Control loop timer
  auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&HardwareBridge::control_loop, this));

  last_control_time_ = this->now();

  // Startup log
  RCLCPP_INFO(this->get_logger(), "RaptorLift Hardware Bridge initialized");
  RCLCPP_INFO(this->get_logger(), "  Mode: %s",
    simulation_mode_ ? "SIMULATION (Virtual PLC)" : "REAL HARDWARE");
  RCLCPP_INFO(this->get_logger(), "  Control rate: %.1f Hz", control_rate_);
  RCLCPP_INFO(this->get_logger(), "  Axis order: FR(0)=%s, FL(1)=%s, RR(2)=%s, RL(3)=%s",
    joints_[0].name.c_str(), joints_[1].name.c_str(),
    joints_[2].name.c_str(), joints_[3].name.c_str());
  RCLCPP_INFO(this->get_logger(), "  Topics: cmd=%s  state=%s",
    topic_joint_commands_.c_str(), topic_joint_states_.c_str());
  if (!simulation_mode_) {
    RCLCPP_INFO(this->get_logger(), "  Modbus: %s:%d  timeout=%.1fs  reconnect=%.1fs",
      modbus_ip_.c_str(), modbus_port_, comm_timeout_, reconnect_interval_);
  }
  RCLCPP_INFO(this->get_logger(), "  Traction PID: Kp=%.1f Ki=%.1f Kd=%.1f (max=%.1f)",
    traction_kp_, traction_ki_, traction_kd_, max_traction_torque_);
  RCLCPP_INFO(this->get_logger(), "  Steering PID: Kp=%.1f Ki=%.1f Kd=%.1f (max=%.1f)",
    steering_kp_, steering_ki_, steering_kd_, max_steering_torque_);
}

void HardwareBridge::initialize_joints()
{
  joints_.clear();
  joints_.resize(4);

  // Traction joints: FR(axis 0), FL(axis 1)
  for (size_t i = 0; i < traction_joint_names_.size() && i < 2; i++) {
    joints_[i].name = traction_joint_names_[i];
    joints_[i].axis_index = static_cast<int>(i);  // 0=FR, 1=FL
    joints_[i].is_traction = true;
    joints_[i].pid.setGains(traction_kp_, traction_ki_, traction_kd_);
    joints_[i].pid.setLimits(-max_traction_torque_, max_traction_torque_);
  }

  // Steering joints: RR(axis 2), RL(axis 3)
  for (size_t i = 0; i < steering_joint_names_.size() && i < 2; i++) {
    int idx = static_cast<int>(i) + 2;
    joints_[idx].name = steering_joint_names_[i];
    joints_[idx].axis_index = idx;  // 2=RR, 3=RL
    joints_[idx].is_traction = false;
    joints_[idx].pid.setGains(steering_kp_, steering_ki_, steering_kd_);
    joints_[idx].pid.setLimits(-max_steering_torque_, max_steering_torque_);
  }

  RCLCPP_INFO(this->get_logger(), "Initialized 4 axes: 2 traction (FR,FL) + 2 steering (RR,RL)");
}

#ifdef HAS_MODBUS
bool HardwareBridge::initialize_modbus()
{
  modbus_driver_ = std::make_unique<RaptorLiftModbusDriver>();

  if (!modbus_driver_->connect(modbus_ip_, modbus_port_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to PLC at %s:%d",
      modbus_ip_.c_str(), modbus_port_);
    return false;
  }

  modbus_connected_ = true;
  last_comm_time_ = std::chrono::steady_clock::now();
  emergency_stop_active_ = false;
  reconnect_count_ = 0;

  RCLCPP_INFO(this->get_logger(), "Connected to PLC (reset pulse + M1-M4 enabled)");
  return true;
}
#else
bool HardwareBridge::initialize_modbus()
{
  RCLCPP_ERROR(this->get_logger(), "Modbus not compiled - install libmodbus-dev");
  return false;
}
#endif

void HardwareBridge::joint_commands_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); i++) {
    for (auto & joint : joints_) {
      if (joint.name == msg->name[i]) {
        if (joint.is_traction) {
          // Velocity command for traction
          if (!msg->velocity.empty() && i < msg->velocity.size() &&
              !std::isnan(msg->velocity[i])) {
            joint.cmd_velocity = msg->velocity[i];
          }
        } else {
          // Position command for steering
          if (!msg->position.empty() && i < msg->position.size() &&
              !std::isnan(msg->position[i])) {
            joint.cmd_position = msg->position[i];
          }
        }
        break;
      }
    }
  }
}

void HardwareBridge::control_loop()
{
  auto current_time = this->now();
  double dt = (current_time - last_control_time_).seconds();
  last_control_time_ = current_time;

  if (dt <= 0.0 || dt > 1.0) {
    dt = 1.0 / control_rate_;
  }

  // Communication timeout check (Compy pattern - real hardware only)
  if (!simulation_mode_) {
    checkCommunicationTimeout();
  }

  // Emergency stop handling
  if (emergency_stop_active_) {
    handleEmergencyStop();
    publish_joint_states();
    return;
  }

  // Compute PID torque for each axis (derivative on measurement to avoid kick)
  for (auto & joint : joints_) {
    double torque = 0.0;

    if (joint.is_traction) {
      double velocity_error = joint.cmd_velocity - joint.state_velocity;
      torque = joint.pid.compute(velocity_error, joint.state_velocity, dt);
    } else {
      double position_error = joint.cmd_position - joint.state_position;
      torque = joint.pid.compute(position_error, joint.state_position, dt);
    }

    joint.state_effort = torque;
  }

  // Compute PLC register equivalents (for logging in all modes)
  for (auto & joint : joints_) {
    int dir = MOTOR_DIRS[joint.axis_index];
    joint.plc_torque_raw = static_cast<int32_t>(
      std::llround(joint.state_effort * dir * PLC_TORQUE_SCALE));
    if (joint.is_traction) {
      joint.plc_speed_limit = static_cast<uint32_t>(
        std::abs(joint.cmd_velocity) * PLC_SPEED_LIMIT_SCALE);
    } else {
      joint.plc_pos_raw = static_cast<int32_t>(
        std::llround(joint.cmd_position * dir * PLC_POSITION_SCALE));
    }
  }

  // Send commands and read feedback
  if (simulation_mode_) {
    // Set torques to VirtualPLC
    for (const auto & joint : joints_) {
      if (joint.is_traction) {
        virtual_plc_->setTractionTorque(joint.axis_index, joint.state_effort);
      } else {
        virtual_plc_->setSteeringTorque(joint.axis_index, joint.state_effort);
      }
    }

    virtual_plc_->update(dt);

    // Read back simulated feedback
    for (auto & joint : joints_) {
      joint.state_position = virtual_plc_->getPosition(joint.axis_index);
      joint.state_velocity = virtual_plc_->getVelocity(joint.axis_index);
    }

    if (detailed_logging_) {
      logControlPipeline(dt);
    }
  } else {
    // Real hardware via Modbus
    if (!write_to_hardware()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Failed to write to hardware");
    }

    if (!read_from_hardware()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Failed to read from hardware");
    } else {
      last_comm_time_ = std::chrono::steady_clock::now();
    }

    if (detailed_logging_) {
      logControlPipeline(dt);
    }
  }

  publish_joint_states();
}

void HardwareBridge::checkCommunicationTimeout()
{
  if (simulation_mode_) {
    return;
  }

  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
    now - last_comm_time_).count();

  if (elapsed > comm_timeout_) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Communication timeout (%.2fs > %.2fs)", elapsed, comm_timeout_);
    emergency_stop_active_ = true;
    modbus_connected_ = false;
  }
}

void HardwareBridge::handleEmergencyStop()
{
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Emergency stop active - brake torque applied");

  for (auto & joint : joints_) {
    joint.cmd_velocity = 0.0;
    joint.state_effort = 0.0;
    joint.pid.reset();
  }

#ifdef HAS_MODBUS
  if (modbus_driver_ && modbus_driver_->isConnected()) {
    modbus_driver_->emergencyStop();
  }
#endif

  if (!modbus_connected_ && !simulation_mode_) {
    if (attemptReconnect()) {
      emergency_stop_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Reconnected - emergency stop cleared");
    }
  }
}

bool HardwareBridge::attemptReconnect()
{
#ifdef HAS_MODBUS
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
    now - last_reconnect_attempt_).count();

  if (elapsed < reconnect_interval_) {
    return false;
  }

  last_reconnect_attempt_ = now;
  reconnect_count_++;

  RCLCPP_INFO(this->get_logger(), "Reconnect attempt #%d...", reconnect_count_);

  if (modbus_driver_) {
    modbus_driver_.reset();
  }

  modbus_driver_ = std::make_unique<RaptorLiftModbusDriver>();
  if (modbus_driver_->connect(modbus_ip_, modbus_port_)) {
    modbus_connected_ = true;
    last_comm_time_ = std::chrono::steady_clock::now();
    reconnect_count_ = 0;
    return true;
  }

  RCLCPP_WARN(this->get_logger(), "Reconnect #%d failed", reconnect_count_);
  return false;
#else
  return false;
#endif
}

#ifdef HAS_MODBUS
bool HardwareBridge::write_to_hardware()
{
  if (!modbus_driver_ || !modbus_driver_->isConnected()) {
    return false;
  }

  for (const auto & joint : joints_) {
    if (joint.is_traction) {
      // velocity -> speed_limit + torque
      double speed = std::abs(joint.cmd_velocity);
      uint32_t speed_limit = static_cast<uint32_t>(speed * 100000.0);
      modbus_driver_->setTractionCommand(joint.axis_index, speed_limit, joint.state_effort);
    } else {
      // position + torque
      modbus_driver_->setSteeringCommand(joint.axis_index, joint.cmd_position, joint.state_effort);
    }
  }

  return modbus_driver_->sendPacket();
}
#else
bool HardwareBridge::write_to_hardware()
{
  return false;
}
#endif

#ifdef HAS_MODBUS
bool HardwareBridge::read_from_hardware()
{
  if (!modbus_driver_ || !modbus_driver_->isConnected()) {
    return false;
  }

  if (!modbus_driver_->readFeedback()) {
    return false;
  }

  double dt = (this->now() - last_control_time_).seconds();
  if (dt <= 0.0) {
    dt = 1.0 / control_rate_;
  }

  for (auto & joint : joints_) {
    if (joint.is_traction) {
      joint.state_velocity = modbus_driver_->getTractionVelocity(joint.axis_index);
      // Integrate velocity to estimate position (PLC doesn't provide traction position)
      joint.state_position += joint.state_velocity * dt;
    } else {
      double prev_position = joint.state_position;
      joint.state_position = modbus_driver_->getSteeringPosition(joint.axis_index);
      // Compute velocity from position difference (PLC doesn't provide steering velocity)
      if (dt > 0.0) {
        joint.state_velocity = (joint.state_position - prev_position) / dt;
      }
    }
  }

  return true;
}
#else
bool HardwareBridge::read_from_hardware()
{
  return false;
}
#endif

void HardwareBridge::logControlPipeline(double dt)
{
  // Throttled full-pipeline log: gear → cmd → PID → PLC regs → state
  // Shows every 500ms (2 Hz) for readability
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    "\n"
    "──── Control Pipeline (dt=%.4f s) ── gear=%d ────\n"
    "[CMD]   FR vel=%+8.3f rad/s   FL vel=%+8.3f rad/s\n"
    "        RR pos=%+8.4f rad     RL pos=%+8.4f rad\n"
    "[PID]   FR trq=%+8.2f Nm     FL trq=%+8.2f Nm\n"
    "        RR trq=%+8.2f Nm     RL trq=%+8.2f Nm\n"
    "[PLC]   FR spd_lim=%8u  trq_raw=%+7d\n"
    "        FL spd_lim=%8u  trq_raw=%+7d\n"
    "        RR pos_raw=%+8d trq_raw=%+7d\n"
    "        RL pos_raw=%+8d trq_raw=%+7d\n"
    "        (spd_lim: |vel|x1e5  pos_raw: rad x1e4  trq_raw: Nm x dir x10 [x0.1%%rated])\n"
    "[STATE] FR vel=%+8.3f rad/s  pos=%+8.3f rad\n"
    "        FL vel=%+8.3f rad/s  pos=%+8.3f rad\n"
    "        RR vel=%+8.4f rad/s  pos=%+8.4f rad\n"
    "        RL vel=%+8.4f rad/s  pos=%+8.4f rad\n"
    "─────────────────────────────────────────────────",
    dt, current_gear_,
    // CMD
    joints_[0].cmd_velocity, joints_[1].cmd_velocity,
    joints_[2].cmd_position, joints_[3].cmd_position,
    // PID
    joints_[0].state_effort, joints_[1].state_effort,
    joints_[2].state_effort, joints_[3].state_effort,
    // PLC registers
    joints_[0].plc_speed_limit, joints_[0].plc_torque_raw,
    joints_[1].plc_speed_limit, joints_[1].plc_torque_raw,
    joints_[2].plc_pos_raw, joints_[2].plc_torque_raw,
    joints_[3].plc_pos_raw, joints_[3].plc_torque_raw,
    // STATE (encoder feedback)
    joints_[0].state_velocity, joints_[0].state_position,
    joints_[1].state_velocity, joints_[1].state_position,
    joints_[2].state_velocity, joints_[2].state_position,
    joints_[3].state_velocity, joints_[3].state_position);
}

void HardwareBridge::publish_joint_states()
{
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = this->now();

  for (const auto & joint : joints_) {
    msg.name.push_back(joint.name);
    msg.position.push_back(joint.state_position);
    msg.velocity.push_back(joint.state_velocity);
    msg.effort.push_back(joint.state_effort);
  }

  joint_states_pub_->publish(msg);
}

}  // namespace raptorlift_hardware_bridge

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<raptorlift_hardware_bridge::HardwareBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
