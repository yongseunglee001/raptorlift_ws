# raptorlift_hardware_bridge

**Standalone ROS 2 node + Modbus driver library** for the RaptorLift rear-steer Ackermann forklift.

## Overview

This package provides two things:

1. **HardwareBridge node** — bridges ROS topics to hardware (Modbus PLC or simulated PLC), used with `TopicBasedSystem` for Isaac Sim integration.
2. **RaptorLiftModbusDriver** — a shared library encapsulating Modbus TCP communication, reused by both this node and the `raptorlift_hardware` ros2_control plugin.

```
ros2_control (TopicBasedSystem)
  │               ^
  │ joint_commands │ joint_states
  v               │
┌──────────────────────────────┐
│     HardwareBridge Node      │
│                              │
│  PID (per joint)             │
│  ┌──────────┐ ┌──────────┐  │
│  │ Steering │ │ Traction │  │
│  │ Position │ │ Velocity │  │
│  │ Control  │ │ Control  │  │
│  └────┬─────┘ └────┬─────┘  │
│       └──────┬──────┘        │
│       ┌──────┴──────┐        │
│       │ Mode Switch │        │
│       └──┬──────┬───┘        │
│   ┌──────┘      └──────┐    │
│   v                    v    │
│ VirtualPLC      ModbusDriver │
│ Simulator       (libmodbus)  │
└──────────────────────────────┘
                    │
               Modbus TCP
                    │
          Mitsubishi PLC / Servos
```

## Relationship to raptorlift_hardware

These two packages serve different deployment scenarios:

| | **raptorlift_hardware** | **raptorlift_hardware_bridge** (this) |
|---|---|---|
| **Type** | ros2_control plugin (SystemInterface) | Standalone node + Modbus library |
| **Data path** | `controller_manager → plugin → PLC` | `TopicBasedSystem → ROS topics → node → PLC` |
| **When to use** | Direct hardware deployment (no Isaac Sim) | Isaac Sim, debugging, topic-based workflows |
| **Modbus driver** | Imports from this package | Provides the driver as a shared library |
| **Control rate** | Driven by controller_manager update_rate | Own 100 Hz wall timer |
| **Safety** | Lifecycle-based (on_error, on_deactivate) | Watchdog + e-stop + auto-reconnect |

**Current bringup** uses `TopicBasedSystem` + this bridge node for Isaac Sim compatibility.

## Operating Modes

| Mode | Condition | Backend |
|---|---|---|
| **Simulation** | `simulation_mode: true` or libmodbus not found | `VirtualPLCSimulator` (first-order dynamics) |
| **Real Hardware** | `simulation_mode: false` + libmodbus built | `RaptorLiftModbusDriver` (Modbus TCP to PLC) |

Falls back to simulation automatically if real hardware connection fails.

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/raptorlift/joint_commands` | `sensor_msgs/JointState` | Subscribe | Commands from TopicBasedSystem |
| `/raptorlift/joint_states` | `sensor_msgs/JointState` | Publish | Feedback to TopicBasedSystem |
| `/gear_state` | `std_msgs/Int32` | Subscribe | Gear state from gamepad (logged only) |

## Configuration

All parameters in `config/hardware_bridge.yaml`:

```yaml
simulation_mode: true          # true=VirtualPLC, false=Modbus TCP
control_rate: 100.0            # Hz
detailed_logging: false

topics:
  joint_commands: "/raptorlift/joint_commands"
  joint_states: "/raptorlift/joint_states"

steering_joints: [rear_left_steering_joint, rear_right_steering_joint]
traction_joints: [front_left_wheel_joint, front_right_wheel_joint]

# Wheel geometry
wheel_radius: 0.1715           # m - for m/s <-> rad/s PLC conversion

# PID tuning
steering_kp: 50.0
steering_ki: 0.0
steering_kd: 5.0
max_steering_torque: 100.0     # Nm
traction_kp: 10.0
traction_ki: 5.0
traction_kd: 0.0
max_traction_torque: 150.0     # Nm

# Modbus connection
modbus_ip: "192.168.2.1"
modbus_port: 502

# Safety
comm_timeout: 1.0              # seconds before emergency stop
reconnect_interval: 5.0        # seconds between reconnect attempts
```

## Modbus Register Map

### Commands (Write: D0-D15, 4 registers per axis)

| Axis | Registers | Field 1 | Field 2 |
|---|---|---|---|
| FR (0) | D0-D3 | speed_limit (uint32) | target_torque (int32) |
| FL (1) | D4-D7 | speed_limit (uint32) | target_torque (int32) |
| RR (2) | D8-D11 | target_position (int32) | target_torque (int32) |
| RL (3) | D12-D15 | target_position (int32) | target_torque (int32) |

### Feedback (Read: D16-D23, 2 registers per axis)

| Axis | Registers | Field |
|---|---|---|
| FR (0) | D16-D17 | rotation_speed (int32) |
| FL (1) | D18-D19 | rotation_speed (int32) |
| RR (2) | D20-D21 | actual_position (int32) |
| RL (3) | D22-D23 | actual_position (int32) |

### Scale Factors

| Value | Conversion |
|---|---|
| Position | rad x 10000 |
| Velocity | rad/s x 10000 |
| Torque | Nm x 10 (0.1% rated torque) |
| Speed limit | \|vel\| x 100000 |

> **Note:** Joint-level traction velocity is in **m/s** (wheel surface speed). The bridge converts to **rad/s** at the PLC boundary using `wheel_radius`.

### PLC Coils

| Coil | Function |
|---|---|
| M0 | PLC reset (pulse on connection) |
| M1-M4 | Motor enable (FR, FL, RR, RL) |

## Safety Features

- **Communication watchdog**: e-stop if Modbus feedback delayed > `comm_timeout`
- **Emergency stop**: zeros all commands via `modbus_driver_->emergencyStop()`
- **Auto-reconnect**: retries PLC connection every `reconnect_interval` seconds
- **Graceful fallback**: uses VirtualPLC if real hardware unavailable

## Build

```bash
# Simulation only (no libmodbus required)
colcon build --packages-select raptorlift_hardware_bridge

# With real hardware support
sudo apt install libmodbus-dev
colcon build --packages-select raptorlift_hardware_bridge
```

## Launch

```bash
# Simulation mode (default)
ros2 launch raptorlift_hardware_bridge hardware_bridge.launch.py

# Real hardware
ros2 launch raptorlift_hardware_bridge hardware_bridge.launch.py \
  simulation_mode:=false modbus_ip:=192.168.2.1
```

## Axis Convention

```
Axis 0 = FR (Front Right)  - Traction  - motor_dir: +1
Axis 1 = FL (Front Left)   - Traction  - motor_dir: -1
Axis 2 = RR (Rear Right)   - Steering  - motor_dir: +1
Axis 3 = RL (Rear Left)    - Steering  - motor_dir: -1
```

## File Structure

```
raptorlift_hardware_bridge/
├── config/hardware_bridge.yaml       # All parameters
├── launch/hardware_bridge.launch.py
├── include/raptorlift_hardware_bridge/
│   ├── hardware_bridge.hpp           # Node, PID, VirtualPLCSimulator
│   └── modbus_driver.hpp             # Modbus TCP driver (shared library)
└── src/
    ├── hardware_bridge.cpp           # Main node + control loop
    └── modbus_driver.cpp             # Modbus register read/write
```

## Dependencies

| Package | Required | Purpose |
|---|---|---|
| `rclcpp` | Yes | ROS 2 C++ client |
| `sensor_msgs` | Yes | JointState messages |
| `std_msgs` | Yes | Int32 for gear state |
| `libmodbus-dev` | Optional | Real hardware Modbus TCP |

## License

Apache-2.0
