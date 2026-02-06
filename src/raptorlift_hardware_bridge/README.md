# raptorlift_hardware_bridge

Hardware bridge for the RaptorLift 4-wheel Ackermann robot.
Converts ROS 2 `sensor_msgs/JointState` commands into Mitsubishi servo torque
commands via Modbus TCP, with a built-in Virtual PLC simulator for
development and testing without real hardware.

## Architecture

```
                ros2_control (TopicBasedSystem)
                  |               ^
     joint_commands (sub)    joint_states (pub)
                  v               |
        ┌─────────────────────────────────┐
        │      HardwareBridge Node        │
        │                                 │
        │   PIDController (per joint)     │
        │   ┌─────────┐  ┌─────────┐     │
        │   │Steering  │  │Traction │     │
        │   │Position  │  │Velocity │     │
        │   │Control   │  │Control  │     │
        │   └────┬─────┘  └────┬────┘     │
        │        └──────┬──────┘          │
        │               │                │
        │        ┌──────┴──────┐          │
        │        │ Mode Switch │          │
        │        └──┬──────┬───┘          │
        │   ┌───────┘      └───────┐      │
        │   v                      v      │
        │ VirtualPLC       ModbusDriver   │
        │ Simulator        (libmodbus)    │
        │ (always)         (optional)     │
        └─────────────────────────────────┘
                               │
                          Modbus TCP
                               │
                     Mitsubishi PLC / Servos
```

### Dual-Mode Operation

| Mode | Condition | Backend |
|------|-----------|---------|
| **Simulation** | `simulation_mode: true` or libmodbus not installed | `VirtualPLCSimulator` - first-order dynamics model |
| **Real Hardware** | `simulation_mode: false` and libmodbus built | `RaptorLiftModbusDriver` - Modbus TCP to PLC |

If real hardware mode fails to connect, the node automatically falls back to
simulation.

### Joint Control

| Joint Type | Control Method | Joints (default) |
|------------|---------------|-------------------|
| Steering | Position -> PID -> Torque | `rear_left_steering_joint`, `rear_right_steering_joint` |
| Traction | Velocity -> PID -> Torque | `front_left_wheel_joint`, `front_right_wheel_joint` |

## Dependencies

| Package | Required | Purpose |
|---------|----------|---------|
| `rclcpp` | Yes | ROS 2 C++ client |
| `sensor_msgs` | Yes | `JointState` message |
| `std_msgs` | Yes | Standard messages |
| `geometry_msgs` | Yes | Geometry messages |
| `libmodbus-dev` | **Optional** | Real hardware Modbus TCP |

## Build

```bash
# Simulation-only (no libmodbus required)
cd ~/jazzy_ws
colcon build --packages-select raptorlift_hardware_bridge

# With real hardware support
sudo apt-get install libmodbus-dev
colcon build --packages-select raptorlift_hardware_bridge
```

## Configuration

All parameters are defined in `config/hardware_bridge.yaml`.
Edit this file instead of modifying C++ source.

### Key config sections

```yaml
# Operating mode
simulation_mode: true

# ROS topic names
topics:
  joint_commands: "/raptorlift/joint_commands"
  joint_states:   "/raptorlift/joint_states"

# Joint names (must match your URDF)
steering_joints:
  - "rear_left_steering_joint"
  - "rear_right_steering_joint"
traction_joints:
  - "front_left_wheel_joint"
  - "front_right_wheel_joint"

# PID tuning
steering_kp: 50.0
traction_kp: 10.0

# Modbus connection
modbus_ip: "192.168.2.1"
modbus_port: 502
```

See `config/hardware_bridge.yaml` for the full parameter reference.

## Usage

### Launch file (recommended)

```bash
# Default (simulation mode, default config)
ros2 launch raptorlift_hardware_bridge hardware_bridge.launch.py

# Real hardware
ros2 launch raptorlift_hardware_bridge hardware_bridge.launch.py \
  simulation_mode:=false \
  modbus_ip:=192.168.2.1

# Custom config file
ros2 launch raptorlift_hardware_bridge hardware_bridge.launch.py \
  config_file:=/path/to/custom_config.yaml

# With verbose logging
ros2 launch raptorlift_hardware_bridge hardware_bridge.launch.py \
  detailed_logging:=true
```

### Direct run

```bash
# Simulation mode (defaults)
ros2 run raptorlift_hardware_bridge hardware_bridge

# With inline parameter overrides
ros2 run raptorlift_hardware_bridge hardware_bridge --ros-args \
  -p simulation_mode:=false \
  -p modbus_ip:="192.168.1.100" \
  -p control_rate:=50.0

# With YAML config
ros2 run raptorlift_hardware_bridge hardware_bridge --ros-args \
  --params-file config/hardware_bridge.yaml
```

## Arguments / Parameters

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `config_file` | `<pkg>/config/hardware_bridge.yaml` | YAML parameter file |
| `simulation_mode` | `true` | Simulation (`true`) or real hardware (`false`) |
| `control_rate` | `100.0` | Control loop frequency (Hz) |
| `modbus_ip` | `192.168.2.1` | PLC Modbus TCP address |
| `modbus_port` | `502` | PLC Modbus TCP port |
| `detailed_logging` | `false` | Verbose runtime logging |

### ROS Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `simulation_mode` | bool | `true` | Enable Virtual PLC simulator |
| `control_rate` | double | `100.0` | Control loop Hz |
| `detailed_logging` | bool | `false` | Verbose logging |
| `topics.joint_commands` | string | `/raptorlift/joint_commands` | Command input topic |
| `topics.joint_states` | string | `/raptorlift/joint_states` | State output topic |
| `steering_joints` | string[] | `[rear_left_steering_joint, rear_right_steering_joint]` | Steering joint names |
| `traction_joints` | string[] | `[front_left_wheel_joint, front_right_wheel_joint]` | Traction joint names |
| `steering_kp` | double | `50.0` | Steering P gain |
| `steering_ki` | double | `0.0` | Steering I gain |
| `steering_kd` | double | `5.0` | Steering D gain |
| `max_steering_torque` | double | `100.0` | Max steering torque (Nm) |
| `traction_kp` | double | `10.0` | Traction P gain |
| `traction_ki` | double | `5.0` | Traction I gain |
| `traction_kd` | double | `0.0` | Traction D gain |
| `max_traction_torque` | double | `150.0` | Max traction torque (Nm) |
| `modbus_ip` | string | `192.168.2.1` | PLC IP address |
| `modbus_port` | int | `502` | Modbus TCP port |

## Topics

| Topic | Type | Direction | QoS | Description |
|-------|------|-----------|-----|-------------|
| `/raptorlift/joint_commands` | `sensor_msgs/JointState` | Subscribe | Reliable (depth 10) | Position/velocity commands from ros2_control |
| `/raptorlift/joint_states` | `sensor_msgs/JointState` | Publish | Reliable (depth 10) | Position/velocity/effort feedback to ros2_control |

Topic names are configurable via the `topics.*` parameters.

## Modbus Register Map

### Command Registers (Write: D0-D15)

| Register | Data | Type | Unit |
|----------|------|------|------|
| D0-D1 | Rear Left Steering Position | int32 | rad * 10000 |
| D2-D3 | Rear Left Steering Torque | int32 | 0.1% rated |
| D4-D5 | Rear Right Steering Position | int32 | rad * 10000 |
| D6-D7 | Rear Right Steering Torque | int32 | 0.1% rated |
| D8-D9 | Front Left Wheel Speed Limit | uint32 | PLC units |
| D10-D11 | Front Left Wheel Torque | int32 | 0.1% rated |
| D12-D13 | Front Right Wheel Speed Limit | uint32 | PLC units |
| D14-D15 | Front Right Wheel Torque | int32 | 0.1% rated |

### Feedback Registers (Read: D16-D31)

| Register | Data | Type | Unit |
|----------|------|------|------|
| D16-D17 | Rear Left Steering Position | int32 | rad * 10000 |
| D18-D19 | Rear Left Steering Velocity | int32 | rad/s * 10000 |
| D20-D21 | Rear Right Steering Position | int32 | rad * 10000 |
| D22-D23 | Rear Right Steering Velocity | int32 | rad/s * 10000 |
| D24-D25 | Front Left Wheel Velocity | int32 | rad/s * 10000 |
| D26-D27 | Front Left Wheel Position | int32 | rad * 10000 |
| D28-D29 | Front Right Wheel Velocity | int32 | rad/s * 10000 |
| D30-D31 | Front Right Wheel Position | int32 | rad * 10000 |

### Motor Control Coils

| Address | Description |
|---------|-------------|
| M8193 | Motor enable (1 = enabled) |

## File Structure

```
raptorlift_hardware_bridge/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── hardware_bridge.yaml        # All parameters (edit this, not C++)
├── launch/
│   └── hardware_bridge.launch.py   # Launch with config + arg overrides
├── include/raptorlift_hardware_bridge/
│   ├── hardware_bridge.hpp         # Node, PIDController, VirtualPLCSimulator
│   └── modbus_driver.hpp           # Modbus TCP driver (optional build)
└── src/
    ├── hardware_bridge.cpp         # Main node implementation
    └── modbus_driver.cpp           # Modbus register read/write
```

## License

Apache-2.0
