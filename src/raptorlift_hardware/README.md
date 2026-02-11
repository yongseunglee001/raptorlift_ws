# raptorlift_hardware

**ros2_control SystemInterface plugin** for the RaptorLift rear-steer Ackermann forklift.

## Overview

This package provides a `hardware_interface::SystemInterface` plugin that sits inside the ros2_control `controller_manager`. It exposes joint command/state interfaces so standard ros2_control controllers (e.g., `ackermann_steering_controller`) can drive the robot.

```
controller_manager
  └─ ackermann_steering_controller
       └─ raptorlift_hardware (this plugin)
            ├─ Simulation mode: built-in torque→dynamics model
            └─ Real hardware mode: Modbus TCP via RaptorLiftModbusDriver
```

## Operating Modes

### Simulation Mode (`simulation_mode: true`)

Built-in physics simulation — no external simulator or hardware needed.

- Torque-driven dynamics: `accel = (torque - damping * vel) / inertia`
- Steering: position-limited to +/-0.7 rad, velocity zeroed at limit
- Traction: continuous joint, position wraps at +/-pi
- PID controllers compute torque from position/velocity commands
- Traction velocity in m/s (surface speed); converted to rad/s at PLC boundary using `wheel_radius`

### Real Hardware Mode (`simulation_mode: false`)

Communicates with Mitsubishi servo motors via Modbus TCP.

- Uses `RaptorLiftModbusDriver` from the `raptorlift_hardware_bridge` package
- Requires `libmodbus` at build time (conditionally compiled via `HAS_MODBUS`)
- Connects to PLC on `on_activate()`, disconnects + e-stop on `on_deactivate()`

## Interfaces

### Command Interfaces

| Joint Type | Interface | Description |
|---|---|---|
| Steering (RR, RL) | `position` | Target steering angle (rad) |
| Steering (RR, RL) | `velocity` | Unused (ackermann controller compatibility) |
| Traction (FR, FL) | `velocity` | Target wheel speed (m/s) |

### State Interfaces

| Joint Type | Interface | Description |
|---|---|---|
| Steering | `position` | Encoder position or simulated (rad) |
| Traction | `position` | Distance traveled (m) |
| Steering | `velocity` | Computed velocity (rad/s) |
| Traction | `velocity` | Wheel surface speed (m/s) |
| All | `effort` | PID torque output (Nm) |

## Configuration (URDF `<ros2_control>` parameters)

```xml
<param name="simulation_mode">true</param>
<param name="modbus_ip">192.168.2.1</param>
<param name="modbus_port">502</param>

<!-- PID Tuning -->
<param name="steering_kp">50.0</param>
<param name="steering_ki">0.0</param>
<param name="steering_kd">5.0</param>
<param name="traction_kp">10.0</param>
<param name="traction_ki">5.0</param>
<param name="traction_kd">0.0</param>

<!-- Torque/Dynamics -->
<param name="max_steering_torque">100.0</param>
<param name="max_traction_torque">150.0</param>
<param name="steering_inertia">0.5</param>
<param name="traction_inertia">0.08</param>
<param name="steering_damping">0.5</param>
<param name="traction_damping">1.0</param>
<param name="max_steering_angle">0.7</param>
<param name="wheel_radius">0.1715</param>
```

## Lifecycle

| Callback | Action |
|---|---|
| `on_init()` | Parse URDF params, identify steering/traction joints, init PID |
| `on_configure()` | Reset joint states, prepare PID controllers |
| `on_activate()` | Connect to PLC (real) or init simulation |
| `on_deactivate()` | Emergency stop + disconnect (real) |
| `on_error()` | Emergency cleanup |
| `read()` | Read feedback from PLC or simulate dynamics |
| `write()` | Compute PID torque, send to PLC or store for simulation |

## Axis Convention

```
Axis 0 = FR (Front Right)  - Traction
Axis 1 = FL (Front Left)   - Traction
Axis 2 = RR (Rear Right)   - Steering
Axis 3 = RL (Rear Left)    - Steering
```

## Dependencies

- `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`
- `raptorlift_hardware_bridge` (for `RaptorLiftModbusDriver` in real hardware mode)
- `libmodbus` (optional, auto-detected at build time)

## Relationship to raptorlift_hardware_bridge

This plugin provides a **direct** path from ros2_control to hardware:

```
ros2_control → raptorlift_hardware → Modbus PLC
```

The alternative path uses `TopicBasedSystem` + the bridge node:

```
ros2_control → TopicBasedSystem → ROS topics → raptorlift_hardware_bridge → Modbus PLC
```

The direct path has lower latency and tighter integration. The bridge path is useful for Isaac Sim (which requires TopicBasedSystem) and debugging.
