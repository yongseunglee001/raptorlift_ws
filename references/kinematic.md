# RaptorLift 4-Wheel Ackermann Steering Forklift Kinematics

## Overview

RaptorLift is a 4-wheel Ackermann steering forklift. Front wheels (FR, FL) provide traction via velocity-torque control, rear wheels (RR, RL) provide steering via position-torque control.

## Wheel Configuration

### Physical Layout

```
    +X (forward, fork side)
     ^
     |
  FL ●───────────● FR       <- Traction (velocity -> torque)
     |           |
     | base_link |
     |           |
  RL ●───────────● RL       <- Steering (position -> torque)
     |
     +Y (left)
```

### Axis Indices & Motor Directions

```cpp
// 4-wheel Ackermann 부호 (FR, FL, RR, RL)
static constexpr int motor_dirs[4] = {+1, -1, +1, -1};
// FR and RR: same direction (+1)
// FL and RL: same direction (-1)
// FL is opposite to FR
```

| Index | Wheel | Joint Name                   | Type     | Control       | Motor Dir |
|-------|-------|------------------------------|----------|---------------|-----------|
| 0     | FR    | front_right_wheel_joint      | Traction | velocity->torque | +1     |
| 1     | FL    | front_left_wheel_joint       | Traction | velocity->torque | -1     |
| 2     | RR    | rear_right_steering_joint    | Steering | position->torque | +1     |
| 3     | RL    | rear_left_steering_joint     | Steering | position->torque | -1     |

### Physical Dimensions

| Parameter    | Value     |
|-------------|-----------|
| Wheel radius | 0.1715 m |
| Wheelbase    | 1.0 m    |
| Track width  | 0.71 m   |

## PLC Register Map

### Command Registers (D0-D15, write)

Each axis uses 4 registers (2 x uint16 for each 32-bit value):

```
Axis 0 (FR): D0,D1 = speed_limit (uint32), D2,D3 = target_torque (int32)
Axis 1 (FL): D4,D5 = speed_limit (uint32), D6,D7 = target_torque (int32)
Axis 2 (RR): D8,D9 = target_position (int32), D10,D11 = target_torque (int32)
Axis 3 (RL): D12,D13 = target_position (int32), D14,D15 = target_torque (int32)
```

### Feedback Registers (D16-D23, read-only)

```
D16,D17 = FR rotation_speed (int32, rad/s * 10000)
D18,D19 = FL rotation_speed (int32, rad/s * 10000)
D20,D21 = RR actual_position (int32, rad * 10000)
D22,D23 = RL actual_position (int32, rad * 10000)
```

### Motor Enable Coils

```
M0 = PLC error reset (pulse: high -> 10ms -> low)
M1 = FR motor enable
M2 = FL motor enable
M3 = RR motor enable
M4 = RL motor enable
```

## Control Architecture

### Traction Control (FR, FL)

```
cmd_velocity (from ros2_control)
     |
     v
  PID Controller (velocity error -> torque)
     |
     v
  Motor Direction Compensation (* motor_dirs[i])
     |
     v
  PLC: speed_limit = |cmd_vel| * SCALE, target_torque = PID output
     |
     v
  Feedback: rotation_speed -> state_velocity
```

### Steering Control (RR, RL)

```
cmd_position (from ros2_control)
     |
     v
  PID Controller (position error -> torque)
     |
     v
  Motor Direction Compensation (* motor_dirs[i])
     |
     v
  PLC: target_position = cmd_pos * SCALE, target_torque = PID output
     |
     v
  Feedback: actual_position -> state_position
```

## Ackermann Steering Geometry

For rear-steer Ackermann, the inner and outer wheel angles follow:

```
cot(outer_angle) = cot(inner_angle) + track_width / wheelbase
```

The `ackermann_steering_controller` in ros2_control handles this kinematic conversion, outputting individual joint commands for each wheel.

## Data Conversion

| Conversion         | Formula                  | Scale Factor |
|--------------------|--------------------------|--------------|
| rad -> raw         | raw = rad * 10000        | POSITION_SCALE = 10000.0 |
| rad/s -> raw       | raw = rad_s * 10000      | VELOCITY_SCALE = 10000.0 |
| Nm -> raw          | raw = Nm * 10.0          | TORQUE_SCALE = 10.0 |
| Emergency brake    | torque = 300.0 (raw)     | BRAKE_TORQUE = 300.0 |

## PLC Communication Sequence

1. **Connect**: Create Modbus TCP connection
2. **PLC Reset**: M0 pulse (high -> 10ms -> low)
3. **Motor Enable**: M1-M4 = 1 (per-axis)
4. **Control Loop**:
   - Write D0-D15 (speed_limit/position + torque per axis)
   - Read D16-D23 (velocity/position feedback per axis)
   - Throttled logging every 2 seconds
5. **Emergency Stop**: speed_limit=0, torque=BRAKE_TORQUE
6. **Disconnect**: Zero all -> disable M1-M4 -> close

## Safety Features

- Communication timeout watchdog (default 1.0s)
- Emergency stop with active brake torque (300.0)
- Per-motor enable/disable on connect/disconnect
- PLC reset pulse on initial connection
- Auto-reconnect on connection loss (rate-limited)
- Simulation mode with VirtualPLC when no hardware

*Last updated: 2026-02-06*
