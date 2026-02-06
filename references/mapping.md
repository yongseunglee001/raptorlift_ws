
// 4-wheel Ackermann Steering Forklift (FR, FL, RR, RL)
// FR,RR = same direction (+1), FL,RL = same direction (-1), FL-FR opposite
static const int motor_dirs[4] = {+1, -1, +1, -1}; // Motor mounting sign

// Axis types
// FR, FL = traction (velocity -> torque)
// RR, RL = steering (position -> torque)


### Wheel Indices
- **Index 0**: FR (Front Right)  - Traction
- **Index 1**: FL (Front Left)   - Traction
- **Index 2**: RR (Rear Right)   - Steering
- **Index 3**: RL (Rear Left)    - Steering

### Physical Dimensions
- **Wheel radius**: 0.1715 m
- **Wheelbase**: 1.0 m (front to rear)
- **Track width**: 0.71 m (left to right)


Register  Role                    Type
M0        Error reset             BOOL
M1        Axis 1 (FR) Move        BOOL
M2        Axis 2 (FL) Move        BOOL
M3        Axis 3 (RR) Move        BOOL
M4        Axis 4 (RL) Move        BOOL

D0        Axis 1 (FR) Speed limit       uint32 (> 0)
D2        Axis 1 (FR) Target Torque     int32
D4        Axis 2 (FL) Speed limit       uint32 (> 0)
D6        Axis 2 (FL) Target Torque     int32
D8        Axis 3 (RR) Target Position   int32 (rad * 10000)
D10       Axis 3 (RR) Target Torque     int32
D12       Axis 4 (RL) Target Position   int32 (rad * 10000)
D14       Axis 4 (RL) Target Torque     int32
D16       Axis 1 (FR) Rotation speed    int32 (feedback, read-only)
D18       Axis 2 (FL) Rotation speed    int32 (feedback, read-only)
D20       Axis 3 (RR) Actual Position   int32 (feedback, read-only)
D22       Axis 4 (RL) Actual Position   int32 (feedback, read-only)


## Control Parameters

```cpp
// Scale factors
constexpr double POSITION_SCALE = 10000.0;  // rad to int32
constexpr double VELOCITY_SCALE = 10000.0;  // rad/s to int32
constexpr double TORQUE_SCALE   = 10.0;     // Nm to 0.1% rated
constexpr double BRAKE_TORQUE   = 300.0;    // Emergency stop brake torque
```

### Gear System
- **Gear range**: 1-20
- **Velocity scaling**: `velocity = min_vel + (max_vel - min_vel) * (gear - 1) / (num_gears - 1)`

### Encoder (reference)
Number of Pulses per Rotation = 4194304 Pulses
Movement Amount per Rotation 65973.4 um (micrometer)
Encoder Resolution = 4194304 Pulses/rev
