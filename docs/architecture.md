# RaptorLift ros2_control 통합 아키텍처

## 시스템 개요

RaptorLift는 **Topic-Based ros2_control** 아키텍처를 사용하여 시뮬레이션과 실제 하드웨어를 동일한 코드베이스로 제어합니다.

## 아키텍처 다이어그램

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          ROS 2 Control Stack                            │
│                    (시뮬레이션/실제 하드웨어 공통)                         │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   Xbox Controller → joy_node → gamepad_teleop                          │
│                                      ↓                                  │
│                              /cmd_vel (Twist)                           │
│                                      ↓                                  │
│                    ackermann_steering_controller                        │
│                   (ros2_controllers 표준 패키지)                         │
│                                      ↓                                  │
│                    TopicBasedSystem (hardware_interface)                │
│                   (topic_based_ros2_control)                            │
│                                      ↓                                  │
│               /raptorlift/joint_commands (sensor_msgs/JointState)       │
│                          (position, velocity)                           │
│                                                                         │
└────────────────────────────┬────────────────────────────────────────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ▼                    ▼                    ▼
┌───────────────┐   ┌───────────────┐   ┌───────────────────────┐
│  Isaac Sim    │   │ Hardware      │   │ 기타 시뮬레이터       │
│  (OmniGraph)  │   │ Bridge        │   │ (Gazebo, etc.)        │
│               │   │               │   │                       │
│  Subscribe:   │   │ Subscribe:    │   │ Subscribe:            │
│  joint_cmds   │   │ joint_cmds    │   │ joint_cmds            │
│               │   │      ↓        │   │                       │
│  Articulation │   │ PID Control   │   │                       │
│  Controller   │   │ pos→torque    │   │                       │
│               │   │ vel→torque    │   │                       │
│               │   │      ↓        │   │                       │
│               │   │ Virtual PLC   │   │                       │
│               │   │ or Modbus TCP │   │                       │
│               │   │               │   │                       │
│  Publish:     │   │ Publish:      │   │ Publish:              │
│  joint_states │   │ joint_states  │   │ joint_states          │
└───────────────┘   └───────────────┘   └───────────────────────┘
        │                    │                    │
        └────────────────────┼────────────────────┘
                             │
                             ▼
               /raptorlift/joint_states (sensor_msgs/JointState)
                          (position, velocity, effort)
```

## Hardware Bridge 상세 구조

```
┌─────────────────────────────────────────────────────────────┐
│                    Hardware Bridge Node                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  /raptorlift/joint_commands ──┐                             │
│         (position, velocity)   │                             │
│                                ▼                             │
│  ┌─────────────────────────────────────────────────┐        │
│  │              PID Controller Layer               │        │
│  │  ┌─────────────────┬─────────────────┐         │        │
│  │  │ Steering PID    │ Traction PID    │         │        │
│  │  │ pos_err→torque  │ vel_err→torque  │         │        │
│  │  │ Kp=50, Kd=5     │ Kp=10, Ki=5     │         │        │
│  │  └────────┬────────┴────────┬────────┘         │        │
│  └───────────┼─────────────────┼──────────────────┘        │
│              │                 │                             │
│   ┌──────────┴─────────────────┴────────────┐               │
│   │          Backend Selection               │               │
│   │  ┌─────────────┐   ┌─────────────────┐  │               │
│   │  │ Virtual PLC │   │ Modbus TCP      │  │               │
│   │  │ (sim mode)  │   │ (real hardware) │  │               │
│   │  │             │   │                 │  │               │
│   │  │ Motor       │   │ D0-D15: cmd     │  │               │
│   │  │ Dynamics    │   │ D16-D31: fb     │  │               │
│   │  │ Simulation  │   │ M1: enable      │  │               │
│   │  └─────────────┘   └─────────────────┘  │               │
│   └─────────────────────────────────────────┘               │
│              │                                               │
│              ▼                                               │
│  /raptorlift/joint_states ────────────────────────────────  │
│         (position, velocity, effort)                         │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Modbus TCP 레지스터 맵 (실제 하드웨어)

### Command Registers (Write, D0-D15) — 축 순서: FR→FL→RR→RL

| Register | Size | Axis | Joint | Description |
|----------|------|------|-------|-------------|
| D0-D1 | uint32 | 0 (FR) | Front Right Wheel | Speed Limit |
| D2-D3 | int32 | 0 (FR) | Front Right Wheel | Torque Command (0.1% rated) |
| D4-D5 | uint32 | 1 (FL) | Front Left Wheel | Speed Limit |
| D6-D7 | int32 | 1 (FL) | Front Left Wheel | Torque Command |
| D8-D9 | int32 | 2 (RR) | Rear Right Steering | Target Position (rad × 10000) |
| D10-D11 | int32 | 2 (RR) | Rear Right Steering | Torque Command |
| D12-D13 | int32 | 3 (RL) | Rear Left Steering | Target Position |
| D14-D15 | int32 | 3 (RL) | Rear Left Steering | Torque Command |

### Feedback Registers (Read, D16-D23) — 축 순서: FR→FL→RR→RL

| Register | Size | Axis | Joint | Description |
|----------|------|------|-------|-------------|
| D16-D17 | int32 | 0 (FR) | Front Right Wheel | Rotation Speed (rad/s × 10000) |
| D18-D19 | int32 | 1 (FL) | Front Left Wheel | Rotation Speed |
| D20-D21 | int32 | 2 (RR) | Rear Right Steering | Actual Position (rad × 10000) |
| D22-D23 | int32 | 3 (RL) | Rear Left Steering | Actual Position |

### Coils (M Registers)

| Coil | Description |
|------|-------------|
| M0 | PLC Reset (pulse: high → 10ms → low) |
| M1 | FR Motor Enable |
| M2 | FL Motor Enable |
| M3 | RR Motor Enable |
| M4 | RL Motor Enable |

## 토픽 구조

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/raptorlift/joint_commands` | sensor_msgs/JointState | ros2_control → 백엔드 | 조인트 명령 (pos/vel) |
| `/raptorlift/joint_states` | sensor_msgs/JointState | 백엔드 → ros2_control | 조인트 상태 (pos/vel/effort) |
| `/cmd_vel` | geometry_msgs/TwistStamped | teleop → controller | 속도 명령 |
| `/joy` | sensor_msgs/Joy | joy_node → teleop | Xbox 컨트롤러 입력 |

## 조인트 구성

| Axis | Joint Name | Type | Command | 제어 방식 |
|------|------------|------|---------|-----------|
| 0 (FR) | `front_right_wheel_joint` | continuous | velocity | PI → Torque |
| 1 (FL) | `front_left_wheel_joint` | continuous | velocity | PI → Torque |
| 2 (RR) | `rear_right_steering_joint` | revolute | position | PD → Torque |
| 3 (RL) | `rear_left_steering_joint` | revolute | position | PD → Torque |

## 실행 모드

### 1. 시뮬레이션 모드 (Virtual PLC)
```bash
ros2 launch raptorlift_bringup raptorlift.launch.py
# hardware_bridge가 Virtual PLC로 모터 동역학 시뮬레이션
```

### 2. Isaac Sim 모드
```bash
# Isaac Sim에서 URDF import 후 setup_isaac_sim.py 실행
ros2 launch raptorlift_bringup raptorlift.launch.py use_hardware_bridge:=false use_sim_time:=true
```

### 3. 실제 하드웨어 모드 (Modbus TCP)
```bash
ros2 launch raptorlift_bringup raptorlift.launch.py simulation_mode:=false modbus_ip:=192.168.2.1
# hardware_bridge가 Mitsubishi 서보와 Modbus TCP로 통신
# 주의: libmodbus-dev 패키지 설치 필요 (sudo apt-get install libmodbus-dev)
```

### 4. Xbox 텔레옵 (모든 모드에서 동일)
```bash
ros2 launch raptorlift_bringup teleop.launch.py
```

## PID 파라미터

### Steering (Position → Torque, PD)
- Kp: 50.0
- Ki: 0.0
- Kd: 5.0
- Max Torque: 100.0 Nm

### Traction (Velocity → Torque, PI)
- Kp: 10.0
- Ki: 5.0
- Kd: 0.0
- Max Torque: 150.0 Nm

## 패키지 구조

```
jazzy_ws/src/
├── raptorlift_description/     # URDF, RViz, Isaac Sim 스크립트
│   ├── urdf/raptorlift.urdf   # ros2_control 태그 포함
│   ├── scripts/setup_isaac_sim.py
│   └── ...
├── raptorlift_teleop/          # Xbox 게임패드 텔레옵
├── raptorlift_hardware_bridge/ # 하드웨어 브릿지 (PID + Virtual PLC/Modbus)
│   ├── include/
│   │   └── raptorlift_hardware_bridge/
│   │       ├── hardware_bridge.hpp   # Main node + VirtualPLCSimulator
│   │       └── modbus_driver.hpp     # Modbus TCP driver (optional)
│   └── src/
│       ├── hardware_bridge.cpp
│       └── modbus_driver.cpp         # (libmodbus required)
├── raptorlift_bringup/         # 런치 파일
│   ├── launch/raptorlift.launch.py
│   ├── launch/teleop.launch.py
│   └── config/raptorlift_controllers.yaml
└── moveit/topic_based_ros2_control/  # Topic 기반 하드웨어 인터페이스
```

## 빌드 옵션

### Simulation Only (libmodbus 없이)
```bash
colcon build --packages-select raptorlift_hardware_bridge
# CMake will warn but build successfully for simulation mode
```

### With Real Hardware Support
```bash
sudo apt-get install libmodbus-dev
colcon build --packages-select raptorlift_hardware_bridge
# HAS_MODBUS defined, full functionality available
```

## Isaac Sim 5.1 설정

1. Isaac Sim 실행
2. File > Import > URDF로 `raptorlift.urdf` 가져오기
3. Window > Script Editor에서 `setup_isaac_sim.py` 실행
4. Play 버튼으로 시뮬레이션 시작
5. 별도 터미널에서 `ros2 launch raptorlift_bringup teleop.launch.py use_hardware_bridge:=false use_sim_time:=true`

---

*Last updated: 2026-02-05*
