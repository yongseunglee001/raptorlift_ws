# RaptorLift ros2_control Implementation

**Version:** 0.1.0
**Date:** 2026-02-05
**ROS2 Distribution:** Jazzy

---

## 1. 시스템 개요

RaptorLift는 후륜 조향(Rear Wheel Steering) Ackermann 구조의 지게차 로봇입니다.

### 1.1 기구적 전제조건

| 위치 | 바퀴 | 모터 | 제어 방식 |
|------|------|------|----------|
| 앞바퀴 (Front) | 2개 | 구동 모터 | Velocity command (rad/s) |
| 뒷바퀴 (Rear) | 2개 | 조향 모터 | Position command (rad) |

**핵심 포인트:**
- 앞바퀴: 구동만 담당 (전진/후진)
- 뒷바퀴: 조향만 담당 (방향 전환), 바퀴 회전은 모터 없이 굴러감
- 뒷바퀴의 wheel joint는 `fixed` 타입 (ros2_control 제어 대상 아님)

### 1.2 좌표계 (ROS 표준)

```
        +X (Forward, Fork side)
         ↑
         │
    +Y ←─┼─→ -Y
         │
         ↓
        -X (Rear, Counterweight side)
```

---

## 2. 조인트 구성

### 2.1 ros2_control 제어 대상

| Joint Name | Type | Command Interface | State Interface | 용도 |
|------------|------|-------------------|-----------------|------|
| `front_left_wheel_joint` | continuous | velocity | position, velocity, effort | 좌측 구동 |
| `front_right_wheel_joint` | continuous | velocity | position, velocity, effort | 우측 구동 |
| `rear_left_steering_joint` | revolute | position, velocity | position, velocity, effort | 좌측 조향 |
| `rear_right_steering_joint` | revolute | position, velocity | position, velocity, effort | 우측 조향 |

**참고:** 조향 조인트에 velocity command interface가 있는 이유는 `ackermann_steering_controller` 호환성 때문

### 2.2 ros2_control 제어 대상 아님

| Joint Name | Type | 설명 |
|------------|------|------|
| `rear_left_wheel_joint` | fixed | 뒷바퀴 회전 (모터 없음) |
| `rear_right_wheel_joint` | fixed | 뒷바퀴 회전 (모터 없음) |

---

## 3. 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│  Xbox Controller → joy_node → gamepad_teleop                │
│  └─→ /ackermann_steering_controller/reference (TwistStamped)│
└─────────────────────┬───────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────────────┐
│  ackermann_steering_controller (ros2_controllers)           │
│  ┌─────────────────┬─────────────────┐                      │
│  │ Steering: pos   │ Traction: vel   │                      │
│  │ [rad]           │ [rad/s]         │                      │
│  └────────┬────────┴────────┬────────┘                      │
└───────────┼─────────────────┼───────────────────────────────┘
            ↓                 ↓
┌─────────────────────────────────────────────────────────────┐
│  RaptorLiftHardware (raptorlift_hardware)                   │
│  ┌─────────────────┬─────────────────┐                      │
│  │ Position PID    │ Velocity PID    │                      │
│  │ pos → torque    │ vel → torque    │                      │
│  └────────┬────────┴────────┬────────┘                      │
└───────────┼─────────────────┼───────────────────────────────┘
            ↓                 ↓
┌─────────────────────────────────────────────────────────────┐
│  Mitsubishi Servos (토크 모드)                               │
│   Steering(Rear)           Traction(Front)                  │
│   RL_steer  RR_steer       FL_wheel  FR_wheel               │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. 패키지 구조

### 4.1 raptorlift_hardware

ros2_control 하드웨어 인터페이스

```
raptorlift_hardware/
├── include/raptorlift_hardware/
│   ├── raptorlift_hardware.hpp   # SystemInterface 클래스
│   ├── visibility_control.h      # Export 매크로
│   └── pid_controller.hpp        # PID 제어기 (header-only)
├── src/
│   └── raptorlift_hardware.cpp   # 하드웨어 인터페이스 구현
├── config/
│   └── hardware_params.yaml      # PID 게인 참조용
├── raptorlift_hardware.xml       # pluginlib 등록
├── CMakeLists.txt
└── package.xml
```

### 4.2 raptorlift_bringup

런치 파일 및 컨트롤러 설정

```
raptorlift_bringup/
├── launch/
│   ├── raptorlift.launch.py      # 기본 런치 (RSP + CM + Controllers)
│   └── teleop.launch.py          # 텔레옵 포함 런치
├── config/
│   └── raptorlift_controllers.yaml  # 컨트롤러 설정
├── CMakeLists.txt
└── package.xml
```

### 4.3 raptorlift_description (수정됨)

URDF에 ros2_control 태그 추가

```
raptorlift_description/
└── urdf/
    └── raptorlift.urdf           # ros2_control 블록 포함
```

---

## 5. 주요 설정값

### 5.1 하드웨어 파라미터 (URDF)

```xml
<param name="simulation_mode">true</param>
<param name="steering_kp">50.0</param>
<param name="steering_ki">0.0</param>
<param name="steering_kd">5.0</param>
<param name="traction_kp">10.0</param>
<param name="traction_ki">5.0</param>
<param name="traction_kd">0.0</param>
<param name="max_steering_torque">100.0</param>
<param name="max_traction_torque">150.0</param>
```

### 5.2 컨트롤러 파라미터 (YAML)

```yaml
ackermann_steering_controller:
  ros__parameters:
    traction_joints_names:
      - front_left_wheel_joint
      - front_right_wheel_joint
    steering_joints_names:
      - rear_left_steering_joint
      - rear_right_steering_joint
    wheelbase: 1.0                    # m
    traction_wheels_radius: 0.1715    # m
    traction_track_width: 0.71        # m
    steering_track_width: 0.71        # m
```

### 5.3 로봇 물리 파라미터

| 파라미터 | 값 | 단위 |
|----------|-----|------|
| Wheelbase | 1.0 | m |
| Track Width | 0.71 | m |
| Wheel Radius | 0.1715 | m |
| Steering Limit | ±0.7 | rad (±40°) |
| Max Linear Vel | 2.0 | m/s |
| Max Angular Vel | 1.0 | rad/s |

---

## 6. 실행 방법

```bash
# 1. 빌드
cd ~/RaptorYamato/IsaacSim-ros_workspaces/jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select raptorlift_hardware raptorlift_bringup raptorlift_description

# 2. 시뮬레이션 모드 실행 (텔레옵 없이)
source install/setup.bash
ros2 launch raptorlift_bringup raptorlift.launch.py simulation_mode:=true

# 3. 텔레옵 포함 실행 (Xbox 컨트롤러 연결 필요)
ros2 launch raptorlift_bringup teleop.launch.py simulation_mode:=true

# 4. 확인 명령어
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

---

## 7. 변경 이력

### v0.1.0 (2026-02-05)

**신규 패키지:**
- `raptorlift_hardware`: ros2_control SystemInterface 구현
- `raptorlift_bringup`: 런치 파일 및 컨트롤러 설정

**수정 파일:**
- `raptorlift_description/urdf/raptorlift.urdf`: ros2_control 태그 추가

**주요 결정사항:**
1. 조향 조인트에 position + velocity command interface 추가 (ackermann_steering_controller 호환)
2. 뒷바퀴 회전 조인트(`rear_*_wheel_joint`)를 `fixed` 타입으로 변경 (모터 없음)
3. 시뮬레이션 모드에서 PID 제어 + 간단한 동역학 시뮬레이션
4. 실제 하드웨어 통신은 TODO (Modbus/EtherCAT)

**의존성:**
- `ackermann_steering_controller` (ros2_controllers)
- `joint_state_broadcaster` (ros2_controllers)
- `controller_manager` (ros2_control)

---

## 8. 게임패드 버튼 매핑 (Xbox Series X Controller)

| Button Index | 버튼 | 기능 |
|--------------|------|------|
| 0 | A | 리프트 다운 |
| 1 | B | 리프트 정지 |
| 2 | X | 리프트 업 |
| 3 | Y | 리프트 홈 |
| 6 | L1 | 기어 다운 |
| 7 | R1 | 기어 업 |
| 10 | MODE (두 사각형) | 텔레옵/자율주행 모드 토글 |
| 11 | ESTOP (햄버거) | 비상정지/시작 토글 |

| Axis Index | 축 | 기능 |
|------------|-----|------|
| 0 | Left Stick X | (미사용) |
| 1 | Left Stick Y | 전진/후진 |
| 2 | LT | 감속 |
| 3 | Right Stick X | 회전 |
| 5 | RT | 터보 |
| 6, 7 | D-pad | 미세 조작 |

---

## 9. TODO

- [ ] 실제 하드웨어 통신 구현 (Mitsubishi 서보 - Modbus/EtherCAT)
- [ ] PID 게인 튜닝 (실제 하드웨어 기반)
- [ ] 안전 기능 추가 (비상 정지, 리밋 스위치)
- [ ] 진단 기능 추가 (서보 상태 모니터링)

---

*Last updated: 2026-02-05*
