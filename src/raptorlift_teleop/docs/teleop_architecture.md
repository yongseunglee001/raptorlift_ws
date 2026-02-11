# RaptorLift Teleop Architecture

## Overview

RaptorLift 로봇의 통합 제어 시스템. twist_mux를 통한 다중 입력 소스 지원과 20단 기어 기반 속도 스케일링을 제공합니다.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           raptorlift.launch.py                                 │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                │
│  ┌─────────────┐     ┌───────────┐     ┌──────────────────────────────────────┐│
│  │ gamepad     │────▶│           │     │                                    ││
│  │ /teleop/    │     │           │     │ ackermann_steering_controller      ││
│  │ cmd_vel     │     │ twist_mux │────▶│ /ackermann_steering_controller/    ││
│  │ (gear-scaled│     │ (우선순위) │     │  reference                         ││
│  │  real m/s)  │     │           │     │                                    ││
│  └─────────────┘     │           │     └──────────────────────────────────────┘│
│  ┌─────────────┐     │           │                                            │
│  │ keyboard    │────▶│           │                                            │
│  │ /key/cmd_vel│     │           │                                            │
│  │ (real m/s)  │     │           │                                            │
│  └─────────────┘     │           │                                            │
│  ┌─────────────┐     │           │                                            │
│  │ nav2        │────▶│           │                                            │
│  │ /nav/cmd_vel│     └───────────┘                                            │
│  │ (real m/s)  │                                                              │
│  └─────────────┘                                                              │
│                                                                                │
└─────────────────────────────────────────────────────────────────────────────────┘
```

All sources publish **real velocity in m/s** (not normalized). Gear scaling is
applied only inside `gamepad_teleop` so that autonomous (Nav2) commands pass
through twist_mux unmodified.

## Launch Files

| Launch File | Contents | Use Case |
|-------------|----------|----------|
| `raptorlift.launch.py` | ros2_control + twist_mux + teleop + LiDAR + webcam | 수동 조종 (기본) |
| `slam.launch.py` | raptorlift.launch.py + Nav2 SLAM stack | 자율주행 |

## Quick Start

```bash
# 시뮬레이션 모드 (기본)
ros2 launch raptorlift_bringup raptorlift.launch.py

# 실제 하드웨어
ros2 launch raptorlift_bringup raptorlift.launch.py simulation_mode:=false

# Isaac Sim 연동 (hardware_bridge 없이)
ros2 launch raptorlift_bringup raptorlift.launch.py use_hardware_bridge:=false use_lidars:=false

# 키보드 조종 추가 (별도 터미널)
# stamped:=true 필수 — twist_mux가 TwistStamped만 수신 (use_stamped: true)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/key/cmd_vel -p stamped:=true -p frame_id:=base_link

# Teleop 없이 실행 (Nav2 전용)
ros2 launch raptorlift_bringup raptorlift.launch.py use_teleop:=false
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `simulation_mode` | `true` | Hardware bridge 시뮬레이션 모드 (true=echo, false=real servo) |
| `use_hardware_bridge` | `true` | Hardware bridge 노드 실행 여부 (Isaac Sim 직접 연동 시 false) |
| `use_rviz` | `true` | RViz2 실행 여부 |
| `use_teleop` | `true` | Teleop 노드 실행 여부 (joy_node + gamepad_teleop) |
| `use_sim_time` | `false` | 시뮬레이션 시간 사용 (Isaac Sim 연동 시 true) |
| `use_lidars` | `true` | LiDAR 드라이버 실행 (Hesai XT32 + RSE1 Front) |
| `use_webcam` | `true` | USB 웹캠 드라이버 실행 (Logitech C922) |
| `control_rate` | `100.0` | 제어 루프 주기 (Hz) |
| `verbose` | `false` | 상세 제어 파이프라인 로깅 |

## Nodes

### 1. gamepad_teleop

Xbox 컨트롤러 입력을 기어 스케일링된 실제 속도(m/s)로 변환합니다.

**Subscriptions:**
| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | sensor_msgs/Joy | 게임패드 raw 입력 |

**Publications:**
| Topic | Type | Description |
|-------|------|-------------|
| `/teleop/cmd_vel` | geometry_msgs/TwistStamped | 기어 스케일링된 속도 (m/s) |
| `/gear_state` | std_msgs/Int32 | 현재 기어 (1-20) |
| `/lift_command` | std_msgs/Int8 | 리프트 명령 (-1, 0, 1, 2) |
| `/teleop_state` | std_msgs/String | 상태 정보 문자열 |

**Gear-Velocity Mapping (20단):**
```
Gear  1:  0.05 m/s  (최저속)
Gear  5:  0.26 m/s
Gear 10:  0.53 m/s  (기본)
Gear 15:  0.79 m/s
Gear 20:  1.00 m/s  (최고속 — 컨트롤러 한계)
```

**공식:** `velocity = min_vel + (max_vel - min_vel) * (gear - 1) / (num_gears - 1)`

**Xbox Controller Mapping:**
| Input | Function |
|-------|----------|
| **Left Stick Y** | 전진/후진 (기어 스케일링) |
| **Right Stick X** | 좌/우 회전 (기어 스케일링) |
| **L1 (LB)** | 기어 다운 |
| **R1 (RB)** | 기어 업 |
| **LT (Left Trigger)** | 감속 (최대 70% 감소) |
| **RT (Right Trigger)** | 가속/터보 (최대 150% 증가) |
| **D-Pad** | 정밀 이동 (절대 속도: 0.1 m/s, 기어 무관) |
| **A** | 리프트 하강 (hold) |
| **X** | 리프트 상승 (hold) |
| **Y** | 리프트 홈 위치 |
| **B** | 리프트 정지 |
| **Menu** | 비상정지 토글 |
| **View** | 모드 토글 (TELEOP/SHARED) |

### 2. twist_mux

여러 cmd_vel 소스 중 우선순위에 따라 하나를 선택합니다.

**Priority Configuration:**
| Source | Topic | Priority | Description |
|--------|-------|----------|-------------|
| Gamepad | `/teleop/cmd_vel` | 100 | 최우선 (수동 조종) |
| Keyboard | `/key/cmd_vel` | 50 | 중간 (키보드 조종) |
| Navigation | `/nav/cmd_vel` | 10 | 최하위 (자율주행) |

**Output:** `/ackermann_steering_controller/reference` (via remapping)

## Topic Flow Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                      raptorlift.launch.py                        │
└──────────────────────────────────────────────────────────────────┘

┌────────┐
│  /joy  │
└────┬───┘
     │
     ▼
┌────────────────┐
│ gamepad_teleop │
│ (gear scaling) │
└────┬───────┬───┘
     │       │
     │       └──────▶ /gear_state (informational)
     │
     ▼
/teleop/cmd_vel (gear-scaled m/s) ────────────────▶ twist_mux
                                                       ▲
/key/cmd_vel (real m/s) ───────────────────────────────┤
(teleop_twist_keyboard --ros-args -p stamped:=true)    │
                                                       │
/nav/cmd_vel (real m/s) ───────────────────────────────┘
(nav2 → velocity_smoother → collision_monitor)

                                                       │
                                                       ▼
                          /ackermann_steering_controller/reference
                                                       │
                                                       ▼
                                    ackermann_steering_controller
                                                       │
                                                       ▼
                                     /raptorlift/joint_commands
```

## Configuration Files

| File | Location | Description |
|------|----------|-------------|
| `twist_mux.yaml` | `raptorlift_teleop/config/` | 입력 소스 토픽 및 우선순위 |
| `gamepad_config.yaml` | `raptorlift_teleop/config/` | 게임패드 파라미터 (참조용) |
| `raptorlift_controllers.yaml` | `raptorlift_bringup/config/` | ros2_control 컨트롤러 설정 |

## Monitoring Commands

```bash
# 현재 기어 상태
ros2 topic echo /gear_state

# gamepad_teleop 출력 (기어 스케일링된 실제 속도)
ros2 topic echo /teleop/cmd_vel

# 컨트롤러 입력 (twist_mux 출력)
ros2 topic echo /ackermann_steering_controller/reference

# 컨트롤러 상태
ros2 control list_controllers

# 노드 목록
ros2 node list
```

## Adding New Input Sources

### Step 1: Update twist_mux.yaml

```yaml
twist_mux:
  ros__parameters:
    topics:
      # 기존 소스들...

      # 새 소스 추가
      my_source:
        topic: /my/cmd_vel
        timeout: 0.5
        priority: 30  # 원하는 우선순위 (0-255)
```

### Step 2: Publish Real Velocity (m/s)

새 노드에서 **실제 속도 (m/s)** 를 TwistStamped로 퍼블리시:

```python
from geometry_msgs.msg import TwistStamped

msg = TwistStamped()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = "base_link"
msg.twist.linear.x = 0.5   # m/s (실제 속도)
msg.twist.angular.z = 0.3   # rad/s (실제 각속도)
publisher.publish(msg)
```

### Step 3: Rebuild

```bash
colcon build --packages-select raptorlift_teleop --symlink-install
```

## Troubleshooting

### 로봇이 움직이지 않음

1. 컨트롤러 상태 확인:
```bash
ros2 control list_controllers
# ackermann_steering_controller가 active인지 확인
```

2. twist_mux 출력 확인:
```bash
ros2 topic echo /ackermann_steering_controller/reference
```

3. gear_state 확인:
```bash
ros2 topic echo /gear_state
```

### 기어가 변경되지 않음

- 게임패드가 연결되어 있는지 확인: `ls /dev/input/js*`
- joy_node가 실행 중인지 확인: `ros2 node list | grep joy`

### 키보드 입력이 안 됨

- `stamped:=true` 파라미터 필수 — twist_mux가 `use_stamped: true`로 TwistStamped만 수신
- `frame_id:=base_link` 파라미터 추가 (stamped 사용 시 필수)
- `/key/cmd_vel` 토픽으로 리맵했는지 확인
- twist_mux에서 keyboard 우선순위가 gamepad보다 낮음 (50 < 100)
- 게임패드 입력이 없을 때만 키보드가 활성화됨

## Dependencies

- `twist_mux`: 다중 cmd_vel 선택
- `joy`: 게임패드 드라이버
- `ackermann_steering_controller`: Ackermann 기구학 컨트롤러
- `topic_based_ros2_control`: 토픽 기반 하드웨어 인터페이스
