# RaptorLift Teleop Architecture

## Overview

RaptorLift 로봇의 통합 제어 시스템. twist_mux를 통한 다중 입력 소스 지원과 20단 기어 기반 속도 스케일링을 제공합니다.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           raptorlift.launch.py                                   │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  ┌─────────────┐     ┌───────────┐     ┌─────────────────┐     ┌──────────────┐│
│  │ gamepad     │────▶│           │     │                 │     │              ││
│  │ /teleop/    │     │           │     │                 │     │ ackermann_   ││
│  │ cmd_vel     │     │ twist_mux │────▶│ velocity_scaler │────▶│ steering_    ││
│  └─────────────┘     │ (우선순위) │     │ (기어 스케일링)  │     │ controller   ││
│  ┌─────────────┐     │           │     │                 │     │              ││
│  │ keyboard    │────▶│           │     └────────▲────────┘     └──────────────┘│
│  │ /key/cmd_vel│     │           │              │                              │
│  └─────────────┘     │           │        /gear_state                          │
│  ┌─────────────┐     │           │              │                              │
│  │ nav2        │────▶│           │        gamepad_teleop                       │
│  │ /nav/cmd_vel│     └───────────┘                                             │
│  └─────────────┘                                                               │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## Launch Files

| Launch File | Contents | Use Case |
|-------------|----------|----------|
| `raptorlift.launch.py` | ros2_control + twist_mux + velocity_scaler + teleop | 수동 조종 (기본) |
| `navigation.launch.py` | raptorlift.launch.py + nav2 stack | 자율주행 (TODO) |

## Quick Start

```bash
# 시뮬레이션 모드 (기본)
ros2 launch raptorlift_bringup raptorlift.launch.py

# 실제 하드웨어
ros2 launch raptorlift_bringup raptorlift.launch.py simulation_mode:=false

# Isaac Sim 연동 (hardware_bridge 없이)
ros2 launch raptorlift_bringup raptorlift.launch.py use_hardware_bridge:=false

# 키보드 조종 추가 (별도 터미널)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/key/cmd_vel

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
| `control_rate` | `100.0` | 제어 루프 주기 (Hz) |

## Nodes

### 1. gamepad_teleop

Xbox 컨트롤러 입력을 정규화된 cmd_vel과 기어 상태로 변환합니다.

**Subscriptions:**
| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | sensor_msgs/Joy | 게임패드 raw 입력 |

**Publications:**
| Topic | Type | Description |
|-------|------|-------------|
| `/teleop/cmd_vel` | geometry_msgs/TwistStamped | 정규화된 속도 [-1, 1] |
| `/gear_state` | std_msgs/Int32 | 현재 기어 (1-20) |
| `/lift_command` | std_msgs/Int8 | 리프트 명령 (-1, 0, 1, 2) |
| `/teleop_state` | std_msgs/String | 상태 정보 문자열 |

**Xbox Controller Mapping:**
| Input | Function |
|-------|----------|
| **Left Stick Y** | 전진/후진 (정규화 출력) |
| **Right Stick X** | 좌/우 회전 (정규화 출력) |
| **L1 (LB)** | 기어 다운 |
| **R1 (RB)** | 기어 업 |
| **LT (Left Trigger)** | 감속 (최대 70% 감소) |
| **RT (Right Trigger)** | 가속/터보 (최대 150% 증가) |
| **D-Pad** | 정밀 이동 (저속 고정값) |
| **A** | 리프트 하강 (hold) |
| **X** | 리프트 상승 (hold) |
| **Y** | 리프트 홈 위치 |
| **B** | 리프트 정지 |
| **Menu (☰)** | 비상정지 토글 |
| **View (⧉)** | 모드 토글 (TELEOP/SHARED) |

### 2. twist_mux

여러 cmd_vel 소스 중 우선순위에 따라 하나를 선택합니다.

**Priority Configuration:**
| Source | Topic | Priority | Description |
|--------|-------|----------|-------------|
| Gamepad | `/teleop/cmd_vel` | 100 | 최우선 (수동 조종) |
| Keyboard | `/key/cmd_vel` | 50 | 중간 (키보드 조종) |
| Navigation | `/nav/cmd_vel` | 10 | 최하위 (자율주행) |

**Output:** `/cmd_vel_mux`

### 3. velocity_scaler

기어 상태에 따라 입력 속도를 스케일링합니다.

**Subscriptions:**
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel_mux` | geometry_msgs/TwistStamped | twist_mux 출력 (정규화) |
| `/gear_state` | std_msgs/Int32 | 현재 기어 |

**Publications:**
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel_scaled` | geometry_msgs/TwistStamped | 스케일링된 속도 |
| `/ackermann_steering_controller/reference` | geometry_msgs/TwistStamped | 컨트롤러 입력 |

**Gear-Velocity Mapping (20단):**
```
Gear  1:  0.1 m/s  (최저속)
Gear  5:  0.5 m/s
Gear 10:  1.05 m/s (기본)
Gear 15:  1.5 m/s
Gear 20:  2.0 m/s  (최고속)
```

**공식:** `velocity = min_vel + (max_vel - min_vel) × (gear - 1) / (num_gears - 1)`

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_linear_vel` | 0.1 | 기어 1 최대 속도 (m/s) |
| `max_linear_vel` | 2.0 | 기어 20 최대 속도 (m/s) |
| `max_angular_vel` | 1.0 | 최대 각속도 (rad/s) |
| `num_gears` | 20 | 기어 단수 |
| `default_gear` | 10 | 기본 기어 |

## Topic Flow Diagram

```
                              ┌──────────────────────────────────────┐
                              │           raptorlift.launch.py       │
                              └──────────────────────────────────────┘
                                               │
    ┌──────────────────────────────────────────┼──────────────────────────────────────────┐
    │                                          │                                          │
    ▼                                          ▼                                          ▼
┌────────┐                              ┌─────────────┐                            ┌───────────┐
│  /joy  │                              │ twist_mux   │                            │ ros2_     │
└────┬───┘                              │   config    │                            │ control   │
     │                                  └─────────────┘                            └───────────┘
     ▼                                         │
┌────────────────┐                             │
│ gamepad_teleop │                             │
└────┬───────┬───┘                             │
     │       │                                 │
     │       └──────▶ /gear_state ─────────────┼─────────────────────┐
     │                                         │                     │
     ▼                                         ▼                     │
/teleop/cmd_vel ─────────────────────────▶ twist_mux                 │
                                              ▲                      │
/key/cmd_vel ─────────────────────────────────┤                      │
(teleop_twist_keyboard)                       │                      │
                                              │                      │
/nav/cmd_vel ─────────────────────────────────┘                      │
(nav2)                                                               │
                                              │                      │
                                              ▼                      │
                                        /cmd_vel_mux                 │
                                              │                      │
                                              ▼                      │
                                      velocity_scaler ◀──────────────┘
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
| `velocity_scaler.yaml` | `raptorlift_teleop/config/` | 기어-속도 매핑 파라미터 |
| `raptorlift_controllers.yaml` | `raptorlift_bringup/config/` | ros2_control 컨트롤러 설정 |

## Monitoring Commands

```bash
# 현재 기어 상태
ros2 topic echo /gear_state

# twist_mux 출력 (정규화된 값)
ros2 topic echo /cmd_vel_mux

# velocity_scaler 출력 (실제 속도)
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

### Step 2: Publish Normalized cmd_vel

새 노드에서 정규화된 cmd_vel [-1, 1]을 퍼블리시:

```python
# Python 예제
from geometry_msgs.msg import TwistStamped

msg = TwistStamped()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = "base_link"
msg.twist.linear.x = 0.5   # -1.0 ~ 1.0 (정규화)
msg.twist.angular.z = 0.0  # -1.0 ~ 1.0 (정규화)
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
ros2 topic echo /cmd_vel_mux
# 값이 나오는지 확인
```

3. gear_state 확인:
```bash
ros2 topic echo /gear_state
# 기어 값이 퍼블리시되는지 확인
```

### 기어가 변경되지 않음

- 게임패드가 연결되어 있는지 확인: `ls /dev/input/js*`
- joy_node가 실행 중인지 확인: `ros2 node list | grep joy`

### 키보드 입력이 안 됨

- `/key/cmd_vel` 토픽으로 리맵했는지 확인
- twist_mux에서 keyboard 우선순위가 gamepad보다 낮음 (50 < 100)
- 게임패드 입력이 없을 때만 키보드가 활성화됨

## Dependencies

- `twist_mux`: 다중 cmd_vel 선택
- `joy`: 게임패드 드라이버
- `ackermann_steering_controller`: Ackermann 기구학 컨트롤러
- `topic_based_ros2_control`: 토픽 기반 하드웨어 인터페이스
