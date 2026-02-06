# Raptor Description Package Manual

## 환경 설정

```bash
# 자동 설정 (~/.bashrc에 추가됨)
source ~/RaptorYamato/IsaacSim-ros_workspaces/jazzy_ws/setup_env.bash
```

---

## raptorlift_description

로봇 URDF 및 시각화 패키지

### Launch Files

#### 1. display.launch.py
RViz에서 로봇 모델 확인 (Joint State Publisher GUI 포함)

```bash
ros2 launch raptorlift_description display.launch.py
```

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `use_sim_time` | `false` | 시뮬레이션 시간 사용 |

#### 2. isaac_sim.launch.py
Isaac Sim 연동용 (Robot State Publisher + RViz)

```bash
ros2 launch raptorlift_description isaac_sim.launch.py
```

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `use_sim_time` | `true` | Isaac Sim 시뮬레이션 시간 |

#### 3. navigation.launch.py
Nav2 네비게이션 스택 실행

```bash
ros2 launch raptorlift_description navigation.launch.py map:=/path/to/map.yaml
```

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `use_sim_time` | `true` | 시뮬레이션 시간 |
| `map` | `''` | 맵 YAML 파일 경로 (필수) |

---

## carter_navigation

Nova Carter 로봇 네비게이션 (Isaac Sim 샘플)

```bash
ros2 launch carter_navigation carter_navigation.launch.py
```

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `map` | warehouse map | 맵 파일 |
| `use_sim_time` | `true` | 시뮬레이션 시간 |

---

## Isaac Sim 실행

```bash
cd ~/isaacsim/_build/linux-x86_64/release
./isaac-sim.sh
```

---

## 테스트 명령어

### Joint States 확인
```bash
ros2 topic echo /joint_states
```

### TF 확인
```bash
ros2 run tf2_ros tf2_echo base_link front_left_wheel_link
```

### 바퀴 회전 테스트 (터미널)
```bash
# 양쪽 바퀴 +1.0 rad 위치
ros2 topic pub /joint_states sensor_msgs/JointState "{
  name: ['front_left_wheel_joint', 'front_right_wheel_joint',
         'rear_left_steering_joint', 'rear_right_steering_joint',
         'rear_left_wheel_joint', 'rear_right_wheel_joint'],
  position: [1.0, 1.0, 0.0, 0.0, 0.0, 0.0]
}" -r 10
```

### cmd_vel 발행 (Ackermann Controller 사용 시)
```bash
ros2 topic pub /cmd_vel geometry_msgs/TwistStamped "{
  header: {frame_id: 'base_link'},
  twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}
}"
```

---

## 로봇 사양

| 항목 | 값 |
|------|-----|
| 이름 | raptorlift |
| 타입 | Ackermann Steering Forklift |
| Wheelbase | 1.0 m |
| Track Width | 0.71 m |
| Wheel Radius | 0.1715 m |
| 구동륜 | Front (포크 쪽) |
| 조향륜 | Rear (카운터웨이트 쪽) |

### 좌표계
- +X: 전진 (포크 방향)
- +Y: 좌측
- +Z: 위

### 조인트
| Joint | Type | 역할 |
|-------|------|------|
| front_left_wheel_joint | continuous | 구동 |
| front_right_wheel_joint | continuous | 구동 |
| rear_left_steering_joint | revolute | 조향 |
| rear_right_steering_joint | revolute | 조향 |
| rear_left_wheel_joint | continuous | 회전 |
| rear_right_wheel_joint | continuous | 회전 |

---

*Last updated: 2026-02-05*
