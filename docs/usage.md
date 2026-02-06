# RaptorLift ros2_control 사용 가이드

## 목차
1. [시스템 요구사항](#시스템-요구사항)
2. [빌드](#빌드)
3. [실행 모드](#실행-모드)
4. [텔레옵 조작](#텔레옵-조작)
5. [토픽 모니터링](#토픽-모니터링)
6. [문제 해결](#문제-해결)

---

## 시스템 요구사항

### 필수
- ROS 2 Jazzy
- Ubuntu 24.04
- colcon build tools

### 선택 (용도별)
| 용도 | 요구사항 |
|------|----------|
| 시뮬레이션 (Virtual PLC) | 기본 패키지만 |
| Isaac Sim 연동 | Isaac Sim 5.1+ |
| 실제 하드웨어 | libmodbus-dev |
| 텔레옵 | Xbox 호환 게임패드 |

---

## 빌드

### 기본 빌드 (시뮬레이션 전용)
```bash
cd ~/RaptorYamato/IsaacSim-ros_workspaces/jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### 실제 하드웨어 지원 빌드
```bash
# libmodbus 설치
sudo apt-get install libmodbus-dev

# 재빌드
colcon build --packages-select raptorlift_hardware_bridge
source install/setup.bash
```

---

## 실행 모드

### 1. 시뮬레이션 모드 (Virtual PLC)

외부 시뮬레이터 없이 모터 동역학을 시뮬레이션합니다.

```bash
# 터미널 1: ros2_control 스택 실행
ros2 launch raptorlift_bringup raptorlift.launch.py \
    simulation_mode:=true \
    use_hardware_bridge:=true

# 터미널 2: 텔레옵 실행 (선택)
ros2 launch raptorlift_bringup teleop.launch.py
```

**파라미터:**
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `simulation_mode` | true | Virtual PLC 사용 |
| `use_hardware_bridge` | true | hardware_bridge 노드 실행 |
| `control_rate` | 100.0 | 제어 루프 주파수 (Hz) |
| `detailed_logging` | false | 상세 로그 출력 |

### 2. Isaac Sim 모드

Isaac Sim이 물리 시뮬레이션을 제공합니다.

#### Isaac Sim 실행 방법

**방법 A: 직접 실행**
```bash
~/isaacsim/_build/linux-x86_64/release/isaac-sim.sh
```

**방법 B: 별칭 사용 (선택)**
```bash
# ~/.bashrc에 추가 (최초 1회)
echo 'alias isaacsim="~/isaacsim/_build/linux-x86_64/release/isaac-sim.sh"' >> ~/.bashrc
source ~/.bashrc

# 이후 간단히 실행
isaacsim
```

**사용 가능한 실행 스크립트:**
| 스크립트 | 용도 |
|----------|------|
| `isaac-sim.sh` | 기본 GUI 모드 |
| `isaac-sim.streaming.sh` | 스트리밍 모드 (원격) |
| `isaac-sim.selector.sh` | 버전 선택기 |

#### RaptorLift + Isaac Sim 연동 실행

```bash
# 터미널 1: Isaac Sim 실행
~/isaacsim/_build/linux-x86_64/release/isaac-sim.sh
```

**Isaac Sim GUI에서 설정:**
1. `File` > `Import` > `URDF` 선택
2. URDF 파일 경로 입력:
   ```
   ~/RaptorYamato/IsaacSim-ros_workspaces/jazzy_ws/src/raptorlift_description/urdf/raptorlift.urdf
   ```
3. `Window` > `Script Editor` 열기
4. `setup_isaac_sim.py` 스크립트 열고 실행 (Ctrl+Enter)
   ```
   ~/RaptorYamato/IsaacSim-ros_workspaces/jazzy_ws/src/raptorlift_description/scripts/setup_isaac_sim.py
   ```
5. **Play** 버튼 클릭 (시뮬레이션 시작)

```bash
# 터미널 2: ros2_control 스택 실행 (hardware_bridge 없이)
cd ~/RaptorYamato/IsaacSim-ros_workspaces/jazzy_ws
source install/setup.bash
ros2 launch raptorlift_bringup raptorlift.launch.py \
    use_hardware_bridge:=false \
    use_sim_time:=true

# 터미널 3: 텔레옵 실행
ros2 launch raptorlift_bringup teleop.launch.py \
    use_sim_time:=true
```

### 3. 실제 하드웨어 모드 (Modbus TCP)

Mitsubishi 서보와 Modbus TCP로 통신합니다.

```bash
# 사전 준비: libmodbus 설치 및 재빌드 필요

# ros2_control 스택 실행
ros2 launch raptorlift_bringup raptorlift.launch.py \
    simulation_mode:=false \
    modbus_ip:=192.168.2.1 \
    modbus_port:=502

# 텔레옵 실행
ros2 launch raptorlift_bringup teleop.launch.py
```

**Modbus 파라미터:**
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `modbus_ip` | 192.168.2.1 | PLC IP 주소 |
| `modbus_port` | 502 | Modbus TCP 포트 |

---

## 텔레옵 조작

### Xbox 컨트롤러 매핑

| 입력 | 기능 |
|------|------|
| 왼쪽 스틱 Y축 | 전진/후진 (linear.x) |
| 오른쪽 스틱 X축 | 조향 (angular.z) |
| RB (R1) | 데드맨 스위치 (누르고 있어야 동작) |
| LT + RT | 속도 스케일 조절 |

### 속도 제한

```yaml
# raptorlift_teleop/config/teleop.yaml
max_linear_speed: 2.0      # m/s
max_angular_speed: 1.0     # rad/s
```

---

## 토픽 모니터링

### 주요 토픽

```bash
# 조인트 상태 확인
ros2 topic echo /joint_states

# 조인트 명령 확인 (ros2_control → hardware_bridge)
ros2 topic echo /raptorlift/joint_commands

# 조인트 피드백 확인 (hardware_bridge → ros2_control)
ros2 topic echo /raptorlift/joint_states

# 속도 명령 확인
ros2 topic echo /cmd_vel

# 컨트롤러 상태 확인
ros2 topic echo /ackermann_steering_controller/controller_state
```

### 컨트롤러 상태 확인

```bash
# 컨트롤러 목록
ros2 control list_controllers

# 하드웨어 인터페이스 목록
ros2 control list_hardware_interfaces

# 컨트롤러 매니저 상태
ros2 control list_hardware_components
```

---

## 문제 해결

### QoS 불일치 오류

**증상:** 토픽 메시지가 수신되지 않음
```
[WARN] QoS requested by subscriber is incompatible with QoS offered by publisher
```

**해결:** hardware_bridge는 RELIABLE QoS를 사용합니다. 커스텀 노드 작성 시 동일한 QoS를 사용하세요:
```cpp
auto qos = rclcpp::QoS(10).reliable().durability_volatile();
```

### Modbus 연결 실패

**증상:**
```
[WARN] Failed to connect to Modbus - falling back to simulation mode
```

**해결:**
1. libmodbus 설치 확인: `dpkg -l | grep libmodbus`
2. 네트워크 연결 확인: `ping 192.168.2.1`
3. 방화벽 확인: `sudo ufw allow 502/tcp`

### 컨트롤러 활성화 실패

**증상:**
```
[ERROR] Could not switch controllers
```

**해결:**
```bash
# 컨트롤러 수동 활성화
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state ackermann_steering_controller active
```

### Isaac Sim 토픽 연결 안됨

**증상:** `/raptorlift/joint_states` 토픽에 데이터 없음

**해결:**
1. Isaac Sim에서 Play 버튼이 눌려있는지 확인
2. setup_isaac_sim.py 스크립트가 실행되었는지 확인
3. ROS 2 Bridge가 활성화되었는지 확인

---

## PID 파라미터 튜닝

### Steering (Position → Torque)
```bash
ros2 param set /hardware_bridge steering_kp 50.0
ros2 param set /hardware_bridge steering_ki 0.0
ros2 param set /hardware_bridge steering_kd 5.0
ros2 param set /hardware_bridge max_steering_torque 100.0
```

### Traction (Velocity → Torque)
```bash
ros2 param set /hardware_bridge traction_kp 10.0
ros2 param set /hardware_bridge traction_ki 5.0
ros2 param set /hardware_bridge traction_kd 0.0
ros2 param set /hardware_bridge max_traction_torque 150.0
```

---

## 빠른 시작 요약

| 모드 | 명령어 |
|------|--------|
| 시뮬레이션 | `ros2 launch raptorlift_bringup raptorlift.launch.py` |
| Isaac Sim | `ros2 launch raptorlift_bringup raptorlift.launch.py use_hardware_bridge:=false use_sim_time:=true` |
| 실제 하드웨어 | `ros2 launch raptorlift_bringup raptorlift.launch.py simulation_mode:=false modbus_ip:=<PLC_IP>` |
| 텔레옵 추가 | `ros2 launch raptorlift_bringup teleop.launch.py` |

---

*Last updated: 2026-02-05*
