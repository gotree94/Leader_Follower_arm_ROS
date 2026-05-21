# 01. 시스템 아키텍처 (System Architecture)

## 1. 개요

본 문서는 Leader_Follower_arm_ROS 프로젝트의 전체 시스템 아키텍처를 정의합니다.
6축 Leader arm의 조작 데이터를 ROS를 통해 Follower arm으로 전달하고, Isaac Sim 시뮬레이션과 Jetson Orin Nano에서의 AI 추론까지 연결하는 엔드-투-엔드 시스템을 설계합니다.

## 2. 시스템 계층 구조

```
┌──────────────────────────────────────────────────────────────────────┐
│                        APPLICATION LAYER                             │
│  ┌─────────────┐ ┌──────────────┐ ┌────────────┐ ┌───────────────┐  │
│  │Leader Ctrl  │ │Follower Ctrl │ │Isaac Sim   │ │Jetson Inf.   │  │
│  │(Python/C++) │ │(Python/C++)  │ │(Python)    │ │(Python/TensorRT)│  │
│  └──────┬──────┘ └──────┬───────┘ └──────┬─────┘ └──────┬────────┘  │
├─────────┼───────────────┼────────────────┼──────────────┼────────────┤
│                    ROS MIDDLEWARE LAYER                              │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  Topics: /leader_joint_states, /follower_joint_states,     │    │
│  │          /leader_arm_cmd, /follower_arm_cmd                │    │
│  │  Services: /home_arm, /calibrate, /set_mode                │    │
│  │  Actions: /record_trajectory, /playback_trajectory         │    │
│  └─────────────────────────────────────────────────────────────┘    │
├──────────────────────────────────────────────────────────────────────┤
│                   COMMUNICATION LAYER                                │
│  ┌──────────────┐  ┌───────────────┐  ┌─────────────────────────┐  │
│  │rosserial/    │  │USB-to-TTL     │  │ros2_tcp_endpoint /      │  │
│  │micro-ROS     │  │(CP2102/FT232) │  │WebSocket (Isaac Sim)    │  │
│  └──────┬───────┘  └──────┬────────┘  └──────────┬──────────────┘  │
├─────────┼─────────────────┼───────────────────────┼────────────────┤ │
│                    HARDWARE ABSTRACTION LAYER                        │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │  STM32 HAL / Dynamixel SDK / AX-12A Protocol 1.0           │    │
│  └─────────────────────────────────────────────────────────────┘    │
├──────────────────────────────────────────────────────────────────────┤
│                    PHYSICAL LAYER                                    │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────┐    │
│  │AX-12A × 6   │  │AX-12A × 6   │  │STM32F103 (Leader/      │    │
│  │(Leader Arm)  │  │(Follower Arm)│  │Follower 각 1기)        │    │
│  └──────────────┘  └──────────────┘  └────────────────────────┘    │
└──────────────────────────────────────────────────────────────────────┘
```

## 3. 시스템 구성 요소

### 3.1 하드웨어 계층

| 구성 요소 | 사양 | 수량 | 역할 |
|-----------|------|------|------|
| AX-12A Dynamixel | 6축, 300° 회전, 1.5Nm 토크 | 12 | Leader 6 + Follower 6 관절 구동 |
| STM32F103C8T6 | 72MHz Cortex-M3, 64KB Flash | 2 | Leader/Follower 각각 제어 |
| USB-to-TTL 변환기 | CP2102 / FT232RL | 2 | STM32 ↔ ROS 직렬 통신 |
| SMPS 12V 5A | 12V DC 전원공급장치 | 2 | AX-12A 구동 전원 |
| 3.3V 레귤레이터 | AMS1117-3.3 | 2 | STM32 보드 전원 |
| 5V 레귤레이터 | LM2596 | 2 | AX-12A 로직 전원 (선택) |

### 3.2 미들웨어 계층 (ROS)

**ROS 버전 선택:**
- **ROS1 Noetic (권장)**: rosserial이 안정적, Dynamixel 패키지 호환성 우수
- **ROS2 Humble**: micro-ROS, Isaac Sim과의 네이티브 호환성, Jetson 지원 우수
- **권장: ROS2 Humble** — Isaac Sim 및 Jetson과의 통합이 원활함

**핵심 ROS 노드:**

| 노드 | 패키지 | 역할 |
|------|--------|------|
| `leader_joint_state_publisher` | `leader_arm_controller` | Leader arm 조인트 각도를 읽어 `/leader_joint_states` 토픽 발행 |
| `follower_joint_command` | `follower_arm_controller` | `/leader_joint_states` 구독 → Follower arm에 명령 전송 |
| `joint_state_relay` | `leader_follower_bringup` | Leader → Follower 조인트 상태 중계 및 변환 |
| `isaac_sim_bridge` | `leader_follower_sim` | Isaac Sim ↔ ROS 간 데이터 브릿징 |
| `inference_node` | `jetson_inference` | 학습된 모델로 실시간 추론 결과를 ROS 토픽으로 발행 |

### 3.3 관절 구성 (6축)

```
Leader/Follower Arm Joint Definition (DH Convention)

Joint 1 (J1): Base Rotation  - Waist (Yaw)       - AX-12A ID 1
Joint 2 (J2): Shoulder       - Shoulder (Pitch)   - AX-12A ID 2
Joint 3 (J3): Elbow          - Elbow (Pitch)       - AX-12A ID 3
Joint 4 (J4): Wrist Roll     - Wrist (Roll)        - AX-12A ID 4
Joint 5 (J5): Wrist Pitch    - Wrist (Pitch)       - AX-12A ID 5
Joint 6 (J6): Wrist Yaw      - Wrist (Yaw)         - AX-12A ID 6
```

## 4. 데이터 흐름

### 4.1 실시간 제어 데이터 흐름

```
[Leader Arm 조작]
     ↓ (사용자가 직접 조작)
[AX-12A 각 조인트 위치 피드백 (Present Position)]
     ↓ (Dynamixel Protocol 1.0 Read 명령: 0x02, 0x24)
[STM32F103 (Leader)] - UART1로 AX-12A polling (1ms 간격)
     ↓ (직렬 패킷 직렬화)
[USB-to-TTL (CP2102)]
     ↓ (ROS Serial Protocol / Micro-ROS)
[ROS Leader Node] - /leader_joint_states (sensor_msgs/JointState)
     ↓ (ROS Topic, 100Hz)
[ROS Follower Node] - Subscribe to /leader_joint_states
     ↓ (직렬 패킷 직렬화)
[USB-to-TTL (CP2102)]
     ↓ (ROS Serial Protocol)
[STM32F103 (Follower)]
     ↓ (Dynamixel Protocol 1.0 Goal Position Write: 0x03, 0x1E)
[AX-12A 각 조인트 위치 명령 실행]
     ↓
[Follower Arm 동기화 완료]
```

### 4.2 시뮬레이션 데이터 흐름

```
[Isaac Sim]
     ↓ URDF 로드
[6축 Arm 모델 시뮬레이션]
     ↓ ROS2 Bridge를 통한 joint state 동기화
[실제 하드웨어와 동일한 ROS 토픽 구조]
     ↓ 데이터 로깅 / 시각화
[개발 및 디버깅]
```

### 4.3 AI 추론 데이터 흐름

```
[Leader Arm 조작 데이터]
     ↓ (장시간 녹화, rosbag)
[Trajectory Dataset]
     ↓ (전처리: 필터링, 정규화, 증강)
[모델 학습 (PC/Cloud)]
     ↓ (학습된 가중치)
[TensorRT 변환]
     ↓ (FP16/INT8 양자화)
[Jetson Orin Nano 배포]
     ↓ (실시간 추론 → /inference_joint_states)
[Follower Arm 실행 또는 시뮬레이션 검증]
```

## 5. ROS 토픽 정의

### 5.1 주요 토픽

| 토픽명 | 메시지 타입 | 발행자 | 설명 |
|--------|------------|--------|------|
| `/leader_joint_states` | `sensor_msgs/JointState` | leader_arm_controller | Leader arm 6축 현재 각도 |
| `/follower_joint_states` | `sensor_msgs/JointState` | follower_arm_controller | Follower arm 6축 현재 각도 |
| `/leader_joint_command` | `sensor_msgs/JointState` | leader_arm_controller (cmd) | Leader arm 목표 각도 |
| `/follower_joint_command` | `std_msgs/Float64MultiArray` | follower_arm_controller | Follower arm 명령 각도 |
| `/leader_follower_mode` | `std_msgs/String` | bringup | 시스템 모드 (CALIBRATION/FOLLOWING/SIMULATION/INFERENCE) |
| `/arm_status` | `std_msgs/String` | both | 시스템 상태 정보 |
| `/inference_joint_states` | `sensor_msgs/JointState` | jetson_inference | AI 추론 결과 각도 |

### 5.2 서비스

| 서비스명 | 타입 | 설명 |
|----------|------|------|
| `/home_arm` | `std_srvs/Trigger` | 모든 조인트 홈 포지션으로 이동 |
| `/calibrate_joints` | `std_srvs/Trigger` | 조인트 캘리브레이션 수행 |
| `/set_mode` | `SetMode.srv` | 시스템 모드 변경 |
| `/record_start` | `std_srvs/Trigger` | 궤적 녹화 시작 |
| `/record_stop` | `std_srvs/Trigger` | 궤적 녹화 중지 |
| `/inference_start` | `std_srvs/Trigger` | AI 추론 모드 시작 |

### 5.3 커스텀 메시지 (`leader_follower_msgs`)

**SetMode.srv:**
```
string mode           # "FOLLOWING", "SIMULATION", "INFERENCE", "CALIBRATION"
---
bool success
string message
```

**JointPosition.msg:**
```
float64[6] joint_angles  # J1~J6 각도 (degrees)
float64[6] joint_velocities
float64[6] joint_torques
float64    timestamp
```

## 6. 타이밍 및 성능 요구사항

| 파라미터 | 목표값 | 비고 |
|----------|--------|------|
| 제어 루프 주기 (Leader) | 1ms (1kHz) | STM32 타이머 인터럽트 |
| 조인트 상태 발행 주기 | 10ms (100Hz) | ROS 토픽 발행 |
| Follower 명령 수신~실행 지연 | < 5ms | 직렬 통신 + AX-12A write |
| Leader→Follower 종단간 지연 | < 20ms | 전체 시스템 레이턴시 |
| Isaac Sim 동기화 주기 | 33ms (30Hz) | 시뮬레이션 실시간성 |
| 데이터 로깅 주기 | 10ms (100Hz) | rosbag 또는 CSV |

## 7. 안전 및 오류 처리

| 상황 | 대응 |
|------|------|
| AX-12A 통신 타임아웃 | 재시도 (3회) → 타임아웃 발생 시 ROS `/arm_status`에 에러 발행 |
| STM32-Watchdog 타이머 | IWDG 1초 설정, 메인 루프에서 리셋 |
| Follower 명령 미수신 (1초) | 안전 정지 모드 진입, 모든 AX-12A 토크 해제 |
| 과전류 감지 | AX-12A 자체 과전류 보호 + STM32 ADC 모니터링 |
| ROS 노드 크래시 | roslaunch `respawn=true` 설정으로 자동 재시작 |
| 비상 정지 (E-Stop) | 하드웨어 E-Stop 스위치 → AX-12A 토크 차단 릴레이 |
