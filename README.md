# Leader_Follower_arm_ROS

## 6축 Leader-Follower 로봇팔 프로젝트 (AX-12A + STM32F103 + ROS + Isaac Sim + Jetson Orin)

![Project Status](https://img.shields.io/badge/status-planning-yellow)
![ROS](https://img.shields.io/badge/ROS-Noetic%20%2F%20ROS2%20Humble-blue)
![STM32](https://img.shields.io/badge/MCU-STM32F103-green)
![Isaac Sim](https://img.shields.io/badge/Sim-Isaac%20Sim-brightgreen)
![Jetson](https://img.shields.io/badge/Edge-Jetson%20Orin%20Nano-orange)

---

## 프로젝트 개요

**Leader_Follower_arm_ROS** 는 AX-12A Dynamixel 서보모터를 이용한 6축 Leader arm의 자세를 ROS를 통해 Follower arm에 실시간으로 전달하여 동기화 동작을 구현하는 로봇팔 시스템입니다.

### 핵심 기술 스택

| 구성 요소 | 기술 | 비고 |
|-----------|------|------|
| **액추에이터** | AX-12A (Dynamixel) × 12 | Leader 6축 + Follower 6축 |
| **메인 제어기** | STM32F103C8T6 (Blue Pill) | Leader/Follower 각 1개 |
| **미들웨어** | ROS (Noetic / ROS2 Humble) | 메시지 브로커링 |
| **시뮬레이션** | NVIDIA Isaac Sim | URDF 기반 시뮬레이션 및 데이터 생성 |
| **AI 추론** | Jetson Orin Nano | 추론 데이터 수집 및 모델 실행 |
| **통신** | UART (AX-12A) ↔ USB-TTL (ROS) | rosserial / micro-ROS |

### 주요 목표

1. **6축 Leader Arm 조작** → 사용자가 Leader arm을 직접 움직이면 각 조인트의 각도를 실시간 읽기
2. **ROS 기반 자세 전달** → Leader arm의 joint state를 ROS 토픽으로 publish
3. **Follower Arm 동기화** → Follower arm이 Leader arm의 자세를 실시간 추종
4. **STM32F103 제어** → AX-12A 서보的直接 제어 및 ROS 직렬 통신
5. **Isaac Sim 시뮬레이션** → 가상 환경에서의 로봇 동작 검증 및 데이터 수집
6. **Jetson Orin Nano 추론** → 수집된 데이터 기반 AI 모델 학습 및 실시간 추론

---

## 프로젝트 구조

```
Leader_Follower_arm_ROS/
├── README.md                          # 프로젝트 개요 (본 파일)
├── docs/                              # 상세 설계 문서
│   ├── 01_System_Architecture.md      # 시스템 아키텍처
│   ├── 02_Hardware_Design.md          # 하드웨어 설계 (BOM, 전기, 기구)
│   ├── 03_STM32_Firmware.md           # STM32 펌웨어 설계
│   ├── 04_ROS_Integration.md          # ROS 통합 설계
│   ├── 05_Isaac_Sim_Integration.md    # Isaac Sim 연동
│   ├── 06_Jetson_Inference.md         # Jetson Orin Nano 추론
│   ├── 07_Communication_Protocol.md   # 통신 프로토콜 정의
│   └── 08_Development_Roadmap.md      # 개발 로드맵
├── hardware/                          # 하드웨어 설계 파일
│   ├── mechanical/                    # 기구 설계
│   ├── electrical/                    # 전기/전자 설계
│   └── bom/                           # 부품 목록
├── firmware/                          # STM32 펌웨어
│   ├── stm32_leader/                  # Leader arm 펌웨어
│   └── stm32_follower/                # Follower arm 펌웨어
├── ros_ws/                            # ROS 워크스페이스
│   ├── src/
│   │   ├── leader_follower_description/   # URDF 모델
│   │   ├── leader_follower_bringup/       # Launch 파일
│   │   ├── leader_arm_controller/         # Leader arm ROS 노드
│   │   ├── follower_arm_controller/       # Follower arm ROS 노드
│   │   ├── leader_follower_msgs/          # 커스텀 메시지
│   │   └── leader_follower_sim/           # 시뮬레이션 브릿지
├── simulation/                        # 시뮬레이션 파일
│   └── isaac_sim/                     # Isaac Sim 관련
├── inference/                         # AI 추론
│   └── jetson/                        # Jetson Orin Nano
└── scripts/                           # 유틸리티 스크립트
```

---

## 시스템 개요

```
┌─────────────────────────────────────────────────────────────────┐
│                        Leader Side                              │
│  ┌──────────┐    ┌──────────────┐    ┌───────────────────────┐  │
│  │  AX-12A  │◄──►│  STM32F103   │◄──►│  ROS Leader Node     │  │
│  │  6축 Arm  │    │  (UART→USB)  │    │  (rosserial/microROS)│  │
│  └──────────┘    └──────────────┘    └───────────┬───────────┘  │
│                                                   │              │
└───────────────────────────────────────────────────┼──────────────┘
                                                      │ ROS Topic
                                                      │ /leader_joint_states
                                                      │
┌───────────────────────────────────────────────────┼──────────────┐
│                        Follower Side              │              │
│  ┌──────────┐    ┌──────────────┐    ┌───────────▼───────────┐  │
│  │  AX-12A  │◄──►│  STM32F103   │◄──►│  ROS Follower Node   │  │
│  │  6축 Arm  │    │  (UART→USB)  │    │  (rosserial/microROS)│  │
│  └──────────┘    └──────────────┘    └───────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                     Simulation & AI                             │
│  ┌──────────────────┐    ┌────────────────────────────────┐    │
│  │   Isaac Sim      │◄──►│   Jetson Orin Nano             │    │
│  │   (URDF 기반)     │    │   (추론 데이터 수집/모델 실행)  │    │
│  └──────────────────┘    └────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

---

## 라이선스

이 프로젝트는 교육 및 연구 목적으로 제공됩니다.
