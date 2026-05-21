# 08. 개발 로드맵 (Development Roadmap)

## 1. 개발 단계 개요

총 8단계로 구성된 16주 개발 계획입니다.

```
Phase 1: 준비 및 환경 구축 (2주)
Phase 2: 기구 설계 및 제작 (3주)
Phase 3: STM32 펌웨어 개발 (2주)
Phase 4: ROS 통합 개발 (2주)
Phase 5: Leader-Follower 시스템 통합 (2주)
Phase 6: Isaac Sim 시뮬레이션 (2주)
Phase 7: Jetson Orin Nano 추론 시스템 (2주)
Phase 8: 통합 테스트 및 최적화 (1주)
```

## 2. 상세 일정

### Phase 1: 준비 및 환경 구축 (Week 1-2)

| 기간 | 작업 | 세부 내용 | 산출물 | 마일스톤 |
|------|------|----------|--------|---------|
| W1 D1-2 | 개발 환경 설치 | Ubuntu 22.04, ROS2 Humble, STM32CubeIDE | 개발 환경 설정 완료 | |
| W1 D3-4 | STM32 개발 환경 | ST-Link 드라이버, OpenOCD, HAL 라이브러리 | STM32 Blinky 테스트 통과 | ✅ STM32 Blinky |
| W1 D5 | AX-12A 준비 | Dynamixel Wizard 2.0 설치, AX-12A 개별 테스트 | AX-12A ID/Baud 설정 완료 | ✅ 6개 AX-12A ID 설정 |
| W2 D1-2 | 부품 조달 | BOM 기준 부품 주문, 3D 프린터 출력물 준비 | 부품 수령 완료 | |
| W2 D3-5 | 회로 설계 | KiCad/EAGLE로 회로도 작성, PCB 레이아웃 | 회로도 v1.0 | |

**검증 항목:**
- [x] ROS2 Humble 정상 작동 확인
- [x] STM32CubeIDE에서 STM32F103C8T6 프로젝트 생성 및 빌드
- [x] ST-Link로 STM32에 Blinky 펌웨어 업로드
- [x] AX-12A 6개 개별 Ping/Read/Write 테스트

### Phase 2: 기구 설계 및 제작 (Week 3-5)

| 기간 | 작업 | 세부 내용 | 산출물 | 마일스톤 |
|------|------|----------|--------|---------|
| W3 D1-3 | 3D 모델링 (CAD) | Fusion 360/SolidWorks로 6축 암 설계 | CAD 파일 완료 | |
| W3 D4-5 | URDF 생성 | CAD → URDF 변환 (sw_urdf_exporter) | URDF 파일 v1.0 | ✅ URDF 생성 완료 |
| W4 D1-5 | 3D 프린팅 | 브라켓, 마운트, 커플링 출력 (PETG) | 모든 출력물 완료 | ✅ 기구 부품 출력 완료 |
| W5 D1-3 | 가공 | 알루미늄 프레임 절단/드릴링 (필요시 CNC) | 프레임 가공 완료 | |
| W5 D4-5 | 조립 시작 | 베이스 → J1 → 상완 순차 조립 | Leader Arm 조립 50% | |

**검증 항목:**
- [x] CAD 모델 간섭 체크 (Interference Check)
- [x] STL 파일 3D 프린팅 슬라이싱 검증
- [x] AX-12A 마운트 체결 검증 (M2.5 나사)

### Phase 3: STM32 펌웨어 개발 (Week 6-7)

| 기간 | 작업 | 세부 내용 | 산출물 | 마일스톤 |
|------|------|----------|--------|---------|
| W6 D1-2 | Dynamixel 드라이버 | DX_Init, DX_Read, DX_Write, DX_Ping 구현 | dynamixel.c/h 완료 | ✅ AX-12A 드라이버 단위 테스트 통과 |
| W6 D3-4 | 조인트 관리자 | JointManager, Calibration 구현 | joint_manager.c 완료 | |
| W6 D5 | 제어 루프 | TIM 인터럽트 기반 100Hz 제어 루프 | control_loop.c 완료 | |
| W7 D1-3 | 직렬 프로토콜 | micro-ROS / 커스텀 프로토콜 구현 | serial_protocol.c 완료 | ✅ STM32 ↔ ROS 직렬 통신 테스트 |
| W7 D4-5 | 안전 기능 | E-Stop, Watchdog, 에러 처리, 안전 정지 | safety.c 완료 | ✅ 안전 기능 테스트 완료 |

**단위 테스트:**
```bash
# AX-12A Ping Test (STM32 → Serial)
ff ff 01 02 01 fb  # Ping ID=1 → 응답 확인

# Joint Read Test
ros2 topic echo /leader_joint_states  # 조인트 각도 발행 확인

# E-Stop Test
PA0 GND 연결 → AX-12A Torque OFF 확인
```

### Phase 4: ROS 통합 개발 (Week 8-9)

| 기간 | 작업 | 세부 내용 | 산출물 | 마일스톤 |
|------|------|----------|--------|---------|
| W8 D1-2 | URDF 패키지 | leader_follower_description 패키지 생성 | URDF + launch 완료 | ✅ RViz에서 URDF 시각화 |
| W8 D3-4 | Leader Controller | leader_arm_controller 노드 구현 | C++ 노드 완료 | ✅ Leader 조인트 상태 ROS 발행 |
| W8 D5 | Follower Controller | follower_arm_controller 노드 구현 | C++ 노드 완료 | |
| W9 D1-2 | 커스텀 메시지 | JointCommand, ArmStatus, SetMode 정의 | leader_follower_msgs 완료 | ✅ 메시지 빌드 확인 |
| W9 D3-4 | Bringup 패키지 | 시스템 launch 파일, 컨트롤러 설정 | leader_follower_bringup 완료 | ✅ 전체 시스템 launch 실행 |
| W9 D5 | ros2_control 통합 | 하드웨어 인터페이스 구현 | ros2_control 설정 완료 | |

### Phase 5: Leader-Follower 시스템 통합 (Week 10-11)

| 기간 | 작업 | 세부 내용 | 산출물 | 마일스톤 |
|------|------|----------|--------|---------|
| W10 D1-2 | Arm 조립 완료 | Leader + Follower Arm 전체 조립 및 배선 | Arm 2기 조립 완료 | ✅ 암 조립 완료 |
| W10 D3-4 | 캘리브레이션 | 각 조인트 오프셋 설정, 소프트웨어 제한 설정 | 캘리브레이션 데이터 | ✅ 캘리브레이션 완료 |
| W10 D5 | Leader 단독 테스트 | Leader arm 조작 → ROS 토픽 발행 확인 | 단독 테스트 완료 | |
| W11 D1-2 | Follower 단독 테스트 | ROS 명령 → Follower arm 동작 확인 | 단독 테스트 완료 | |
| W11 D3-4 | Leader→Follower 추종 테스트 | Leader 조작 → Follower 동기화 확인 | 통합 테스트 완료 | ✅ Leader-Follower 동기화 성공 |
| W11 D5 | 지연 시간 측정 | 종단간 지연 측정 및 최적화 | 성능 측정 리포트 | |

**통합 테스트 시나리오:**

```
시나리오 1: Joint-by-Joint
  - J1만 천천히 회전 → J1만 추종 확인
  - J2만 천천히 회전 → J2만 추종 확인
  - ... (모든 조인트 개별 확인)

시나리오 2: Sinusoidal Trajectory
  - 각 조인트 사인파 궤적 생성
  - 최대 추종 오차 측정 (< 2° 목표)

시나리오 3: Random Trajectory
  - 무작위 궤적 생성
  - 추종 정확도 및 지연 시간 측정

시나리오 4: E-Stop
  - 동작 중 E-Stop 활성화
  - 안전 정지 확인 (< 50ms)

시나리오 5: 장시간 안정성
  - 1시간 연속 동작
  - 온도, 전압, 드리프트 모니터링
```

### Phase 6: Isaac Sim 시뮬레이션 (Week 12-13)

| 기간 | 작업 | 세부 내용 | 산출물 | 마일스톤 |
|------|------|----------|--------|---------|
| W12 D1-2 | Isaac Sim 설치 | Isaac Sim 2024.1 설치, ROS2 Bridge 설정 | Isaac Sim 실행 확인 | ✅ Isaac Sim 실행 |
| W12 D3-4 | URDF 임포트 | Leader/Follower URDF → USD 변환 | USD 파일 완료 | ✅ URDF→USD 변환 성공 |
| W12 D5 | 시뮬레이션 기본 동작 | Python API로 조인트 제어 테스트 | 시뮬레이션 기본 동작 확인 | |
| W13 D1-2 | ROS2 Bridge | Isaac Sim ↔ ROS2 양방향 통신 구축 | Bridge 노드 완료 | ✅ ROS2 ↔ Isaac Sim 통신 |
| W13 D3-4 | Leader-Follower 시뮬레이션 | 시뮬레이션에서 Leader→Follower 추종 | 통합 시뮬레이션 완료 | |
| W13 D5 | 데이터 수집 파이프라인 | 시뮬레이션 데이터 rosbag/CSV 저장 | 데이터 수집 자동화 | ✅ 시뮬레이션 데이터 수집 |

### Phase 7: Jetson Orin Nano 추론 시스템 (Week 14-15)

| 기간 | 작업 | 세부 내용 | 산출물 | 마일스톤 |
|------|------|----------|--------|---------|
| W14 D1-2 | Jetson 환경 설정 | JetPack 6.0 설치, ROS2 Humble, PyTorch 설치 | Jetson 환경 완료 | ✅ Jetson 개발 환경 구축 |
| W14 D3-4 | 데이터 수집 | Leader arm 조작 데이터 대량 수집 (rosbag) | 10,000+ 샘플 데이터셋 | |
| W14 D5 | 데이터 전처리 | 정규화, 시퀀스 생성, Train/Val 분할 | 전처리 파이프라인 완료 | |
| W15 D1-2 | 모델 학습 | LSTM/Transformer 학습, 하이퍼파라미터 튜닝 | 학습된 모델 (best_model.pth) | ✅ 모델 학습 완료 (Val Loss < 0.01) |
| W15 D3 | TensorRT 변환 | ONNX → TensorRT FP16/INT8 변환 | TensorRT Engine (.engine) | ✅ TensorRT 변환 완료 |
| W15 D4-5 | 추론 노드 구현 | ROS2 Inference Node (C++) 구현 및 테스트 | Inference Node 완료 | ✅ Jetson 실시간 추론 성공 |

**추론 성능 검증 기준:**
```
- 추론 지연 시간 < 2ms (GPU)
- 처리량 > 500 FPS
- 모델 크기 < 50MB
- 추론 각도 오차 < 3° (실제 대비)
```

### Phase 8: 통합 테스트 및 최적화 (Week 16)

| 기간 | 작업 | 세부 내용 | 산출물 | 마일스톤 |
|------|------|----------|--------|---------|
| W16 D1 | 전체 시스템 통합 | 실제 하드웨어 + Isaac Sim + Jetson 동시 운영 | 통합 시스템 가동 | |
| W16 D2 | 모드별 테스트 | FOLLOWING / SIMULATION / INFERENCE 모드 전환 테스트 | 모드 전환 테스트 완료 | ✅ 모드 전환 정상 동작 |
| W16 D3 | 성능 최적화 | 지연 시간 최적화, 제어 루프 튜닝, 통신 최적화 | 성능 최적화 완료 | |
| W16 D4-5 | 문서화 및 마무리 | 최종 문서 정리, 회로도/소스 정리, 데모 준비 | 최종 산출물 | ✅ 프로젝트 완료 |

## 3. 위험 관리

### 3.1 기술 리스크

| 리스크 | 영향 | 확률 | 대응 방안 |
|--------|------|------|----------|
| AX-12A 토크 부족 (특히 J2) | 암 자체 무게 지지 불가 | 중 | 링크 경량화, 스프링 밸런싱, 구동 비율 조정 |
| STM32F103 메모리 부족 | micro-ROS 동작 불안정 | 중 | 경량 프로토콜 사용, FreeRTOS 대신 Super Loop |
| USB-to-TTL 통신 끊김 | ROS 연결 손실 | 낮음 | Watchdog 자동 재연결, 안전 정지 모드 |
| Isaac Sim 성능 부족 | 시뮬레이션 실시간성 저하 | 중 | 렌더링 해상도 조정, physics dt 조정 |
| Jetson 추론 지연 | 실시간 제어 불가 | 낮음 | TensorRT FP16/INT8 최적화, 모델 경량화 |
| 3D 프린팅 부품 강도 부족 | 구조적 파손 | 중 | PETG/CF-PETG 사용, 설계 보강, 필요시 CNC 가공 |

### 3.2 일정 리스크

| 리스크 | 대응 |
|--------|------|
| 부품 수입 지연 | 알리/아마존 대체 공급처 확보, 국내 부품사 우선 |
| 3D 프린팅 실패 | 여유분 출력, 2대 이상 프린터 동시 운영 |
| 예상치 못한 기술 이슈 | phases 간 0.5주 여유 버퍼 편성 |
| HW/SW 동시 디버깅 필요 | 분업: 한 사람은 HW, 한 사람은 SW |

## 4. 성공 기준

### 4.1 Phase 별 마일스톤

```
Phase 1: ✅ STM32 Blinky, AX-12A Ping/Read/Write
Phase 2: ✅ CAD 완료, 부품 출력 완료
Phase 3: ✅ AX-12A 드라이버, ROS 직렬 통신
Phase 4: ✅ URDF RViz 표시, ROS 노드 통신
Phase 5: ✅ Leader→Follower 실시간 추종 (< 20ms)
Phase 6: ✅ Isaac Sim URDF 로드, ROS2 Bridge
Phase 7: ✅ Jetson TensorRT 추론 (< 2ms)
Phase 8: ✅ 전체 시스템 통합 운영
```

### 4.2 시스템 최종 성능 목표

| 항목 | 목표 | 측정 방법 |
|------|------|----------|
| Leader→Follower 지연 | < 20ms | ROS 타임스탬프 비교 |
| 조인트 각도 오차 | < 2° | 엔코더 값 비교 |
| 제어 루프 주기 | 100Hz (10ms) | 오실로스코프/로직 분석기 |
| Isaac Sim 동기화 | 30Hz 실시간 | 시뮬레이션 시간 vs 실제 시간 |
| AI 추론 지연 (Jetson) | < 2ms | CUDA Event |
| 전체 시스템 가동률 | > 99% (연속 1시간) | 시스템 로그 |
| 통신 에러율 | < 0.1% | CRC 에러 카운트 |

## 5. 개발자 가이드

### 5.1 코드 컨벤션

| 언어 | 컨벤션 | 도구 |
|------|--------|------|
| C (STM32) | MISRA-C 2004 준수 | cppcheck |
| C++ (ROS2) | ROS2 C++ Style Guide | ament_lint, clang-format |
| Python | PEP8 | black, flake8 |
| URDF | ROS URDF 표준 | xacro 형식 |

### 5.2 버전 관리

```bash
# Git 브랜치 전략
main        ← 안정화된 릴리스
develop     ← 개발 통합 브랜치
feature/*   ← 개별 기능 (feature/stm32-dynamixel-driver)
bugfix/*    ← 버그 수정
release/*   ← 릴리스 준비

# 커밋 메시지 컨벤션
[PHASE] type: description
예: [STM32] feat: add AX-12A SyncWrite support
예: [ROS] fix: correct joint state timestamp
예: [SIM] docs: update Isaac Sim URDF import guide
```

### 5.3 테스트 명령어 모음

```bash
# === STM32 테스트 ===
# 펌웨어 빌드
cd firmware/stm32_leader
make -j4

# ST-Link 플래싱
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg \
  -c "program build/stm32_leader.bin 0x08000000 reset exit"

# 직렬 모니터
screen /dev/ttyUSB0 115200

# === ROS 테스트 ===
# 워크스페이스 빌드
cd ros_ws
colcon build --symlink-install
source install/setup.bash

# 시스템 실행
ros2 launch leader_follower_bringup leader_follower_system.launch.py

# === Isaac Sim ===
cd simulation/isaac_sim
./python.sh python_scripts/leader_follower_sim.py

# === Jetson ===
cd inference/jetson
python3 model/train.py
python3 model/convert_to_trt.py
ros2 launch jetson_inference inference_node.launch.py
```
