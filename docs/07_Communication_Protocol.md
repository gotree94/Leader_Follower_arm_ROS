# 07. 통신 프로토콜 정의 (Communication Protocol)

## 1. 개요

본 문서는 Leader_Follower_arm_ROS 시스템의 모든 통신 계층에서 사용되는 프로토콜을 정의합니다.
총 3개의 통신 계층이 존재합니다:

1. **AX-12A Protocol 1.0** (STM32 ↔ AX-12A Dynamixel)
2. **ROS Serial Protocol** (STM32 ↔ ROS PC / Jetson)
3. **ROS2 Message Protocol** (ROS 노드 간)

## 2. AX-12A Protocol 1.0 (Half-Duplex UART)

### 2.1 물리 계층

| 파라미터 | 값 | 비고 |
|----------|------|------|
| 인터페이스 | Half-duplex UART (TTL 3.3V) | STM32 USART1 |
| Baud Rate | 1,000,000 bps (1M) | AX-12A 설정 필요 |
| Data Bits | 8 | |
| Parity | None | |
| Stop Bits | 1 | |
| 흐름 제어 | None | |
| 방향 제어 | GPIO (PA5) | TX=HIGH / RX=LOW |

### 2.2 프레임 구조

**Instruction Packet (호스트 → AX-12A):**

```
Byte 0:      0xFF            (Header 1)
Byte 1:      0xFF            (Header 2)
Byte 2:      ID              (0x00~0xFD: Motor ID, 0xFE: Broadcast)
Byte 3:      Length          (Parameter N + 2)
Byte 4:      Instruction     (Command)
Byte 5..N+4: Parameters      (명령에 따른 파라미터)
Byte N+5:    Checksum        (체크섬)
```

**Status Packet (AX-12A → 호스트):**

```
Byte 0:      0xFF            (Header 1)
Byte 1:      0xFF            (Header 2)
Byte 2:      ID              (응답 모터 ID)
Byte 3:      Length          (Parameter N + 2)
Byte 4:      Error           (에러 플래그)
Byte 5..N+4: Parameters      (데이터 파라미터)
Byte N+5:    Checksum        (체크섬)
```

**체크섬 계산:**
```
Checksum = ~(ID + Length + Instruction + Param1 + ... + ParamN) & 0xFF
         = (비트 NOT) (모든 바이트 합 하위 1바이트)
```

### 2.3 주요 명령어 프레임

**Ping (0x01):**
```
송신: 0xFF 0xFF ID 0x02 0x01 CHECKSUM
수신: 0xFF 0xFF ID 0x02 ERROR CHECKSUM  (정상: ERROR=0x00)
```

**Read Data (0x02):**
```
송신: 0xFF 0xFF ID 0x04 0x02 ADDR_LEN CHECKSUM
                    Length=4 → Inst(1)+Addr(1)+Len(1)+CS(1)
수신: 0xFF 0xFF ID LEN ERROR DATA... CHECKSUM

예: ID=1, Present Position(0x24) 읽기 (2바이트)
송신: FF FF 01 04 02 24 02 D4
수신: FF FF 01 04 00 00 02 F9  (Position = 512 = 0x0200)
```

**Write Data (0x03):**
```
송신: 0xFF 0xFF ID 0x05 0x03 ADDR DATA_L DATA_H CHECKSUM
                    Length=5 → Inst(1)+Addr(1)+Data(2)+CS(1)

예: ID=1, Goal Position(0x1E) = 512 (0x0200)
송신: FF FF 01 05 03 1E 00 02 DB
수신: FF FF 01 02 00 DC          (ACK)
```

**Sync Write (0x83):** — 다중 모터 동시 제어
```
송신: FF FF FE LEN 83 ADDR DATA_LEN ID1 D1... ID2 D1... CHECKSUM

예: ID 1,2,3 동시에 Goal Position 쓰기
송신: FF FF FE 0E 83 1E 02
      01 00 02   → ID=1, Position=512
      02 00 02   → ID=2, Position=512
      03 00 02   → ID=3, Position=512
      CHECKSUM
```

### 2.4 에러 플래그 (Status Packet Byte 4)

| Bit | 플래그 | 설명 |
|-----|--------|------|
| Bit 0 | Input Voltage Error | 전압이 설정 범위를 벗어남 |
| Bit 1 | Angle Limit Error | 목표 각도가 제한 범위를 벗어남 |
| Bit 2 | Overheating Error | 내부 온도가 제한을 초과 |
| Bit 3 | Range Error | 명령 파라미터가 범위를 벗어남 |
| Bit 4 | Checksum Error | 송신된 체크섬이 일치하지 않음 |
| Bit 5 | Overload Error | 최대 토크를 초과하는 부하 |
| Bit 6 | Instruction Error | 정의되지 않은 명령어 |

## 3. ROS Serial Protocol (STM32 ↔ ROS PC)

### 3.1 물리 계층

| 파라미터 | 값 | 비고 |
|----------|------|------|
| 인터페이스 | Full-duplex UART (TTL 3.3V) | STM32 USART2 |
| 변환기 | CP2102 / FT232RL USB-to-TTL | |
| Baud Rate | 1,000,000 bps (1M) | |
| Data Bits | 8 | |
| Parity | None | |
| Stop Bits | 1 | |

### 3.2 커스텀 패킷 포맷 (ROS 노드 ↔ STM32)

rosserial/micro-ROS의 표준 프로토콜 외에도, 커스텀 경량 패킷을 정의하여 사용합니다.

```
Request Packet (ROS PC → STM32):
┌──────┬──────┬──────┬──────┬──────────┬──────┐
│ 0xAA │ 0x55 │ LEN  │ CMD  │ PAYLOAD  │ CRC  │
│ 1B   │ 1B   │ 1B   │ 1B   │ N Bytes  │ 1B   │
└──────┴──────┴──────┴──────┴──────────┴──────┘

Response Packet (STM32 → ROS PC):
┌──────┬──────┬──────┬──────┬──────────┬──────┐
│ 0xAA │ 0x55 │ LEN  │ STAT │ PAYLOAD  │ CRC  │
│ 1B   │ 1B   │ 1B   │ 1B   │ N Bytes  │ 1B   │
└──────┴──────┴──────┴──────┴──────────┴──────┘

CRC = XOR of all bytes from LEN to last PAYLOAD byte
```

### 3.3 명령어 테이블

| CMD | 명령 | 방향 | Payload 길이 | 설명 |
|-----|------|------|-------------|------|
| 0x01 | READ_ALL_POSITIONS | ROS→STM32 | 0 | 모든 조인트 현재 위치 요청 |
| 0x02 | STM32→ROS | 12 | J1~J6 위치 (각 2바이트, Little Endian) |
| 0x03 | WRITE_ALL_POSITIONS | ROS→STM32 | 12 | J1~J6 목표 위치 (각 2바이트) |
| 0x04 | STM32→ROS | 1 | ACK (0x00=OK) |
| 0x05 | SET_TORQUE | ROS→STM32 | 2 | ID(1B) + ON/OFF(1B) |
| 0x06 | STM32→ROS | 2 | ID(1B) + STATUS(1B) |
| 0x07 | GET_STATUS | ROS→STM32 | 0 | 시스템 상태 요청 |
| 0x08 | STM32→ROS | 18 | 전압(1)+온도(1)+에러(1)+로드(6*2)+기타 |
| 0x09 | CALIBRATE | ROS→STM32 | 0 | 캘리브레이션 시작 |
| 0x0A | STM32→ROS | 12 | 캘리브레이션 오프셋 (각 2바이트) |
| 0x0B | HOME_ALL | ROS→STM32 | 0 | 모든 조인트 홈 위치로 이동 |
| 0x0C | STM32→ROS | 1 | ACK |
| 0x0D | SET_MODE | ROS→STM32 | variable | 모드 문자열 전송 |
| 0x0E | STM32→ROS | 1+ | ACK + 모드 확인 |
| 0x0F | DIAG | ROS→STM32 | 0 | 진단 요청 |
| 0x10 | STM32→ROS | var | 진단 정보 |

### 3.4 패킷 예시

**모든 조인트 위치 요청:**
```
요청:  AA 55 01 01 01           → CRC=01⊕01=00
         AA 55 01 01 00         → Header + LEN(1) + CMD(0x01) + CRC

응답:  AA 55 0D 02 [J1L][J1H][J2L][J2H]...[J6L][J6H] CRC
         AA 55 0D 02 00 02 00 02 00 02 00 02 00 02 00 02 D4
         → J1=512(0x0200), J2=512, ... (현재 위치 raw value)
```

**모든 조인트 명령 전송:**
```
명령:  AA 55 0D 03 [J1L][J1H][J2L][J2H]...[J6L][J6H] CRC
         AA 55 0D 03 00 02 00 02 00 02 00 02 00 02 00 02 D4

응답:  AA 55 01 04 00                 → ACK OK
```

## 4. ROS2 메시지 프로토콜 (ROS 노드 간)

### 4.1 표준 메시지

```yaml
# sensor_msgs/JointState (표준)
std_msgs/Header header
string[] name           # 조인트 이름 배열
float64[] position      # 조인트 각도 (radians)
float64[] velocity      # 속도 (rad/s)
float64[] effort        # 토크 (Nm)
```

```yaml
# std_msgs/Float64MultiArray (표준)
std_msgs/MultiArrayLayout layout
float64[] data           # [j1, j2, j3, j4, j5, j6] 각도 (radians)
```

```yaml
# std_msgs/String (표준)
string data              # 모드 문자열: "FOLLOWING", "SIMULATION", "INFERENCE", "CALIBRATION"
```

### 4.2 커스텀 메시지 (leader_follower_msgs)

**JointCommand.msg:**
```yaml
# Leader/Follower 조인트 명령 메시지
std_msgs/Header header
float64[6] joint_angles       # 목표 각도 (radians)
float64[6] joint_velocities   # 목표 속도 (rad/s, 선택)
uint8 mode_flag               # 0: 절대 위치, 1: 상대 위치, 2: 속도 제어
```

**ArmStatus.msg:**
```yaml
# 암 시스템 상태 메시지
std_msgs/Header header
uint8 arm_id                   # 0: Leader, 1: Follower
uint8 system_mode              # 0: IDLE, 1: FOLLOWING, 2: SIMULATION, 3: INFERENCE, 4: CALIBRATION, 5: ERROR
uint8 error_code               # 0: NONE, 1: COMM_TIMEOUT, 2: OVERLOAD, 3: OVERHEAT, 4: VOLTAGE, 5: E_STOP
float32 present_voltage        # 현재 전압 (V)
float32 present_temperature    # 현재 온도 (°C)
uint16[6] raw_positions        # AX-12A Raw 값 (디버깅용)
```

**SetMode.srv:**
```yaml
string mode                    # "FOLLOWING", "SIMULATION", "INFERENCE", "CALIBRATION"
---
bool success
string message
```

**Calibrate.srv:**
```yaml
uint8 joint_id                 # 0: ALL, 1~6: 특정 조인트
---
bool success
float64[6] offsets             # 조인트 오프셋 각도 (radians)
string message
```

**RecordTrajectory.srv:**
```yaml
string filename                # 저장 파일명 (선택, 빈칸=자동 생성)
float32 duration               # 녹화 시간 (초)
---
bool success
string filepath                # 저장 경로
uint32 sample_count            # 녹화된 샘플 수
string message
```

### 4.3 토픽/서비스 매핑

```
Publisher:
  leader_arm_controller  →  /leader_joint_states         (sensor_msgs/JointState, 100Hz)
  follower_arm_controller →  /follower_joint_states       (sensor_msgs/JointState, 100Hz)
  jetson_inference_node   →  /inference_joint_states      (sensor_msgs/JointState, 100Hz)
  leader_follower_sim     →  /leader_joint_states         (Isaac Sim, 30~60Hz)

Subscriber:
  follower_arm_controller ←  /leader_joint_states         (Leader 상태 구독)
  follower_arm_controller ←  /follower_joint_command      (외부 명령 구독)
  leader_arm_controller   ←  /leader_joint_command        (외부 명령 구독)
  isaac_sim_ros2_bridge   ←  /leader_joint_command        (시뮬레이션 명령)

Service Server:
  leader_arm_controller   →  /set_mode                    (SetMode)
  leader_arm_controller   →  /home_arm                    (Trigger)
  leader_arm_controller   →  /calibrate_joints            (Calibrate)
  follower_arm_controller →  /record_trajectory           (RecordTrajectory)
  jetson_inference_node   →  /inference_start             (Trigger)
```

## 5. 데이터 포맷 변환

### 5.1 AX-12A Raw Value ↔ 각도 (Radians)

```python
# 스케일 변환
RAW_MAX = 1023
ANGLE_MAX_DEG = 300.0  # AX-12A 최대 회전각

def raw_to_degrees(raw: int) -> float:
    """AX-12A Raw Value (0~1023) → 각도 (degrees)"""
    return raw * (ANGLE_MAX_DEG / RAW_MAX)

def degrees_to_raw(deg: float) -> int:
    """각도 (degrees) → AX-12A Raw Value (0~1023)"""
    return int(round(deg * (RAW_MAX / ANGLE_MAX_DEG)))

def degrees_to_radians(deg: float) -> float:
    return deg * 3.14159265359 / 180.0

def radians_to_degrees(rad: float) -> float:
    return rad * 180.0 / 3.14159265359

# 전체 변환 체인:
# Raw(0~1023) → Degrees(0~300) → Radians(-2.618~2.618)
# Radians → Degrees → Raw
```

### 5.2 직렬 프레임 직렬화/역직렬화

```c
// STM32 Firmware (C)

// 패킷 직렬화 (STM32 송신용)
void SerializeJointPositions(uint8_t *buffer, uint16_t *raw_positions) {
    buffer[0] = 0xAA;  // Header 1
    buffer[1] = 0x55;  // Header 2
    buffer[2] = 13;    // Length (1 CMD + 12 data + 1 CRC)
    buffer[3] = 0x02;  // Command: READ_ALL_POSITIONS response

    // 6개 조인트 Raw Value (각 2바이트, Little Endian)
    for (int i = 0; i < 6; i++) {
        buffer[4 + i*2]     = raw_positions[i] & 0xFF;        // Low byte
        buffer[5 + i*2]     = (raw_positions[i] >> 8) & 0xFF; // High byte
    }

    // CRC (XOR of all bytes from LEN to last data byte)
    buffer[16] = 0;
    for (int i = 2; i < 16; i++) {
        buffer[16] ^= buffer[i];
    }
}

// 패킷 역직렬화 (ROS PC 수신 처리)
bool DeserializeJointPositions(const uint8_t *buffer, uint16_t *raw_positions) {
    // 헤더 확인
    if (buffer[0] != 0xAA || buffer[1] != 0x55) return false;

    uint8_t len = buffer[2];
    uint8_t cmd = buffer[3];

    if (cmd != 0x02 || len != 13) return false;

    // CRC 검증
    uint8_t crc = 0;
    for (int i = 2; i < 16; i++) crc ^= buffer[i];
    if (crc != 0) return false;  // CRC should be 0 when including CRC byte

    // Raw Value 추출 (Little Endian)
    for (int i = 0; i < 6; i++) {
        raw_positions[i] = buffer[4 + i*2] | (buffer[5 + i*2] << 8);
    }
    return true;
}
```

## 6. 타이밍 및 동기화

### 6.1 통신 타이밍

```
AX-12A Polling Cycle (Leader STM32):
  T0: Send Read Present Position for ID 1
  T1: Receive Status Packet from ID 1
  T2: Send Read Present Position for ID 2
  ...
  T_{10}: Read complete for all 6 joints
  T_{11}: Publish to ROS via Serial
  T_{12}: Wait for next 10ms timer tick

  Total cycle ≈ 6 × (TX_time + 2ms response) + ROS_time
  TX_time at 1Mbps = 8 bytes × 10μs = 80μs
  Total ≈ 6 × 2.1ms + 1ms ≈ 13.6ms (cycle may exceed 10ms)

Solution: Read in interleaved schedule
  Tick 0-4: Read J1, J2
  Tick 5-9: Read J3, J4
  Tick 10-14: Read J5, J6
  → Each joint updated at 50Hz, publish at 100Hz
  → Better: Use bulk read or minimize individual reads
```

### 6.2 타임스탬프 동기화

```
ROS2 노드는 hardware time 대신 ROS2 system time 사용:
  msg.header.stamp = node->now();  // ROS2 시스템 클럭

Isaac Sim과 실제 하드웨어 간 time 동기화:
  - use_sim_time 파라미터로 관리
  - 실제 모드: false (system clock)
  - 시뮬레이션 모드: true (sim clock)
```
