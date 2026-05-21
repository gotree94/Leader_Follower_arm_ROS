# 03. STM32 펌웨어 설계 (Firmware Design)

## 1. 개요

STM32F103C8T6 (Blue Pill)은 AX-12A Dynamixel 서보모터를 직접 제어하고 ROS와 직렬 통신을 담당합니다.
Leader와 Follower 각각 별도의 STM32 보드가 사용되며, 펌웨어 구조는 동일하지만 동작 모드가 다릅니다.

## 2. 개발 환경

| 항목 | 내용 |
|------|------|
| MCU | STM32F103C8T6 (Cortex-M3, 72MHz) |
| IDE | STM32CubeIDE (Ver 1.15+) |
| HAL | STM32Cube HAL / LL (Low-Layer) |
| 컴파일러 | ARM GCC (arm-none-eabi-gcc) |
| 디버거 | ST-Link V2 (SWD) |
| OS | No RTOS (Super Loop + Timer Interrupt) — 선택적으로 FreeRTOS 적용 가능 |
| 직렬 프로토콜 | ROS Serial (rosserial) / Micro-ROS |
| Dynamixel SDK | C++ 기반 자체 구현 (Protocol 1.0) |

### 2.1 ROS 통신 방식 선택

| 방식 | 장점 | 단점 | 권장 |
|------|------|------|------|
| **rosserial** (ROS1) | 검증된 안정성, 풍부한 예제 | ROS1 한정, NodeHandle 메모리 제한 | ⭐ ROS1 선택 시 |
| **micro-ROS** (ROS2) | ROS2 네이티브, QoS 지원 | 설정 복잡, Flash 사용량 ↑ | ⭐ ROS2 선택 시 |
| **UART-to-ROS** (수동 파싱) | 자유도 높음, 경량 | 구현 복잡도 높음 | 학습용 |

**권장:** ROS2 Humble + micro-ROS 조합 — Isaac Sim과의 네이티브 호환성 및 장기 지원 측면에서 최적

## 3. 펌웨어 아키텍처

### 3.1 모듈 구성

```
firmware/stm32_leader/
├── Core/
│   ├── Inc/
│   │   ├── main.h                    # 메인 헤더
│   │   ├── dynamixel.h               # AX-12A Protocol 1.0 드라이버
│   │   ├── dynamixel_config.h        # AX-12A 설정 (ID, Baud, Limit)
│   │   ├── joint_manager.h           # 조인트 상태 관리
│   │   ├── serial_protocol.h         # ROS 직렬 프로토콜 인터페이스
│   │   ├── control_loop.h            # 제어 루프 타이밍
│   │   ├── safety.h                  # 안전/에러 처리
│   │   └── calibration.h             # 캘리브레이션
│   ├── Src/
│   │   ├── main.c                    # 메인 루프
│   │   ├── dynamixel.c               # AX-12A 드라이버 구현
│   │   ├── dynamixel_config.c        # AX-12A 설정 초기화
│   │   ├── joint_manager.c           # 조인트 상태 관리
│   │   ├── serial_protocol.c         # 직렬 프로토콜
│   │   ├── control_loop.c            # 제어 루프
│   │   ├── safety.c                  # 안전/에러 처리
│   │   ├── calibration.c             # 캘리브레이션
│   │   ├── stm32f1xx_it.c            # 인터럽트 핸들러
│   │   ├── stm32f1xx_hal_msp.c       # HAL MSP 초기화
│   │   └── system_stm32f1xx.c        # 시스템 클럭 설정
│   └── Startup/
│       └── startup_stm32f103xb.s     # 부트 코드
├── Drivers/
│   └── STM32F1xx_HAL_Driver/        # STM32 HAL 라이브러리
├── CMakeLists.txt                    # CMake 빌드 (선택: STM32CubeIDE)
└── README.md
```

### 3.2 소프트웨어 계층 구조

```
┌──────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER                      │
│  ┌──────────┐  ┌────────────┐  ┌────────┐  ┌─────────┐  │
│  │Joint Mgr │  │Control Loop│  │Safety  │  │Calibrate│  │
│  └─────┬────┘  └─────┬──────┘  └───┬────┘  └────┬────┘  │
├────────┼─────────────┼─────────────┼─────────────┼───────┤
│                  COMMUNICATION LAYER                      │
│  ┌──────────────────┐  ┌──────────────────────────┐     │
│  │ Dynamixel Driver │  │ Serial Protocol (microROS)│     │
│  │ (Protocol 1.0)   │  │ (rosserial/micro-ROS)     │     │
│  └────────┬─────────┘  └───────────┬──────────────┘     │
├───────────┼────────────────────────┼────────────────────┤
│                  HARDWARE ABSTRACTION LAYER              │
│  ┌──────────────────────────────────────────────────┐   │
│  │              STM32 HAL / LL Drivers              │   │
│  │  USART1  │  USART2  │  TIM  │  GPIO  │  ADC  │  │   │
│  └──────────────────────────────────────────────────┘   │
├──────────────────────────────────────────────────────────┤
│                    HARDWARE LAYER                         │
│  AX-12A (USART1)    USB-to-TTL (USART2)    E-Stop (GPIO) │
└──────────────────────────────────────────────────────────┘
```

## 4. AX-12A Dynamixel Protocol 1.0 드라이버

### 4.1 Protocol 1.0 패킷 구조

```
Instruction Packet (Host → AX-12A):
┌────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│ 0xFF   │ 0xFF   │ ID     │ Length │Inst/CMD│ PARAM  │  CRC   │
│ (Header)│(Header)│(0~253) │(N+2)   │        │ (N)    │(Checksum)│
└────────┴────────┴────────┴────────┴────────┴────────┴────────┘

Status Packet (AX-12A → Host):
┌────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│ 0xFF   │ 0xFF   │ ID     │ Length │ Error  │ PARAM  │  CRC   │
│ (Header)│(Header)│(0~253) │(N+2)   │        │ (N)    │(Checksum)│
└────────┴────────┴────────┴────────┴────────┴────────┴────────┘

Checksum = ~(ID + Length + Inst + Param1 + ... + ParamN) & 0xFF
```

### 4.2 주요 명령어

| 명령 | Instruction | 설명 | 예제 |
|------|-----------|------|------|
| Ping | 0x01 | 존재 확인 | ID=1 → 응답 확인 |
| Read Data | 0x02 | 레지스터 읽기 | Read(1, 0x24, 2) → Present Position |
| Write Data | 0x03 | 레지스터 쓰기 | Write(1, 0x1E, 512) → Goal Position |
| Reg Write | 0x04 | 레지스터 예약 쓰기 | 여러 모터 동기화 |
| Action | 0x05 | Reg Write 실행 | 일괄 실행 |
| Reset | 0x06 | 공장 초기화 | ID/Baud 초기화 |
| Sync Write | 0x83 | 동기 쓰기 (확장) | 다중 모터 동시 제어 |

### 4.3 주요 제어 테이블 (Control Table)

| 주소 | 크기 | 이름 | 설명 | 읽기/쓰기 |
|------|------|------|------|-----------|
| 0x00 | 2 | Model Number | 모델 번호 (AX-12A = 0x0C) | R |
| 0x02 | 1 | Firmware Version | 펌웨어 버전 | R |
| 0x03 | 1 | ID | 모터 ID (1~253) | RW |
| 0x04 | 1 | Baud Rate | 통신 속도 (1: 1Mbps) | RW |
| 0x05 | 1 | Return Delay Time | 응답 지연 (2μs 단위) | RW |
| 0x06 | 2 | CW Angle Limit | 시계방향 각도 제한 | RW |
| 0x08 | 2 | CCW Angle Limit | 반시계방향 각도 제한 | RW |
| 0x0B | 1 | Temperature Limit | 온도 제한 | RW |
| 0x0C | 1 | Min Voltage Limit | 최소 전압 | RW |
| 0x0D | 1 | Max Voltage Limit | 최대 전압 | RW |
| 0x0E | 2 | Max Torque | 최대 토크 | RW |
| 0x10 | 1 | Status Return Level | 응답 레벨 | RW |
| 0x11 | 1 | Alarm LED | 알람 LED 설정 | RW |
| 0x12 | 1 | Alarm Shutdown | 알람 셧다운 설정 | RW |
| **0x1E** | **2** | **Goal Position** | **목표 위치 (0~1023)** | **RW** |
| **0x20** | **2** | **Moving Speed** | **이동 속도 (0~1023)** | **RW** |
| 0x22 | 2 | Torque Limit | 토크 한계 | RW |
| **0x24** | **2** | **Present Position** | **현재 위치 (0~1023)** | **R** |
| **0x26** | **2** | **Present Speed** | **현재 속도** | **R** |
| **0x28** | **2** | **Present Load** | **현재 부하** | **R** |
| 0x2A | 1 | Present Voltage | 현재 전압 | R |
| 0x2B | 1 | Present Temperature | 현재 온도 | R |
| 0x2C | 1 | Registered Instruction | 등록된 명령어 | R |
| 0x2E | 1 | Moving | 이동 중 여부 | R |
| 0x30 | 1 | Lock | EEPROM 락 | RW |
| 0x32 | 2 | Punch | 펀치 (최소 토크) | RW |

### 4.4 드라이버 API 설계

```c
// dynamixel.h
#ifndef __DYNAMIXEL_H
#define __DYNAMIXEL_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define DX_MAX_MOTORS           6
#define DX_BROADCAST_ID         0xFE
#define DX_HEADER_1             0xFF
#define DX_HEADER_2             0xFF
#define DX_DEFAULT_TIMEOUT      100  // ms

// Protocol 1.0 Instructions
typedef enum {
    DX_INST_PING        = 0x01,
    DX_INST_READ        = 0x02,
    DX_INST_WRITE       = 0x03,
    DX_INST_REG_WRITE   = 0x04,
    DX_INST_ACTION      = 0x05,
    DX_INST_RESET       = 0x06,
    DX_INST_SYNC_WRITE  = 0x83,
} DynamixelInstruction;

// Control Table Addresses
typedef enum {
    DX_ADDR_MODEL_NUMBER       = 0x00,
    DX_ADDR_FIRMWARE_VERSION   = 0x02,
    DX_ADDR_ID                 = 0x03,
    DX_ADDR_BAUD_RATE          = 0x04,
    DX_ADDR_RETURN_DELAY_TIME  = 0x05,
    DX_ADDR_CW_ANGLE_LIMIT     = 0x06,
    DX_ADDR_CCW_ANGLE_LIMIT    = 0x08,
    DX_ADDR_GOAL_POSITION      = 0x1E,
    DX_ADDR_MOVING_SPEED       = 0x20,
    DX_ADDR_TORQUE_LIMIT       = 0x22,
    DX_ADDR_PRESENT_POSITION   = 0x24,
    DX_ADDR_PRESENT_SPEED      = 0x26,
    DX_ADDR_PRESENT_LOAD       = 0x28,
    DX_ADDR_PRESENT_VOLTAGE    = 0x2A,
    DX_ADDR_PRESENT_TEMPERATURE= 0x2B,
    DX_ADDR_MOVING             = 0x2E,
    DX_ADDR_TORQUE_ENABLE      = 0x18,
} DynamixelAddress;

// Status Packet Error Flags
typedef enum {
    DX_ERR_NONE         = 0x00,
    DX_ERR_INPUT_VOLTAGE = 0x01,
    DX_ERR_ANGLE_LIMIT  = 0x02,
    DX_ERR_OVERHEATING  = 0x04,
    DX_ERR_RANGE        = 0x08,
    DX_ERR_CHECKSUM     = 0x10,
    DX_ERR_OVERLOAD     = 0x20,
    DX_ERR_INSTRUCTION  = 0x40,
} DynamixelError;

// Dynamixel Status
typedef struct {
    uint8_t  id;
    uint16_t present_position;
    uint16_t present_speed;
    uint16_t present_load;
    uint8_t  present_voltage;
    uint8_t  present_temperature;
    bool     moving;
    uint8_t  last_error;
} DynamixelStatus;

// Driver Functions
void     DX_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *dir_port, uint16_t dir_pin);
void     DX_SetTXMode(void);
void     DX_SetRXMode(void);
bool     DX_Ping(uint8_t id);
bool     DX_Read(uint8_t id, uint16_t address, uint8_t length, uint8_t *data);
bool     DX_Write(uint8_t id, uint16_t address, uint16_t value);
bool     DX_ReadStatus(DynamixelStatus *status);
bool     DX_SyncWrite(uint8_t *ids, uint8_t count, uint16_t address, uint8_t data_size, uint8_t *data);
bool     DX_TorqueEnable(uint8_t id, bool enable);
void     DX_FlushBuffer(void);
uint16_t DX_RawToAngle(uint16_t raw);     // 0~1023 → 0°~300°
uint16_t DX_AngleToRaw(float angle);      // 0°~300° → 0~1023

#endif
```

### 4.5 드라이버 구현 상세

```c
// dynamixel.c (핵심 함수)

#define TX_BUFFER_SIZE  32
#define RX_BUFFER_SIZE  32

static UART_HandleTypeDef *dx_uart;
static GPIO_TypeDef *dx_dir_port;
static uint16_t dx_dir_pin;
static uint8_t tx_buf[TX_BUFFER_SIZE];
static uint8_t rx_buf[RX_BUFFER_SIZE];

void DX_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *dir_port, uint16_t dir_pin) {
    dx_uart = huart;
    dx_dir_port = dir_port;
    dx_dir_pin = dir_pin;
    DX_SetRXMode();  // Default: Receive mode
    HAL_UART_Receive_IT(dx_uart, rx_buf, 1);  // Start interrupt-based receive
}

void DX_SetTXMode(void) {
    HAL_GPIO_WritePin(dx_dir_port, dx_dir_pin, GPIO_PIN_SET);
}

void DX_SetRXMode(void) {
    HAL_GPIO_WritePin(dx_dir_port, dx_dir_pin, GPIO_PIN_RESET);
}

static uint8_t DX_CalculateChecksum(uint8_t *packet, uint8_t length) {
    uint8_t checksum = 0;
    for (int i = 2; i < length - 1; i++) {  // Skip headers, include checksum byte
        checksum += packet[i];
    }
    return (~checksum) & 0xFF;
}

bool DX_Write(uint8_t id, uint16_t address, uint16_t value) {
    uint8_t param[3];
    param[0] = (uint8_t)(address & 0xFF);       // Address low byte
    param[1] = (uint8_t)(value & 0xFF);          // Value low byte
    param[2] = (uint8_t)((value >> 8) & 0xFF);   // Value high byte

    uint8_t length = 5;  // Instruction(1) + Address(1) + Data(2) + Checksum(1)
    uint8_t packet[8];
    packet[0] = DX_HEADER_1;
    packet[1] = DX_HEADER_2;
    packet[2] = id;
    packet[3] = length;
    packet[4] = DX_INST_WRITE;
    packet[5] = param[0];
    packet[6] = param[1];
    packet[7] = DX_CalculateChecksum(packet, 8);

    // 전송
    DX_SetTXMode();
    HAL_Delay(1);  // direction switching delay
    HAL_UART_Transmit(dx_uart, packet, 8, 100);
    HAL_Delay(1);
    DX_SetRXMode();

    // 응답 대기 (Status Packet)
    uint8_t resp[6];
    if (HAL_UART_Receive(dx_uart, resp, 6, DX_DEFAULT_TIMEOUT) == HAL_OK) {
        if (resp[4] == DX_ERR_NONE) return true;
    }
    return false;
}

bool DX_Read(uint8_t id, uint16_t address, uint8_t length, uint8_t *data) {
    uint8_t packet[8];
    packet[0] = DX_HEADER_1;
    packet[1] = DX_HEADER_2;
    packet[2] = id;
    packet[3] = 4;  // Instruction(1) + Address(1) + Length(1) + Checksum(1)
    packet[4] = DX_INST_READ;
    packet[5] = (uint8_t)(address & 0xFF);
    packet[6] = length;
    packet[7] = DX_CalculateChecksum(packet, 8);

    DX_SetTXMode();
    HAL_Delay(1);
    HAL_UART_Transmit(dx_uart, packet, 8, 100);
    HAL_Delay(1);
    DX_SetRXMode();

    // 응답: Header(2) + ID(1) + Length(1) + Error(1) + Data(n) + Checksum(1)
    uint8_t resp_len = length + 6;
    uint8_t resp[16];
    if (HAL_UART_Receive(dx_uart, resp, resp_len, DX_DEFAULT_TIMEOUT) == HAL_OK) {
        if (resp[4] == DX_ERR_NONE) {
            for (int i = 0; i < length; i++) {
                data[i] = resp[5 + i];
            }
            return true;
        }
    }
    return false;
}
```

## 5. 제어 루프 설계 (Leader)

### 5.1 Leader 모드 제어 흐름

```
Main Loop (Super Loop):
┌────────────────────────────────────────────┐
│  초기화 ()                                  │
│  ├── HAL_Init()                             │
│  ├── SystemClock_Config()  (72MHz)          │
│  ├── MX_USART1_UART_Init() (AX-12A: 1Mbps) │
│  ├── MX_USART2_UART_Init() (ROS: 1Mbps)    │
│  ├── MX_GPIO_Init()                        │
│  ├── MX_TIM_Init() (1ms period)            │
│  ├── DX_Init()                             │
│  ├── ROS_Init()                            │
│  └── JointManager_Init()                   │
└────────────────────────────────────────────┘
                      │
           ┌──────────▼──────────┐
           │    TIM Interrupt    │ ← 1ms 주기
           │    (10kHz→1ms)      │
           └──────────┬──────────┘
                      │
           ┌──────────▼──────────┐
           │  Control Loop Step  │
           │  (control_loop.c)   │
           │                     │
           │  매 1ms 실행:       │
           │  1. Safety Check    │
           │  2. AX-12A Polling  │ ← 10ms마다 (100Hz)
           │  3. Joint State 업데이트│
           │  4. ROS Publish     │ ← 10ms마다 (100Hz)
           │  5. ROS Subscribe   │ ← 10ms마다 (100Hz)
           │  6. Watchdog Reset  │
           └──────────┬──────────┘
                      │
           ┌──────────▼──────────┐
           │   rosserial spin()  │ ← 메인 루프에서 호출
           │   또는               │
           │   Micro-ROS spin()  │
           └─────────────────────┘
```

### 5.2 타이밍 다이어그램

```
TIM Interrupt (1ms):
│ Tick 0│ Tick 1│ Tick 2│ ... │ Tick 9│ Tick 10│ ... │ Tick 99│ Tick 100
├───────┼───────┼───────┼─────┼───────┼────────┼─────┼────────┼────────┤
│       │       │       │     │       │        │     │        │        │
│ Safety│ Safety│ Safety│ ... │ Safety│ Safety │ ... │ Safety │ Safety │
│ Check │ Check │ Check│     │ Check │ Check  │     │ Check  │ Check  │
│       │       │       │     │       │        │     │        │        │
└───────┴───────┴───────┴─────┴───────┴────────┴─────┴────────┴────────┘
                                  │                         │
                             AX-12A Poll (J1~J6)      ROS Publish
                             Read Position/Speed      /leader_joint_states
```

### 5.3 Follower 모드 제어 흐름

```
Main Loop (Super Loop):
┌──────────────────────────────────────────────┐
│  초기화 (Leader와 동일)                       │
└──────────────────────────────────────────────┘
                      │
           ┌──────────▼──────────┐
           │    TIM Interrupt    │ ← 1ms 주기
           └──────────┬──────────┘
                      │
           ┌──────────▼──────────┐
           │  Control Loop Step  │
           │  (control_loop.c)   │
           │                     │
           │  1. Safety Check    │
           │  2. ROS Subscribe   │ ← /follower_joint_command
           │  3. Command 수신 확인│
           │  4. AX-12A Write    │ ← Goal Position 설정
           │  5. Joint State 업데이트│
           │  6. ROS Publish     │ ← /follower_joint_states
           │  7. Watchdog Reset  │
           └──────────┬──────────┘
                      │
           ┌──────────▼──────────┐
           │   rosserial spin()  │
           └─────────────────────┘
```

## 6. ROS Serial 통신 (micro-ROS)

### 6.1 micro-ROS 설정

micro-ROS는 STM32F103에서 ROS2 노드를 실행할 수 있게 해줍니다.

**필요 구성 요소:**
- micro_ros_stm32cubemx_utils (GitHub: micro-ROS/micro_ros_stm32cubemx_utils)
- uROSNode (MIDDLEWARE Layer)
- rclc (C client library for ROS 2)

**설정 순서:**

1. STM32CubeMX에서 USART2를 micro-ROS용으로 설정
2. micro_ros_stm32cubemx_utils를 프로젝트에 추가
3. 커스텀 UART 트랜스포트 구현
4. ROS2 노드 생성 및 Pub/Sub 설정

### 6.2 micro-ROS Publisher/Subscriber 예제

```c
// serial_protocol.c (Leader Firmware)

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <uros_transport.h>

// ROS 2 객체
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t joint_pub;
static rcl_subscription_t cmd_sub;
static rclc_executor_t executor;
static rcl_timer_t control_timer;

// 메시지 버퍼
static sensor_msgs__msg__JointState joint_msg;
static std_msgs__msg__Float64MultiArray cmd_msg;

// Joint names (6-DOF)
static const char* joint_names[] = {
    "joint_1_waist",
    "joint_2_shoulder",
    "joint_3_elbow",
    "joint_4_wrist_roll",
    "joint_5_wrist_pitch",
    "joint_6_wrist_yaw"
};
#define JOINT_COUNT 6

// micro-ROS UART Transport 구현
bool uros_transport_open(void) {
    // USART2 초기화 확인
    return true;
}

bool uros_transport_close(void) {
    return true;
}

size_t uros_transport_write(uint8_t *buffer, size_t length, int timeout_ms) {
    HAL_StatusTypeDef hal_status = HAL_UART_Transmit(
        &huart2, buffer, length, timeout_ms);
    return (hal_status == HAL_OK) ? length : 0;
}

size_t uros_transport_read(uint8_t *buffer, size_t length, int timeout_ms) {
    HAL_StatusTypeDef hal_status = HAL_UART_Receive(
        &huart2, buffer, length, timeout_ms);
    return (hal_status == HAL_OK) ? length : 0;
}

// micro-ROS 초기화
void ROS_Init(void) {
    // 메모리 할당자 설정
    allocator = rcl_get_default_allocator();

    // rclc 초기화
    rclc_support_init(&support, 0, NULL, &allocator);

    // 노드 생성
    rclc_node_init_default(&node, "leader_arm_controller", "", &support);

    // Publisher 생성: /leader_joint_states (sensor_msgs/JointState)
    rclc_publisher_init_default(
        &joint_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/leader_joint_states");

    // Subscriber 생성: /follower_joint_command
    rclc_subscription_init_default(
        &cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "/follower_joint_command");

    // 메시지 초기화
    joint_msg.name.data = (char**)joint_names;
    joint_msg.name.size = JOINT_COUNT;
    joint_msg.name.capacity = JOINT_COUNT;
    joint_msg.position.data = (double*)calloc(JOINT_COUNT, sizeof(double));
    joint_msg.position.size = JOINT_COUNT;
    joint_msg.position.capacity = JOINT_COUNT;

    // Executor 초기화
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, NULL, ON_ROS_FEEDBACK);

    // Control Timer (10ms period for 100Hz publish)
    rclc_timer_init_default2(
        &control_timer, &support, RCL_MS_TO_NS(10), control_callback, true);
    rclc_executor_add_timer(&executor, &control_timer);
}

// 콜백: 제어 타이머 10ms
void control_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer != NULL) {
        // 현재 조인트 각도 읽기
        for (int i = 0; i < JOINT_COUNT; i++) {
            DynamixelStatus status;
            if (DX_Read(i + 1, DX_ADDR_PRESENT_POSITION, 2, (uint8_t*)&status)) {
                joint_msg.position.data[i] = DX_RawToAngle(status.present_position);
            }
        }
        joint_msg.header.stamp.sec = ...;  // ROS 시간
        joint_msg.header.stamp.nanosec = ...;

        // Publish
        rcl_publish(&joint_pub, &joint_msg, NULL);
    }
}

// 콜백: Follower 명령 수신
void ON_ROS_FEEDBACK(const void *msgin) {
    const std_msgs__msg__Float64MultiArray *cmd =
        (const std_msgs__msg__Float64MultiArray*)msgin;
    if (cmd->data.size >= JOINT_COUNT) {
        for (int i = 0; i < JOINT_COUNT; i++) {
            uint16_t goal = DX_AngleToRaw((float)cmd->data.data[i]);
            DX_Write(i + 1, DX_ADDR_GOAL_POSITION, goal);
        }
    }
}

// 메인 루프에서 호출
void ROS_Spin(void) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
```

## 7. 안전 및 에러 처리

### 7.1 Watchdog 설정

```c
// IWDG (Independent Watchdog) 설정
// 타임아웃: ~1초 (40kHz LSI, PR=4, RLR=625)

void MX_IWDG_Init(void) {
    // IWDG 키 레지스터 언락
    IWDG->KR = 0x5555;
    // 프리스케일러: 4 (0.1ms tick @ 40kHz LSI)
    IWDG->PR = IWDG_PRESCALER_4;
    // 리로드 값: 10000 → ~1초 타임아웃
    IWDG->RLR = 10000;
    // IWDG 시작
    IWDG->KR = 0xCCCC;
}

// 메인 루프에서 주기적으로 리프레시
void IWDG_Refresh(void) {
    IWDG->KR = 0xAAAA;  // 키 값 0xAAAA로 리프레시
}
```

### 7.2 에러 처리 테이블

| 에러 조건 | 감지 방법 | 조치 |
|-----------|----------|------|
| AX-12A 통신 타임아웃 | HAL_UART_Receive 타임아웃 (100ms) | 재시도 3회 → 실패 시 Error flag + ROS `/arm_status` 발행 |
| AX-12A 과전류 | Status Packet Error flag (Bit 5) | 즉시 Torque OFF, 에러 LED 점등 |
| AX-12A 과열 | Status Packet Error flag (Bit 2) | 냉각 대기 (온도 하강 시 자동 복구) |
| STM32 Watchdog 리셋 | IWDG 리셟 감지 | 시스템 재시작, 원인 로깅 |
| E-Stop 활성화 | PA0 외부 인터럽트 | 즉시 모든 AX-12A 토크 OFF, 모든 모터 정지 |
| ROS 연결 끊김 | micro-ROS 가드 조건 카운터 | 1초 이상 미수신 시 안전 정지 |
| 전압 강하 | AX-12A Present Voltage 모니터링 | 10V 이하 시 경고 → 9.5V 이하 시 정지 |

### 7.3 E-Stop 인터럽트 핸들러

```c
// EXTI0_IRQHandler (PA0, E-Stop)
void EXTI0_IRQHandler(void) {
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET) {
        // E-Stop 활성화 → 모든 AX-12A 토크 OFF
        for (int i = 1; i <= 6; i++) {
            DX_Write(i, DX_ADDR_TORQUE_ENABLE, 0);  // Torque Disable
        }
        // 상태 LED → RED
        HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_SET);

        // 에러 플래그 설정
        system_error = SYS_ERROR_ESTOP;
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    }
}
```

## 8. 캘리브레이션

### 8.1 조인트 캘리브레이션 절차

1. **초기화**: 모든 AX-12A ID 1~6 Torque OFF
2. **수동 조정**: 각 관절을 물리적 0° 위치로 수동 회전
3. **현재 값 기록**: 각 AX-12A의 Present Position 읽기 → Offset 저장
4. **오프셋 적용**: 모든 후속 Angle 변환에 Offset 적용
5. **범위 확인**: 각 관절이 소프트웨어 제한 범위 내에서 동작하는지 확인

### 8.2 캘리브레이션 데이터 구조

```c
typedef struct {
    uint8_t  id;              // AX-12A ID (1~6)
    uint16_t raw_offset;      // Raw value at physical 0°
    uint16_t raw_min;         // Minimum raw value
    uint16_t raw_max;         // Maximum raw value
    float    angle_min;       // Minimum angle (degrees)
    float    angle_max;       // Maximum angle (degrees)
    float    current_angle;   // Current angle (degrees, with offset)
} CalibrationData;

CalibrationData calib_data[JOINT_COUNT] = {
    {1, 512, 0, 1023, -150.0, 150.0},   // J1 Waist
    {2, 512, 150, 850, -105.0, 105.0},  // J2 Shoulder
    {3, 512, 150, 850, -105.0, 105.0},  // J3 Elbow
    {4, 512, 0, 1023, -150.0, 150.0},   // J4 Wrist Roll
    {5, 512, 200, 800, -90.0, 90.0},    // J5 Wrist Pitch
    {6, 512, 0, 1023, -150.0, 150.0},   // J6 Wrist Yaw
};

// Offsets are stored in EEPROM emulation (STM32 Flash last page)
void Calibration_SaveToEEPROM(void);
void Calibration_LoadFromEEPROM(void);
```

## 9. 빌드 및 디버깅

### 9.1 STM32CubeIDE 프로젝트 설정

| 파라미터 | 설정값 | 비고 |
|----------|--------|------|
| MCU | STM32F103C8T6 | 64KB Flash, 20KB RAM |
| 클럭 소스 | HSE 8MHz → PLL → 72MHz | SYSCLK 72MHz |
| APB1 Prescaler | /2 → 36MHz | TIM2, USART2 |
| APB2 Prescaler | /1 → 72MHz | USART1, GPIO |
| USART1 | 1000000 baud, 8-N-1 | AX-12A 통신 |
| USART2 | 1000000 baud, 8-N-1 | ROS 직렬 통신 |
| TIM2 | 72MHz / 72 = 1MHz → 1ms | 제어 루프 타이머 |
| GPIO Output Speed | High (50MHz) | 특히 USART1 TX |

### 9.2 디버깅 핀

| 핀 | 신호 | 용도 |
|----|------|------|
| PA8 | MCO | 72MHz 클럭 출력 (오실로스코프) |
| PB0 | LOGIC_1 | 로직 분석기 Channel 1 (제어 루프 tick) |
| PB1 | LOGIC_2 | 로직 분석기 Channel 2 (ROS publish) |
| PA1 | STATUS_LED | 시스템 상태 LED (초록/빨강) |

### 9.3 통합 테스트

```bash
# ST-Link를 통한 플래싱
openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg \
  -c "program build/stm32_leader.bin 0x08000000 reset exit"

# 직렬 모니터 (디버그 출력)
screen /dev/ttyUSB0 115200

# ROS 통신 테스트
rostopic echo /leader_joint_states
```
