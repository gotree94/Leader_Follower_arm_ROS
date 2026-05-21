# 02. 하드웨어 설계 (Hardware Design)

## 1. 기구 설계 (Mechanical Design)

### 1.1 암(Arm) 구조 개요

Leader와 Follower 암은 동일한 6자유도(6-DOF) 구조를 가집니다. 각 관절은 AX-12A Dynamixel 서보모터로 구동됩니다.

### 1.2 관절 구성 및 링크 길이

```
                    ┌─────────────────────┐
                    │    Joint 6 (Yaw)    │ ← AX-12A ID 6
                    │    ┌─────────────┐  │
                    │    │  Joint 5    │  │ ← AX-12A ID 5 (Pitch)
                    │    │  ┌────────┐ │  │
                    │    │  │ Joint4 │ │  │ ← AX-12A ID 4 (Roll)
                    │    │  └──▒─────┘ │  │
                    │    └─────▒───────┘  │
                    └───────▒────────────┘
                            │ L4 = 80mm
                    ┌───────▒────────────┐
                    │    Joint 3 (Elbow)  │ ← AX-12A ID 3
                    │         ▒           │
                    │         ▒ L3 = 120mm│
                    │    ┌────▒────┐      │
                    │    │ Joint2  │      │ ← AX-12A ID 2 (Shoulder)
                    │    └────▒────┘      │
                    │         ▒           │
                    │         ▒ L2 = 140mm│
                    │    ┌────▒────┐      │
                    │    │ Joint1  │      │ ← AX-12A ID 1 (Waist)
                    │    └────▒────┘      │
                    │         ▒           │
                    │    Base Plate       │
                    └─────────────────────┘
```

**권장 링크 치수:**

| 링크 | 길이 | 설명 |
|------|------|------|
| L1 (Base→J2) | 60mm | 베이스에서 숄더까지 높이 |
| L2 (J2→J3) | 140mm | 상완 (Upper Arm) |
| L3 (J3→J4) | 120mm | 전완 (Forearm) |
| L4 (J4→J6) | 80mm | 손목 (Wrist) |
| L5 (End Effector) | 50mm | 그리퍼/툴 마운트 |

### 1.3 관절 운동 범위

| 관절 | AX-12A 모드 | 소프트웨어 제한 | 물리적 제한 |
|------|------------|----------------|------------|
| J1 (Waist) | Wheel Mode (0~1023) | 0°~300° | 300° (엔드스톱 없음) |
| J2 (Shoulder) | Joint Mode (0~1023) | 15°~165° | 0°~300° |
| J3 (Elbow) | Joint Mode (0~1023) | 15°~165° | 0°~300° |
| J4 (Wrist Roll) | Wheel Mode (0~1023) | 0°~300° | 300° |
| J5 (Wrist Pitch) | Joint Mode (0~1023) | 15°~165° | 0°~300° |
| J6 (Wrist Yaw) | Wheel Mode (0~1023) | 0°~300° | 300° |

**값 변환:** AX-12A raw value (0~1023) ↔ 각도 (0°~300°)
```
Angle(°) = RawValue × (300/1024)
RawValue = Angle(°) × (1024/300)
```

### 1.4 구조 재료

| 부품 | 권장 재료 | 비고 |
|------|----------|------|
| 링크 프레임 | 알루미늄 6061-T6 (5mm) | CNC 가공 또는 레이저 커팅 |
| 관절 브라켓 | 3D 프린팅 (PETG/CF-PETG) | 100% infill |
| 베이스 플레이트 | 알루미늄 10mm 또는 아크릴 15mm | 무게 안정성 확보 |
| AX-12A 마운트 | 3D 프린팅 (PETG) | AX-12A 프레임 혼 패턴 호환 |
| 커플링 | 알루미늄 | AX-12A 출력축 연결 |

### 1.5 AX-12A 마운팅 고려사항

- 각 AX-12A는 출력축이 다음 링크와 수직/수평이 되도록 배치
- AX-12A의 3개 M2.5 나사 구멍을 활용한 고정 브라켓 설계
- Horn(Y) 또는 Horn(B) 부품을 링크 연결에 사용
- 케이블 라우팅을 위한 중앙 홀 또는 외부 덕트 고려
- 각 관절의 무게를 고려하여 하위 관절일수록 보강 필요

```
  AX-12A 프레임 마운트 패턴:
  
        M2.5 × 3 holes (120° spacing)
              │
         ┌────┼────┐
         │    │    │
         │  ○─┼─○  │  ← Horn mounting
         │    │    │
         └────┼────┘
              │
         Output Shaft
```

## 2. 전기/전자 설계 (Electrical Design)

### 2.1 시스템 블록 다이어그램 (Leader Side)

```
                         ┌──────────────────────────────────┐
                         │          SMPS 12V 5A             │
                         └────────┬─────────────────────────┘
                                  │ 12V
                    ┌─────────────┼─────────────────────┐
                    │             │                      │
             ┌──────▼──────┐ ┌───▼────────┐     ┌──────▼──────┐
             │  5V Regulator│ │ E-Stop     │     │ 3.3V Reg    │
             │  (LM2596)    │ │ Relay      │     │ (AMS1117)   │
             └──────┬──────┘ └─────┬──────┘     └──────┬──────┘
                    │              │                    │
                    │       ┌──────▼──────┐            │
                    │       │ E-Stop SW   │            │
                    │       └─────────────┘            │
                    │                                   │
┌───────────────────┼───────────────────────────────────┼──────────────┐
│  AX-12A Daisy Chain                                 │              │
│                                                      │              │
│  ┌──────┐   ┌──────┐   ┌──────┐   ┌──────┐   ┌──────┐   ┌──────┐  │
│  │AX-12A│   │AX-12A│   │AX-12A│   │AX-12A│   │AX-12A│   │AX-12A│  │
│  │ ID 1 │◄──┤ ID 2 │◄──┤ ID 3 │◄──┤ ID 4 │◄──┤ ID 5 │◄──┤ ID 6 │  │
│  │(J1)  │   │(J2)  │   │(J3)  │   │(J4)  │   │(J5)  │   │(J6)  │  │
│  └──────┘   └──────┘   └──────┘   └──────┘   └──────┘   └──────┘  │
│      │          │          │          │          │          │       │
│      └──────────┴──────────┴──────────┴──────────┴──────────┘       │
│                             │ DATA+ DATA- VCC GND                   │
│                             │ 12V                                   │
└─────────────────────────────┼───────────────────────────────────────┘
                              │
                     ┌────────▼────────┐
                     │  DYNAMIXEL      │
                     │  UART to TTL    │
                     │  (Half-Duplex)  │
                     │  TXD RXD        │
                     └────────┬────────┘
                              │ USART1 (PA9-TX, PA10-RX)
                     ┌────────▼────────┐
                     │   STM32F103     │
                     │  (Blue Pill)    │
                     │                 │
                     │ USART2(PA2-TX,PA3-RX)  ← USB-to-TTL
                     └────────┬────────┘
                              │
                     ┌────────▼────────┐
                     │  CP2102 / FT232 │
                     │  USB-to-TTL     │
                     └────────┬────────┘
                              │ USB
                              ▼
                          ROS PC
```

### 2.2 핀 할당 (STM32F103C8T6 - Blue Pill)

| 핀 | 기능 | 연결 대상 | 비고 |
|----|------|----------|------|
| PA9 | USART1_TX | AX-12A Data (via 485) | Dynamixel 통신 |
| PA10 | USART1_RX | AX-12A Data (via 485) | Half-duplex 제어 |
| PA2 | USART2_TX | USB-to-TTL (TX) | ROS 직렬 통신 |
| PA3 | USART2_RX | USB-to-TTL (RX) | ROS 직렬 통신 |
| PA0 | E-Stop Input | E-Stop 스위치 | 외부 인터럽트 |
| PA1 | LED Indicator | 상태 LED | 시스템 상태 표시 |
| PA4 | Current Sense | AX-12A 전류 센싱 (ADC) | 과전류 모니터링 |
| PA5 | Dynamixel TX Enable | 74HC126/GPIO | Half-duplex 방향 제어 |
| PB0 | Boot Mode LED | 부트 상태 LED | |
| PB1 | Error LED | 에러 표시 LED | |
| PB10 | I2C2_SCL | 외부 센서 (선택) | |
| PB11 | I2C2_SDA | 외부 센서 (선택) | |
| PA13 | SWDIO | ST-Link | 디버깅 |
| PA14 | SWCLK | ST-Link | 디버깅 |

### 2.3 AX-12A 배선 (Daisy Chain)

AX-12A는 3핀 커넥터를 통한 Daisy Chain 방식으로 연결됩니다.

```
AX-12A 커넥터 핀맵 (JST B3B-PH):

핀 1 (GND)  ●━━━━━━━━━━━━━━━━  Black (공통 GND)
핀 2 (VDD)  ●━━━━━━━━━━━━━━━━  Red   (12V 전원)
핀 3 (DATA) ●━━━━━━━━━━━━━━━━  Yellow/White (Half-duplex UART)
```

**Daisy Chain 연결 방식:**
```
[STM32] ──→ [AX-12A ID1] ──→ [AX-12A ID2] ──→ ... ──→ [AX-12A ID6]
  TXD        DATA IN           DATA IN                    DATA IN
  RXD        DATA OUT          DATA OUT                   DATA OUT
             (through)         (through)                  (terminated)
```

**종단 처리:** 마지막 AX-12A(ID 6)는 Daisy Chain의 종단입니다. AX-12A 자체에 1kΩ 풀업 저항이 내장되어 있어 별도 종단 저항이 필요하지 않습니다.

### 2.4 Half-Duplex 통신 회로 (Dynamixel)

AX-12A는 Half-duplex UART 통신(TTL Level, 3.3V)을 사용합니다. STM32 USART1을 Half-duplex 모드로 설정하거나, 별도 방향 제어 회로를 구성합니다.

**옵션 A: STM32 Half-duplex 모드 (HW 기반)**

```
STM32 USART1 Half-duplex 설정:
- PA9 (TX)를 Open-Drain 모드로 사용
- PA10 (RX)는 TX와 동일 핀에 연결
- 외부 풀업 저항 4.7kΩ 필요
- 데이터 시트 참조: RM0008 Section 27.5.5
```

**옵션 B: GPIO 방향 제어 (SW 기반)** — 권장

```
STM32 PA9 (TX) ──┬── 1kΩ ──┬── AX-12A DATA
STM32 PA10 (RX) ─┤         │
                 │         │
PA5 (DIR_CTRL) ──┤ 74HC126 │
                 │ (Buffer)│
                 └─────────┘
                GND ─── 4.7kΩ ──┘

동작:
- DIR_CTRL = HIGH → TX 모드 (데이터 송신)
- DIR_CTRL = LOW  → RX 모드 (데이터 수신, 기본값)
```

### 2.5 전원 시스템 설계

| 전원 레일 | 전압 | 최대 전류 | 용도 |
|-----------|------|----------|------|
| VIN | 12V DC | 5A | AX-12A 서보모터 |
| 5V | 5V DC | 2A | USB-to-TTL, 5V 로직 |
| 3.3V | 3.3V DC | 500mA | STM32F103, 센서 |

**전원 분배:**

```
SMPS 12V 5A
  ├── 12V Rail ──┬── E-Stop Relay ──┬── AX-12A ID 1~6 (Daisy Chain)
  │              │                  └── 12V→5V (LM2596) ── USB-to-TTL
  │              └── 12V→3.3V (AMS1117-3.3) ── STM32F103
  └── GND (공통 접지)
```

**E-Stop 회로:**
```
12V ──┬── NC Relay ──┬── AX-12A VDD (Daisy Chain)
      │               │
      │          E-Stop SW
      │               │
      └───────────────┴── GND

정상 상태: Relay CLOSED → AX-12A 전원 ON
E-Stop:   Relay OPEN   → AX-12A 전원 OFF (토크 해제)
```

### 2.6 USB-to-TTL 연결

```
CP2102 / FT232RL 모듈:
  ┌──────────┐          ┌──────────┐
  │ CP2102   │          │ STM32F103│
  │ TXD  ────┼──────────┤ PA3 (RX) │
  │ RXD  ────┼──────────┤ PA2 (TX) │
  │ GND  ────┼──────────┤ GND      │
  │ 5V   ────┼──────────┤ 5V (VCC) │ (선택)
  └──────────┘          └──────────┘
         │
       USB-A
         │
       ROS PC
```

**Baud Rate:** `1000000` (1Mbps) — rosserial/micro-ROS 기본 설정
**참고:** STM32F103은 최대 1.5Mbps UART 지원 (72MHz 클럭 기준)

## 3. 부품 목록 (BOM)

### 3.1 Leader Arm BOM

| # | 품목 | 사양 | 수량 | 예상 단가 | 비고 |
|---|------|------|------|----------|------|
| 1 | AX-12A Dynamixel | ROBOTIS AX-12A | 6 | $25 | 6축 Leader |
| 2 | STM32F103C8T6 | Blue Pill 보드 | 1 | $3 | 제어기 |
| 3 | USB-to-TTL | CP2102 모듈 | 1 | $2 | ROS 직렬 통신 |
| 4 | SMPS 12V 5A | 12V DC 어댑터 | 1 | $10 | 전원 |
| 5 | LM2596 DC-DC | Step-down 12V→5V | 1 | $3 | USB-to-TTL 전원 |
| 6 | AMS1117-3.3 | 3.3V 레귤레이터 | 1 | $1 | STM32 전원 |
| 7 | E-Stop 스위치 | 푸시버튼 타입 | 1 | $3 | 비상정지 |
| 8 | 릴레이 모듈 | 12V 1채널 릴레이 | 1 | $2 | E-Stop |
| 9 | 74HC126 | Quad Buffer IC | 1 | $1 | Half-duplex 제어 |
| 10 | 저항/커패시터 | 0805 SMD | 10 | $1 | 회로 구성 |
| 11 | 알루미늄 프레임 | 6061-T6 5mm | 1set | $50 | CNC 가공 |
| 12 | 3D 프린팅 부품 | PETG | 1set | $20 | 브라켓/마운트 |
| 13 | 볼트/너트/와셔 | M2.5, M3 | 1set | $5 | 체결 |
| 14 | JST 커넥터 | B3B-PH (3핀) | 6 | $3 | AX-12A 연결 |
| 15 | 케이블 | 22AWG 실리콘 | 2m | $5 | 배선 |
| 16 | ST-Link V2 | 프로그래머 | 1 | $3 | 펌웨어 디버깅 |
| **합계 (Leader)** | | | | **~$137** | |

### 3.2 Follower Arm BOM

| # | 품목 | 사양 | 수량 | 예상 단가 |
|---|------|------|------|----------|
| 1~16 | Leader Arm과 동일 | | 1set | ~$137 |

### 3.3 공통/시스템 BOM

| # | 품목 | 사양 | 수량 | 예상 단가 |
|---|------|------|------|----------|
| 1 | ROS PC | Intel NUC / Mini PC (Ubuntu 22.04) | 1 | $300 |
| 2 | USB 허브 | USB 2.0 4포트 | 1 | $5 |
| 3 | Jetson Orin Nano | Developer Kit (8GB) | 1 | $250 |
| 4 | USB Camera (선택) | RGB 웹캠 | 1 | $20 |
| 5 | 추가 케이블/커넥터 | | 1set | $10 |
| **합계 (공통)** | | | | **~$585** |

### 3.4 전체 예상 비용

| 구분 | 비용 |
|------|------|
| Leader Arm | ~$137 |
| Follower Arm | ~$137 |
| 공통/시스템 | ~$585 |
| **총 합계** | **~$859** |

## 4. 기구 설계 상세 고려사항

### 4.1 무게 밸런싱

- 베이스 플레이트는 충분히 무겁게 (최소 2kg) 설계하여 전복 방지
- 알루미늄 프레임 사용으로 상부 무게 최소화
- 각 관절의 모터 부하 계산 필요:
  - J2 (Shoulder)가 가장 큰 부하를 받음 → AX-12A 1.5Nm 토크 한계 확인
  - 전체 암 길이 ~450mm, 예상 무게 ~500g → 최대 토크 ~2.2Nm (오버)
  - **해결 방안:** 링크 길이 단축 또는 스프링 밸런싱 메커니즘 추가
  - 또는 상완/전완을 카본/경량 소재로 제작

**토크 계산 (J2 Shoulder):**
```
τ = m × g × L/2
m = 400g (상부 링크+모터 총 무게)
L = 0.34m (J2에서 엔드 이펙터까지 거리)
g = 9.81 m/s²

τ = 0.4 × 9.81 × 0.17 = 0.667 Nm (팔 수평 시)
여유율: 1.5Nm / 0.667Nm ≈ 2.25x (안전)
```

### 4.2 AX-12A ID 할당 및 설정

Dynamixel Wizard 2.0을 사용하여 각 AX-12A의 ID와 Baud rate를 사전 설정해야 합니다.

| 파라미터 | 값 | 비고 |
|----------|------|------|
| ID 1~6 | 각 관절별 고유 ID | 리더/팔로워 동일 구조 |
| Baud Rate | 1,000,000 bps (1M) | 고속 통신 |
| Return Delay Time | 2 (≈ 4μs) | 응답 지연 최소화 |
| CW/CCW Angle Limit | 0/1023 (Wheel) 또는 제한 | 관절별 설정 |
| Temperature Limit | 75°C | 과열 보호 |
| Voltage Limit | 10V ~ 14.8V | 전압 범위 |

### 4.3 3D 프린팅 부품 설계 가이드

| 부품 | 권장 재료 | infill | 지지대 | 표면처리 |
|------|----------|--------|--------|---------|
| AX-12A 마운트 브라켓 | PETG / CF-PETG | 100% | 필요 | 없음 |
| 링크 커넥터 | PETG | 80% | 불필요 | 사포 #200 |
| 베이스 마운트 | PETG | 80% | 불필요 | 없음 |
| 손목 커플링 | PETG | 100% | 필요 | 없음 |
| 케이블 가이드 | PLA/PETG | 20% | 불필요 | 없음 |

### 4.4 조립 순서

1. **베이스 플레이트 준비** — 베이스에 J1 AX-12A 고정
2. **링크 1-2 조립** — J1 + L1 링크 + J2 AX-12A
3. **상완 조립** — J2 + L2(140mm) + J3 AX-12A
4. **전완 조립** — J3 + L3(120mm) + J4 AX-12A
5. **손목 조립** — J4 + L4(80mm) + J5 AX-12A + J6 AX-12A
6. **엔드 이펙터 장착** — J6에 그리퍼/툴 연결
7. **Daisy Chain 배선** — 각 AX-12A 간 3핀 케이블 연결
8. **전원 배선** — E-Stop 릴레이를 통한 12V 전원 분배
9. **STM32 연결** — STM32 UART1 ↔ AX-12A Daisy Chain
10. **USB-to-TTL 연결** — STM32 UART2 ↔ CP2102 ↔ ROS PC
