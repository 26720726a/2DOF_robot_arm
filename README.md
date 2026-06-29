# 2DOF Robot Arm

ATmega128 기반의 2-DOF 로봇팔 제어 실험 프로젝트입니다.
PC/MFC 프로그램과 AVR 보드 간의 UART 패킷 통신을 통해 목표 위치, 속도, 전류 명령을 전달하고, 엔코더와 전류 센서 값을 읽어 다시 PC로 송신하는 구조로 작성되었습니다.

This project is an AVR-based embedded control firmware for a 2-DOF robotic arm experiment.
It focuses on packet communication between a PC/MFC interface and an ATmega128 board, sensor feedback acquisition, and a basic framework for future cascade control.

---

## Project Overview

이 프로젝트의 목적은 2자유도 로봇팔의 관절 제어를 위해 필요한 임베디드 제어 구조를 구현하는 것입니다.

주요 기능은 다음과 같습니다.

* PC/MFC 프로그램과 AVR 간 UART 통신
* 패킷 기반 목표값 수신
* LS7366 엔코더 카운터를 이용한 위치 측정
* ADC 기반 전류 센서 값 측정
* PWM 모터 드라이버 초기화
* 현재 위치, 속도, 전류 값을 PC로 송신
* 향후 위치-속도-전류 cascade 제어기 확장을 위한 기본 구조 제공

---

## Key Features

| Feature           | Description                                                   |
| ----------------- | ------------------------------------------------------------- |
| MCU               | ATmega128                                                     |
| Communication     | UART0, 9600 baud                                              |
| Encoder Interface | LS7366 SPI encoder counter                                    |
| Motor Output      | PWM motor driver                                              |
| Current Sensing   | ADC-based current feedback                                    |
| Packet Protocol   | Header, ID, size, mode, position, velocity, current, checksum |
| Control Structure | Sensor feedback loop with future cascade control support      |

---

## Hardware / Software Environment

### Hardware

* ATmega128 MCU board
* 2-DOF robot arm mechanism
* DC motor or actuator for each joint
* Motor driver
* Encoder
* LS7366 encoder counter IC
* Current sensor
* PC for MFC-based monitoring/control interface

### Software

* Microchip Studio / Atmel Studio 7
* AVR-GCC
* C/C++
* UART serial communication
* MFC-based PC control/monitoring program

---

## Repository Structure

```bash
2DOF_robot_arm/
├── robot4_final/
│   ├── main.cpp
│   ├── robot4_final.cppproj
│   └── robot4_final.componentinfo.xml
│
├── dataType/
│   ├── main.cpp
│   ├── dataType.cppproj
│   └── dataType.componentinfo.xml
│
├── robot4_final.atsln
└── README.md
```

---

## Main Code Description

### `robot4_final/main.cpp`

이 파일은 로봇팔 제어용 AVR 메인 펌웨어입니다.

주요 구성은 다음과 같습니다.

```cpp
#define F_CPU 16000000UL
```

ATmega128을 16 MHz 클럭 기준으로 동작하도록 설정합니다.

---

### 1. Global Variables

```cpp
volatile double g_Pdes = 0.0;
volatile double g_Vlimit = 0.0;
volatile double g_Climit = 0.0;

volatile double cur_pos = 0.0;
volatile double cur_vel = 0.0;
volatile double cur_cur = 0.0;
```

PC/MFC에서 전달받는 목표값과 현재 센서 측정값을 저장합니다.

| Variable   | Meaning               |
| ---------- | --------------------- |
| `g_Pdes`   | Desired position      |
| `g_Vlimit` | Velocity limit        |
| `g_Climit` | Current limit         |
| `cur_pos`  | Current position      |
| `cur_vel`  | Current velocity      |
| `cur_cur`  | Current current value |

---

### 2. UART Receive Interrupt

```cpp
ISR(USART0_RX_vect)
{
    g_buf[g_bufWriteCnt++] = UDR0;
}
```

UART0로 수신된 데이터를 ring buffer에 저장합니다.
PC에서 전달된 제어 명령 패킷을 실시간으로 받기 위한 구조입니다.

---

### 3. Initialization

```cpp
void InitIO()
{
    cli();

    UART0_Init(9600);
    LS7366_Init();
    ADC_Init();
    PWM_Init();

    TCCR0 = (1 << CS02) | (1 << CS00);
    TIMSK |= (1 << TOIE0);

    sei();
}
```

초기화 함수에서는 다음 장치를 설정합니다.

* UART0 serial communication
* LS7366 encoder counter
* ADC current sensing
* PWM motor output
* Timer0 overflow interrupt

---

### 4. Packet Receive Parser

```cpp
void ReceivePacket()
```

PC/MFC에서 전송한 패킷을 파싱하는 함수입니다.

패킷 구조는 다음과 같은 흐름으로 처리됩니다.

```text
Header check
→ ID check
→ Data receive
→ Checksum validation
→ Desired position / velocity / current update
```

수신된 값은 다음과 같이 변환됩니다.

```cpp
g_Pdes   = g_PacketBuffer.data.pos / 1000.0;
g_Vlimit = g_PacketBuffer.data.vel / 1000.0;
g_Climit = g_PacketBuffer.data.cur / 1000.0;
```

즉, PC에서 정수형으로 보낸 값을 AVR 내부에서 실수 단위로 변환하여 사용합니다.

---

### 5. Current Data Transmission

```cpp
void SendCurrentData()
```

현재 위치, 속도, 전류 값을 패킷으로 만들어 PC/MFC로 송신합니다.

```cpp
packet.data.pos = cur_pos * 1000.0;
packet.data.vel = cur_vel * 1000.0;
packet.data.cur = cur_cur * 1000.0;
```

PC에서는 이 값을 받아 로봇팔의 현재 상태를 모니터링하거나 그래프로 표시할 수 있습니다.

---

### 6. Control Loop

```cpp
void ControlLoop()
{
    static double pos_prev = 0;

    long enc = LS7366_ReadCounter();
    cur_pos = enc * (2 * 3.141592) / (4000.0);

    cur_vel = (cur_pos - pos_prev) * 1000.0;
    pos_prev = cur_pos;

    cur_cur = ADC_ReadCurrent();

    SendCurrentData();
}
```

현재 제어 루프는 다음 작업을 수행합니다.

1. LS7366에서 엔코더 카운트 읽기
2. 엔코더 값을 각도 위치로 변환
3. 이전 위치와 현재 위치 차이를 이용해 속도 계산
4. ADC에서 전류 센서 값 읽기
5. 현재 상태를 PC로 송신

현재 코드에서는 실제 위치/속도/전류 제어기는 비활성화되어 있으며, 센서 측정 및 데이터 송신 구조가 중심입니다.

---

## System Flow

```text
PC / MFC Program
      ↓ UART Packet
ATmega128
      ↓
ReceivePacket()
      ↓
Desired Position / Velocity / Current Update
      ↓
Sensor Feedback
      ├── LS7366 Encoder
      └── ADC Current Sensor
      ↓
Current Position / Velocity / Current Calculation
      ↓
SendCurrentData()
      ↓ UART Packet
PC / MFC Monitoring
```

---

## Packet Communication

The firmware uses a packet-based communication structure.

General packet flow:

```text
Header
ID
Size
Mode
Position
Velocity
Current
Checksum
```

The checksum is calculated by summing packet bytes except the final checksum field.

```cpp
for (int i = 0; i < packet.data.size - 1; i++)
    packet.data.check += packet.buffer[i];
```

This allows the AVR board to verify whether the received command packet is valid.

---

## Build and Upload

### 1. Open Project

Open the solution file in Atmel Studio or Microchip Studio.

```text
robot4_final.atsln
```

### 2. Select Target MCU

Target device:

```text
ATmega128
```

### 3. Build

Build the `robot4_final` project.

### 4. Upload

Upload the generated `.hex` file to the ATmega128 board using an AVR programmer.

---

## Required Header / Driver Files

The main firmware depends on the following driver headers.

```cpp
#include "uart.h"
#include "LS7366.h"
#include "pwm.h"
#include "adc.h"
#include "dataType.h"
```

These files should contain:

| File         | Role                                       |
| ------------ | ------------------------------------------ |
| `uart.h`     | UART initialization and transmit functions |
| `LS7366.h`   | Encoder counter interface                  |
| `pwm.h`      | PWM motor driver control                   |
| `adc.h`      | ADC current sensor reading                 |
| `dataType.h` | Packet structure definition                |

If these files are not included in the repository, they should be added before building the full firmware.

---

## Current Status

Current implementation includes:

* UART packet receive structure
* Packet checksum verification
* Target position, velocity, current command parsing
* Encoder-based position measurement
* Velocity calculation
* ADC-based current measurement
* Current state packet transmission to PC

Future extension:

* Position control loop
* Velocity control loop
* Current control loop
* 2-DOF coordinated trajectory control
* Inverse kinematics-based end-effector control
* PC GUI visualization and command interface refinement

---

## What I Learned

Through this project, I practiced:

* Embedded C/C++ programming on AVR
* UART packet communication
* Interrupt-based serial data reception
* Encoder-based position feedback
* ADC-based sensor acquisition
* PWM motor control structure
* Basic robot actuator control architecture
* Foundation of cascade control for robotic joints

---

## Notes

This repository is intended as an embedded control project for a 2-DOF robotic arm experiment.
The current code provides the communication and sensing framework, while the full closed-loop controller can be extended based on the existing structure.

---

## Author

Kim Ji Seong
Robotics / Embedded Systems / Robot Arm Control
