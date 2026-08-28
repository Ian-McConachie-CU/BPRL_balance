# BPRL_Balance

ChibiOS RTOS firmware for a 5-bar linkage wheeled biped balancing robot, running on the [CubePilot](https://docs.cubepilot.org) CubeOrange+ (STM32H743ZI at 400 MHz).

Forked from BPRL_flight; drone-specific code removed and replaced with dual-bus CAN motor control and SBUS radio input.

---

## TODO

- Implement balance controller (LQR / whole-body) in `BalanceController.cpp`.
- Add EKF states for leg joint angles and rates.
- Register IMU and current sensor callbacks on FDCAN2 (CAN bus 2).
- Implement SDC102 CAN protocol for Steadywin GIM6010-6 wheel motors.
- Add firmware-side USB command handlers for `MOTOR,torque` and `MOTOR,velocity`.
- IMX5 yaw magnetometer / heading reference integration.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware](#2-hardware)
3. [State Estimation (EKF)](#3-state-estimation-ekf)
4. [Controllers](#4-controllers)
5. [CAN Motor Interface](#5-can-motor-interface)
6. [SD Card Logging](#6-sd-card-logging)
7. [Build and Upload](#7-build-and-upload)
8. [Ground Tools](#8-ground-tools)

---

## 1. Project Overview

### What it does

The firmware runs RTOS threads on STM32H743 that read three onboard SPI IMUs and one external CAN IMU (Inertial Sense IMX5), fuse the data into a 19-state EKF estimate, run the robot state machine and balance controller, and send torque commands to six CAN motors. A logging thread records state and sensor data to an SD card in ArduPilot DataFlash binary format.

### Directory layout

```
BPRL_balance/
├── main.cpp                  Entry point — hardware init, CAN motor registration, threads_start()
├── Makefile
│
├── src/
│   ├── RobotState.hpp        Shared index enums: StateIdx (19 states), InputIdx
│   ├── threads.hpp           Shared state (g_state, g_imu, g_motor_torques, …), ThreadRates
│   ├── threads.cpp           All thread bodies + global state definitions
│   │
│   ├── coms/                 Peripheral drivers
│   │   ├── SPI.hpp/.cpp      SPI bus init, ICM-20948 / ICM-45686 drivers
│   │   ├── CAN.hpp/.cpp      FDCAN1 + FDCAN2 dual-bus driver, device callback table
│   │   ├── CANMotor.hpp/.cpp 6-motor CAN abstraction (RMD + SDC102 protocols)
│   │   ├── Radio.hpp/.cpp    SBUS radio input (USART6, SBUSo port)
│   │   ├── SBUS.hpp/.cpp     100000 baud 8E2 parser for 16-channel SBUS frames
│   │   ├── ICM20948.hpp/.cpp InvenSense ICM-20948 9-DOF driver
│   │   └── ICM45686.hpp/.cpp TDK ICM-45686 6-DOF driver
│   │
│   ├── controllers/          Robot control algorithms
│   │   ├── PID.hpp/.cpp                 Cascade PID with derivative filter + anti-windup
│   │   ├── RobotStateMachine.hpp/.cpp   Mode dispatcher: IDLE / BALANCING / MANUAL (400 Hz)
│   │   └── BalanceController.hpp/.cpp   Balance controller stub — TODO: implement LQR/WBC
│   │
│   ├── sensors/
│   │   └── StrainRate.hpp/.cpp   4-channel strain rate sensor (CAN bus 1, ID 0x69)
│   │
│   ├── state_estimator/      3-lane EKF
│   │   ├── EKF.hpp/.cpp      16-state IMU-driven filter (pos, vel, quat, accel/gyro bias)
│   │   └── StateManager.hpp/.cpp  Lane manager — 3 IMU lanes + IMX5 output blending
│   │
│   ├── logging/
│   │   ├── Logger.hpp/.cpp       SD card writer (ArduPilot DataFlash binary format)
│   │   └── LogMessages.hpp       Packed log record structs and kLogDefs[] table
│   │
│   ├── math/
│   │   └── math.hpp/.cpp         Quaternion math, rotation utilities
│   │
│   └── usb_serial.hpp/.cpp   USB CDC-ACM init (VID:PID 0483:5740)
│
├── cfg/
│   ├── halconf.h             ChibiOS HAL peripheral enables
│   └── mcuconf.h             STM32H7 peripheral clock and driver settings
│
├── boards/CubeOrangePlus/    Board-specific pin definitions (STM32H743ZI)
│
└── tools/                    Ground-station Python scripts (see tools/README.md)
```

---

## 2. Hardware

| Item | Detail |
|---|---|
| **MCU** | STM32H743ZI — 400 MHz, 2 MB flash, 1 MB SRAM |
| **Board** | CubePilot CubeOrange+ |
| **CAN bus 1 (FDCAN1)** | 6 motors + strain sensor + IMX5 INS |
| **CAN bus 2 (FDCAN2)** | CAN IMU + current sensor (TBD) |
| **Radio** | SBUS on SBUSo port (USART6/PC7, 100000 baud 8E2) |
| **Logging** | microSD via SDMMC1 (SPI1/SPI4 for IMUs) |
| **Debug** | USB CDC-ACM (`/dev/ttyACMx`) |

### Motor layout (all on CAN bus 1)

| CAN ID | Motor | Protocol |
|---|---|---|
| 1 | Hip FL — LKMTECH MG8016E-i6 | RMD / MyActuator |
| 2 | Hip FR — LKMTECH MG8016E-i6 | RMD / MyActuator |
| 3 | Hip RL — LKMTECH MG8016E-i6 | RMD / MyActuator |
| 4 | Hip RR — LKMTECH MG8016E-i6 | RMD / MyActuator |
| 5 | Wheel L — Steadywin GIM6010-6 (GDS6/SDC102) | SDC102 (stub) |
| 6 | Wheel R — Steadywin GIM6010-6 (GDS6/SDC102) | SDC102 (stub) |

RMD command frame: SID `0x140 + id`, response SID `0x240 + id`, 8 bytes, command byte `0xA1` = torque.

---

## 3. State Estimation (EKF)

Three parallel 16-state EKF lanes (one per onboard IMU) each estimate:
- NED position (3)
- body-frame velocity (3)
- attitude quaternion NED→Body (4)
- accelerometer bias (3)
- gyroscope bias (3)

StateManager selects the primary lane based on innovation residuals and blends with the Inertial Sense IMX5 CAN INS when available.

The 19 output states (`g_state[]`) add body-frame translational acceleration, angular velocity, and angular acceleration derived from the primary lane.

Future: joint angles and joint rates will be appended beyond index 18 once leg kinematics are added.

---

## 4. Controllers

### RobotStateMachine (400 Hz)

| Mode | Trigger | Action |
|---|---|---|
| `ROBOT_IDLE` | Disarmed or `MODE_SW < 0` | Zero torque on all motors |
| `ROBOT_BALANCING` | Armed and `MODE_SW ≥ 0` | Calls BalanceController |
| `ROBOT_MANUAL` | Reserved | Not yet implemented |

### BalanceController

Stub — outputs zero torque. Implement LQR or whole-body controller using:
- `state[StateIdx::Q0..Q3]` — attitude quaternion
- `state[StateIdx::P, Q, R]` — body angular rates
- `input[InputIdx::ROLL_TGT]` — lean setpoint
- `input[InputIdx::THRUST]` — forward speed demand
- `input[InputIdx::YAW_RATE]` — yaw rate demand

---

## 5. CAN Motor Interface

```cpp
// Register motors in main.cpp (already done):
can_motor_register(id, CAN_MOTOR_RMD);      // hip joints
can_motor_register(id, CAN_MOTOR_SDC102);   // wheels

// Send commands (from ControlThread via RobotStateMachine):
can_motor_set_torque(id, torque_Nm);
can_motor_set_velocity(id, vel_rads);

// Read state (mutex-protected):
CanMotorState s;
can_motor_get_state(id, &s);   // pos_rad, vel_rads, torque_Nm, temp_C, valid

// Register arbitrary CAN callbacks (for IMU/sensors on bus 2):
bprl_can_register(CAN_BUS_2, id, my_callback, context);
```

---

## 6. SD Card Logging

Log files are ArduPilot DataFlash binary format — open with [UAV Log Viewer](https://plot.ardupilot.org).

| Message | Rate | Contents |
|---|---|---|
| ATT | 50 Hz | Roll, pitch, yaw, angular rates and accelerations |
| LIN | 50 Hz | NED position, body velocity and accelerations |
| RCIN | 50 Hz | RC stick inputs, arm state |
| OUTP | 50 Hz | Motor torque commands (first 4 of 6) |

---

## 7. Build and Upload

### Prerequisites

- ARM GCC cross-compiler (`arm-none-eabi-gcc`)
- `make`

### Build

```bash
cd BPRL_balance
make
# Output: build/BPRL_BALANCE.bin  build/BPRL_BALANCE.hex  build/BPRL_BALANCE.elf
```

Debug build (enables `$TEL` / `$EKFL` / `$IMU` USB telemetry streams):

```bash
make EXTRA_CFLAGS=-DBPRL_DEBUG
```

### Flash

Via USB bootloader (requires ArduPilot bootloader at 0x08000000):

```bash
python3 tools/flash_upload.py build/BPRL_BALANCE.bin
```

Or with `dfu-util`:

```bash
dfu-util -a 0 -s 0x08020000:leave -D build/BPRL_BALANCE.bin
```

---

## 8. Ground Tools

See [`tools/README.md`](tools/README.md) for full documentation.

```bash
pip install pyserial rich

python3 tools/telemetry.py telemetry     # live attitude + IMU dashboard (DEBUG build)
python3 tools/telemetry.py ekf-status    # per-lane EKF angles
python3 tools/motor_test.py motor-test   # CAN motor status
python3 tools/can_tools.py can-status    # FDCAN1/2 register dump
python3 tools/can_tools.py can-scan      # sniff all CAN IDs on bus 1
python3 tools/logs.py logs list          # list SD card logs
python3 tools/logs.py logs download 1    # download log 1
```
