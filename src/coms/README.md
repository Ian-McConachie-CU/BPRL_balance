# BPRL Communications Drivers

All peripheral drivers live in `src/coms/`. Register CAN devices and I2C devices in `main.cpp` before calling `threads_start()`.

---

## CAN — FDCAN1 (`CAN.hpp/.cpp`)

**Clock:** PLL2Q = 80 MHz → 1 Mbit/s (BRP=5, TSEG1=13, TSEG2=2, 87.5% sample point)

Classical CAN, standard 11-bit IDs, RxFIFO0 accept-all filter.

### Registered devices

| CAN ID | Device | Content | Rate |
|---|---|---|---|
| `0x01` | IMX5 quaternion | NED→Body [W,X,Y,Z], 4×int16 ÷ 10000 | 200 Hz |
| `0x02` | IMX5 p + ax | int16 ÷ 1000 (rad/s), int16 ÷ 100 (m/s²) | 100 Hz |
| `0x03` | IMX5 q + ay | same encoding | 100 Hz |
| `0x04` | IMX5 r + az | same encoding | 100 Hz |
| `0x69` | Strain rate sensor | 4 signed int16 values, one per arm (FR/RL/FL/RR) | 100 Hz (in development) |

### Adding a device

```cpp
// in main.cpp, before threads_start():
bprl_can_register(0x10, my_callback, nullptr);
```

`CANThread` blocks on `canReceiveTimeout` and calls `can_dispatch()` on each frame, routing it to the registered callback in O(n) time.

---

## DShot — Bidirectional DShot 600 (`PWM.hpp/.cpp`)

Motor output is selected at compile time via `MOTOR_PROTOCOL` in `PWM.hpp`:

```cpp
#define MOTOR_PROTO_DSHOT  0   // bidirectional DShot 600 (default)
#define MOTOR_PROTO_PWM    1   // standard servo PWM
```

`motor_output_write(val[4])` accepts normalized commands: **0 = disarm**, **1–1000 = 0.1–100% throttle**.

### Motor pin mapping

| Motor | Position | Pin | Timer / Channel |
|---|---|---|---|
| 0 | FR | PE11 | TIM1 CH2 |
| 1 | RL | PE9  | TIM1 CH1 |
| 2 | FL | PD13 | TIM4 CH2 |
| 3 | RR | PE13 | TIM1 CH3 |

### TX (burst DMA, 400 Hz)

- TIM1: burst DMA via `TIM1_UP→DMAR`, DCR burst of 4 CCRs covers CH1/CH2/CH3 simultaneously
- TIM4: burst DMA via `TIM4_UP→DMAR`, DBL=0 (single CCR, CH2 only)
- Bit period: 1.67 µs (DShot 600); each bit is encoded as a high pulse of 625 ns (0) or 1250 ns (1)

### RX — GCR telemetry (bidirectional)

Each ESC returns an eRPM frame on the same wire after the TX burst ends:

- TIM1: input-capture DMA rotates across CH1/CH2/CH3 at ~133 Hz each motor
- TIM4: input-capture DMA captures TIM4/CH2 every frame (400 Hz)
- GCR decode: 21-bit transition-marking → 4×5-bit symbols → 16-bit frame → eRPM = 60,000,000 / period_µs

### DShot value mapping

| `val` | DShot command | Effect |
|---|---|---|
| 0 | 0 | Disarm / stop |
| 1 | 48 | Minimum throttle |
| 1000 | 2047 | Full throttle |

---

## SPI (`SPI.hpp/.cpp`)

Two SPI buses drive all three on-board IMUs.

| Bus | Peripheral | CS pin | Device | Role |
|---|---|---|---|---|
| SPI1 | SPID1 | PC2  | ICM-20948 | Primary IMU (imu[0]) |
| SPI4 | SPID4 | PE4  | ICM-20948 | External IMU (imu[1]) |
| SPI4 | SPID4 | PC13 | ICM-20602 | Backup IMU (imu[2]) |

`spi_drv_init()` must run inside `SPIThread` (power-on reset sequences use `chThdSleepMilliseconds`). imu[1] and imu[2] share SPI4 with `spiAcquireBus`/`spiReleaseBus` for mutual exclusion.

SPI clock: 1 MHz for init, 8 MHz for burst reads. Both ICM drivers use 32-byte aligned DMA buffers with `cacheBufferFlush`/`cacheBufferInvalidate` for H7 D-cache coherency.

---

## I2C (`I2C.hpp/.cpp`)

**Status: stub.** Registration table and `i2c_poll_all()` dispatch loop are implemented; the peripheral is not started.

**To enable:** set `HAL_USE_I2C TRUE` in `cfg/halconf.h`, configure `I2CD1` in `cfg/mcuconf.h`, call `i2cStart()` inside `i2c_drv_init()`.

Planned use: strain gauge amplifiers.

---

## PWM / Radio (`Radio.hpp/.cpp`)

CRSF receiver input. `radio_thr()`, `radio_roll()`, `radio_pitch()`, `radio_yaw()` return normalized RC channel values. `radio_armed()` returns the arm switch state.

`motor_output_write()` selects DShot 600 or standard servo PWM (1000–2000 µs) based on the `MOTOR_PROTOCOL` define in `PWM.hpp`. The MotorMixer and ControlThread always produce 0–1000 normalized commands and are unaffected by this choice.
