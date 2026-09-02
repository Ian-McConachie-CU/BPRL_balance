# BPRL_Balance Tools

Ground-station utilities for the BPRL_Balance robot controller. Connect via USB CDC (`/dev/ttyACMx`, VID:PID `0483:5740`). All scripts auto-detect the port.

## Dependencies

```bash
pip install pyserial rich
```

---

## Script overview

| Script | Subcommands | DEBUG build? |
|---|---|---|
| `telemetry.py` | `telemetry`, `ekf-status`, `imu-compare` | Required |
| `motor_test.py` | `motor-test` | No |
| `radio_test.py` | `rc-status` | No |
| `targets_test.py` | `tgt-status` | No |
| `calibrate.py` | `calibrate` | Required |
| `can_tools.py` | `can-status`, `can-scan` | No |
| `logs.py` | `logs list/download/decode/erase`, `log-status` | No |
| `flash_upload.py` | *(positional firmware path)* | — |

All scripts accept `--port /dev/ttyACMx` and `--baud N` global options.

---

## telemetry.py

> Requires `-DBPRL_DEBUG` firmware build.

Parses the `$TEL`, `$EKFL`, and `$IMU` 10 Hz streams emitted by `DebugThread`.

| Subcommand | Description |
|---|---|
| `telemetry` | Live attitude / rates / IMU status dashboard |
| `ekf-status` | Per-lane EKF roll/pitch/yaw/p/q/r table (three onboard lanes + IMX5 CAN) |
| `imu-compare` | Side-by-side raw accel and gyro from all three onboard IMUs |

```bash
python3 tools/telemetry.py telemetry
python3 tools/telemetry.py ekf-status
python3 tools/telemetry.py imu-compare
```

---

## motor_test.py

> Works on any firmware build.

Interactive CAN motor test. Currently supports reading motor state via the `MOTOR,status` USB command.
Torque and velocity commands are stubbed pending firmware-side USB handlers.

```bash
python3 tools/motor_test.py motor-test
```

**Commands inside the tool:**

| Command | Description |
|---|---|
| `status` | Print position, velocity, torque, and temperature for all 6 CAN motors |
| `quit` | Exit |

Motor IDs: 1–4 = hip joints (LKMTECH MG8016E-i6, RMD protocol), 5–6 = wheels (Steadywin GIM6010-6, SDC102).

---

## radio_test.py

> Works on any firmware build.

Live dump of all 16 raw SBUS channels received on TELEM2 (USART3), via the
`RC,status` USB command — polls the firmware at `--rate` Hz (default 20).
Shows each channel's raw 11-bit value (172–1811, center 992) with a bar
gauge, the receiver's `frame_lost`/`failsafe` flags, and the decoded armed
state. Channels 0–9 are labeled per this robot's channel map (see the main
`README.md` for the full table and firmware wiring); 10–15 show raw values
only.

```bash
python3 tools/radio_test.py rc-status
python3 tools/radio_test.py rc-status --rate 30
```

Use this to confirm a transmitter/receiver pair is bound and wired correctly
before trusting stick/switch input in the balance controller: move each
stick/switch and confirm the corresponding channel tracks it, and check that
`frame_lost`/`failsafe` stay clear with the transmitter on.

---

## targets_test.py

> Works on any firmware build.

Live view of the **processed** system targets — `g_input[]`, `g_armed`, and
`RobotStateMachine::mode()` — via the `TGT,status` USB command, polling at
`--rate` Hz (default 20). Unlike `radio_test.py` (which shows raw SBUS
channel values straight off the wire), this shows what the control pipeline
actually sees after `Radio.cpp`'s normalization/inversion, so it exercises
the full radio-input plumbing end to end: SBUS → `Radio.cpp` accessors →
`g_input[]`/`g_armed` → `RobotStateMachine`.

```bash
python3 tools/targets_test.py tgt-status
```

Shows: armed state, the mode-select switch's raw value alongside the
resulting state-machine mode name, and bar gauges for velocity/yaw/height/
lean targets. `height_tgt`/`lean_tgt` are read but not yet consumed by any
controller, and `CAR` mode is reachable (mode switch low + armed) but
**stubbed** — zero torque, same as `IDLE` — until wheel mixing and the
car/balance transitions are implemented (see the main `README.md`'s planned
state machine section).

---

## calibrate.py

> Requires `-DBPRL_DEBUG` firmware build.

Runs the IMU calibration routine (gyro bias + accel scale) and writes calibration to flash.

```bash
python3 tools/calibrate.py calibrate
```

---

## can_tools.py

> Works on any firmware build.

CAN bus diagnostics.

| Subcommand | Description |
|---|---|
| `can-status` | Read FDCAN1/2 protocol registers (PSR, ECR, RXF0S, CCCR) |
| `can-scan` | Sniff all CAN IDs on bus 1 for N seconds and show Hz breakdown |

```bash
python3 tools/can_tools.py can-status
python3 tools/can_tools.py can-scan --duration 5
```

**Registered IDs (bus 1):**

| ID | Device |
|---|---|
| 0x241–0x244 | Hip motor responses (RMD, IDs 1–4) |
| 0x245–0x246 | Wheel motor responses (SDC102, IDs 5–6) |

---

## logs.py

> Works on any firmware build.

SD card log management. Logs are ArduPilot DataFlash binary format, compatible with [UAV Log Viewer](https://plot.ardupilot.org).

```bash
python3 tools/logs.py logs list
python3 tools/logs.py logs download <n>
python3 tools/logs.py logs decode <file.bin>
python3 tools/logs.py logs erase
python3 tools/logs.py log-status
```

---

## flash_upload.py

Upload a firmware binary via the USB bootloader.

```bash
python3 tools/flash_upload.py build/BPRL_BALANCE.bin
```
