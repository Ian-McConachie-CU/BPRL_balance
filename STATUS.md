# BPRL_Balance — Project Status

## What was done

### Project setup
- Forked from BPRL_flight; stripped git history; restructured for a 5-bar linkage wheeled biped
- Board locked to CubeOrangePlus (STM32H743ZI) only — CubeBlueH7 board files deleted
- Build output renamed to `BPRL_BALANCE.elf/.bin/.hex`

### Hardware removed
| Removed | Reason |
|---|---|
| DShot / PWM motor output | Motors are CAN-only |
| CRSF radio protocol | Replaced with SBUS |
| I2C subsystem | Not used on this platform |
| Strain rate sensors | Not present on this robot |

### Hardware added / ported
| Added | Notes |
|---|---|
| FDCAN2 (bus 2) | Enabled in `mcuconf.h`; both buses start at 1 Mbit/s |
| USART6 for SBUS | Enabled in `mcuconf.h`; 100000 baud 8E2, hardware-inverted on CubeOrangePlus PCB |
| IMX5 INS on CAN bus 2 | Standard 11-bit IDs 0x01–0x04; quaternion + rates → `g_can_imu` |
| Matek CAN-L4-BM power monitor | DroneCAN BatteryInfo (DTID 1092); masked EID match; voltage + current → `g_power` |
| 6 CAN motors on bus 1 | IDs 1–4 hip (RMD/LKMTECH MG8016E-i6), IDs 5–6 wheel (SDC102/Steadywin GIM6010-6) |

### Software structure
- `src/controllers/RobotStateMachine` — replaces FlightStateMachine; states: IDLE, BALANCING, MANUAL
- `src/controllers/BalanceController` — stub; outputs zero torques until balance logic is written
- `src/coms/CANMotor` — abstracts RMD and SDC102 protocols; `can_motor_set_torque()` / `can_motor_get_state()`
- `src/coms/CANPower` — DroneCAN BatteryInfo parser with float16→float32 conversion
- `src/coms/SBUS` — real SD6 implementation (was a stub from BPRL_flight)
- `src/RobotState.hpp` — `InputIdx::MODE_SW` replaces `FLIGHT_MODE`
- 3-lane EKF (16-state) carried over unchanged

### Tools
- `telemetry.py` — RPM panel removed; shows attitude, rates, IMU status, IMU compare
- `motor_test.py` — rewritten as CAN motor status tool; torque/velocity commands are TODO stubs
- `can_tools.py` — updated for FDCAN1/2; strain sensor ID removed
- `dshot_tools.py`, `strain_rate.py` — deleted
- `tools/README.md`, `README.md` — rewritten for robot context

### USB commands available
| Command | Response |
|---|---|
| `BOOT` | Triggers system reset |
| `LOG,status` / `list` / `get,N` / `erase` | SD card log management |
| `CAL,set,N,...` / `commit` / `clear` / `query` | IMU calibration |
| `CAN,status` / `diag` / `scan,start` / `scan,stop` / `regdump` | CAN diagnostics |
| `MOTOR,status` | Position, velocity, torque, temp for all 6 motors |
| `POWER,status` | Matek power monitor: voltage, current, node ID |

---

## What still needs testing

### Bring-up / boot
- [ ] Flash to CubeOrangePlus and confirm heartbeat LED blinks (RTOS boots)
- [ ] USB CDC enumerates (`/dev/ttyACM0`, VID:PID `0483:5740`)
- [ ] Watchdog does not trip under normal operation

### SBUS radio
- [ ] SBUS receiver connected to SBUSo port; `$RCIN` stream shows live channel values
- [ ] `InputIdx::MODE_SW` (channel 4) reads correctly from the mode switch
- [ ] Frame-lost and failsafe flags set correctly when receiver is powered off

### CAN bus 1 — motors
- [ ] `CAN,diag` shows frames being received on bus 1
- [ ] `MOTOR,status` returns valid state for all 6 motor IDs after power-on
- [ ] RMD torque command (`can_motor_set_torque`) produces motion on hip motors
- [ ] SDC102 torque command produces motion on wheel motors — **SDC102 frame format is a stub; verify against GIM6010-6 manual before use**

### CAN bus 2 — IMX5
- [ ] `CAN,diag` shows frames on bus 2
- [ ] `$TEL` stream shows `can_valid=1` and non-zero `can_hz`
- [ ] Quaternion from IMX5 tracks real orientation (check with `telemetry.py imu-compare`)

### CAN bus 2 — Matek power monitor
- [ ] `POWER,status` returns `valid=1` with sensible voltage and current after monitor powers up
- [ ] DroneCAN node ID is logged correctly (`node=N`)
- [ ] Voltage and current match a reference meter to within a few percent

### EKF / state estimator
- [ ] `$EKFL` stream outputs at ~500 Hz; all 3 lanes converge
- [ ] Primary lane selection switches correctly when a lane diverges
- [ ] IMX5 quaternion fused as external measurement (check against onboard EKF attitude)

### Logging
- [ ] SD card mounts; `LOG,status` reports `ready`
- [ ] `logs.py logs list` shows files; `logs.py logs download N` produces a valid DataFlash binary
- [ ] Log opens in UAV Log Viewer; ATT, RCIN, OUTP messages present

### Balance controller
- [ ] `BalanceController` is a zero-output stub — **balance logic not yet written**
- [ ] Robot should be safe to power up in IDLE state; verify no torques are sent until BALANCING mode is commanded

### Build
- [x] `make` compiles cleanly (warnings only, no errors) as of 2026-08-28
