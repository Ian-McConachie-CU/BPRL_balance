# CAN Configuration Reference

Consolidated protocol reference for the two motor families this tool drives.
Source documents (vendor-provided, user-supplied to this project):

- **LK-TECH / Shanghai LingKong Technology, "CAN PROTOCOL V2.35"** — covers the
  DG80R/C7 drive used by the MG8016E-i6 hip motors.
- **SteadyWin / Skyline Innovation, "STEADYWIN MOTOR DRIVER PROTOCOL
  SPECIFICATION rev2.2"** — covers the GIM6010-6 wheel motors (SDC10x-class
  drive). Applies to CAN, RS485 and RS232; only the CAN framing is used here.

This file is documentation only — the actual firmware implementation lives in
`src/RmdMotor.hpp/.cpp` (LK-TECH) and `src/GimMotor.hpp/.cpp` (SteadyWin), and
their header comments record exactly which parts of these protocols are
confirmed against real hardware vs. still placeholder. Treat those header
comments as the source of truth if this file and the code ever disagree.

---

## Fleet layout (this robot, as currently configured)

| Motor | Vendor model | Drive | CAN ID | Bus | Role |
|---|---|---|---|---|---|
| Hip 1 | LK-TECH MG8016E-i6 | DG80R/C7 | 1 | CAN bus 1 (FDCAN1) | RMD protocol |
| Hip 2 | LK-TECH MG8016E-i6 | DG80R/C7 | 2 | CAN bus 1 (FDCAN1) | RMD protocol |
| Hip 3 | LK-TECH MG8016E-i6 | DG80R/C7 | 3 | CAN bus 1 (FDCAN1) | RMD protocol |
| Hip 4 | LK-TECH MG8016E-i6 | DG80R/C7 | 4 | CAN bus 1 (FDCAN1) | RMD protocol |
| Wheel 1 | SteadyWin GIM6010-6 | SDC10x-class | 10 | CAN bus 1 (FDCAN1) | GIM protocol |
| Wheel 2 | SteadyWin GIM6010-6 | SDC10x-class | 11 | CAN bus 1 (FDCAN1) | GIM protocol |

All six motors share **one physical CAN bus** (`BUS,1` in this tool / bus 1 in
`motor_tool.py`). CAN bus 2 (FDCAN2) is reserved in this firmware for the
IMX5 IMU (`Imx5.hpp`) — not for motors. IDs above match `RmdMotor`/`GimMotor`
defaults in the firmware and `DEFAULT_RMD_IDS`/`DEFAULT_GIM_IDS` in
`tools/motor_tool.py`.

Bench power: all six drives are being run from a single 41 V supply.
- DG80R/C7 (MG8016E-i6): rated 12–60 V — 41 V is comfortably inside range.
- GIM6010-6 drive: input voltage range is **not stated** in the provided
  SteadyWin document (only baud rates/DLC are — see below). You already
  brought each GIM motor up over UART/RS485 with the LingKong-equivalent GUI
  at this same supply, so it presumably tolerated 41 V, but there's no
  written spec here to point to — if either GIM's Over/Under Voltage
  Threshold config (ConfType 0x00, ConfID 0x16/0x17) is set tighter than
  41 V it will fault on power-up. Check that config if a GIM refuses to
  start.

---

## 1. LK-TECH MG8016E-i6 — CAN Protocol V2.35 ("RMD" driver)

### 1.1 Bus parameters

- Interface: CAN, standard 11-bit frame, data frame, DLC = 8.
- Baud rate (single-motor / normal mode): 1 Mbps (default), 500k, 250k,
  125k, 100k.
- Baud rate (broadcast/multi-motor mode): 1 Mbps or 500 kbps only.
- Up to 32 nodes per bus; each drive needs a unique ID, 1–32, set either by
  the drive's 4-position DIP switch (values `#1`–`#8` direct-encoded — see
  table below) or by the software "Driver ID" field when the DIP is set to
  `0`. **The DIP switch overrides the software ID when non-zero** — if you
  configured IDs 1–4 via the GUI, confirm each drive's DIP is actually set
  to `0`, or the physical switch position wins.
- Termination: 120 Ω resistor at both physical ends of the bus (DIP switch
  position `R`/4th switch enables a drive's own internal 120 Ω — only the
  two end nodes should have it enabled).
- A new ID or baud rate is only valid after Save + power cycle on the drive.

DIP switch → ID mapping (when Driver ID = 0):

| ID | switch3 | switch2 | switch1 |
|---|---|---|---|
| #1 | OFF | OFF | OFF |
| #2 | OFF | OFF | ON |
| #3 | OFF | ON | OFF |
| #4 | OFF | ON | ON |
| #5 | ON | OFF | OFF |
| #6 | ON | OFF | ON |
| #7 | ON | ON | OFF |
| #8 | ON | ON | ON |

### 1.2 Frame addressing

- Command frame **and** reply frame both use arbitration ID `0x140 + ID`
  (ID = 1..32) — **confirmed against real hardware in this project**. This
  is *not* the `0x240 + ID` convention some MyActuator/RMD-X clones use;
  that was this project's original (wrong) assumption and meant replies
  were never received regardless of wiring. See `RmdMotor.hpp` for the
  history.
- Reply is sent within ~0.25 ms of the command being received.
- Byte order: the multi-byte fields below are little-endian (LSB first),
  built by casting the value's address to `uint8_t*` and reading forward.

### 1.3 Command byte reference

All frames are 8 bytes, `DATA[0]` is the command byte. `NULL` = don't-care
byte, send `0x00`.

| Command | Byte | Payload (host→drive) | Reply (drive→host) |
|---|---|---|---|
| Motor off | `0x80` | all NULL | echo of command |
| Motor on | `0x88` | all NULL | echo of command |
| Motor stop | `0x81` | all NULL | echo of command |
| Open loop control (MS series only) | `0xA0` | `DATA[4:5]` = int16 PowerControl, −850..850 | temp(int8), power(int16 −850..850), speed(int16, 1dps/LSB), encoder(uint16) |
| Torque closed-loop (MF/MH/MG) | `0xA1` | `DATA[4:5]` = int16 iqControl, −2048..2048 (MG: −33..33 A actual) | temp(int8), iq(int16), speed(int16, 1dps/LSB), encoder(uint16) |
| Speed closed-loop | `0xA2` | `DATA[4:7]` = int32 speedControl, 0.01 dps/LSB | same layout as torque reply, cmd byte 0xA2 |
| Multi-loop angle 1 | `0xA3` | `DATA[4:7]` = int32 angleControl, 0.01 deg/LSB | same layout, cmd byte 0xA3 |
| Multi-loop angle 2 (+ speed limit) | `0xA4` | `DATA[2:3]` = uint16 maxSpeed (1 dps/LSB), `DATA[4:7]` = int32 angleControl (0.01 deg/LSB) | same layout, cmd byte 0xA4 |
| Single-loop angle 1 | `0xA5` | `DATA[1]` = spinDirection (0=CW,1=CCW), `DATA[4:7]` = uint32 angleControl (0.01 deg/LSB, 0-359.99°) | same layout, cmd byte 0xA5 |
| Single-loop angle 2 (+ speed limit) | `0xA6` | `DATA[1]` = spinDirection, `DATA[2:3]` = uint16 maxSpeed, `DATA[4:7]` = uint32 angleControl | same layout, cmd byte 0xA6 |
| Increment angle 1 | `0xA7` | `DATA[4:7]` = int32 angleIncrement, 0.01 deg/LSB, sign = direction | same layout, cmd byte 0xA7 |
| Increment angle 2 (+ speed limit) | `0xA8` | `DATA[2:3]` = uint32(sic, actually fits uint16) maxSpeed, `DATA[4:7]` = int32 angleIncrement | same layout, cmd byte 0xA8 |
| Read PID | `0x30` | all NULL | `DATA[2..7]` = anglePidKp/Ki, speedPidKp/Ki, iqPidKp/Ki (each 1 byte) |
| Write PID → RAM (volatile) | `0x31` | `DATA[2..7]` = same 6 PID bytes | echo |
| Write PID → ROM (persistent) | `0x32` | `DATA[2..7]` = same 6 PID bytes | echo |
| Read acceleration | `0x33` | all NULL | `DATA[4:7]` = int32 Accel, 1 dps/s |
| Write acceleration → RAM | `0x34` | `DATA[4:7]` = int32 Accel | echo |
| Read encoder | `0x90` | all NULL | `DATA[2:3]` encoder (uint16, raw−offset, **16-bit** — see note below), `DATA[4:5]` encoderRaw, `DATA[6:7]` encoderOffset |

> **Encoder bit-width correction (2026-08-31, confirmed against real
> MG8016E-i6/DG80R7E hardware):** the vendor doc's example for this command
> describes a 14-bit field (0..16383). Real replies contradict that — raw
> values seen on the bus (e.g. `encoder=54735`) exceed 16383, and the
> `encoder`/`encoderRaw`/`encoderOffset` triple in the same reply is
> internally self-consistent (`encoder == encoderRaw - encoderOffset`),
> ruling out a decode/byte-position bug. This drive's own GUI reports
> "Encoder Type: 16Bit Encoder" (`Ktech_manual.pdf`) — treating the field as
> 16-bit (0..65535, i.e. `angle(RAD) = raw * 2π/65536`) puts every reading
> back in a sane single-turn 0-360° range. The 14-bit width in the vendor
> doc's Read-encoder section is a per-model example, not a fixed constant —
> don't assume it carries over to other LK-TECH motor models without
> checking that model's own encoder resolution first.
| Write encoder offset → ROM (zero point) | `0x91` | `DATA[6:7]` = uint16 encoderOffset | echo |
| Write current position → ROM as zero | `0x19` | all NULL | `DATA[6:7]` = new encoderOffset (0 bias) |
| Read multi-turn angle | `0x92` | all NULL | `DATA[1:7]` = int64 (7 bytes used) motorAngle, 0.01°/LSB, signed cumulative |
| Read single-turn angle | `0x94` | all NULL | `DATA[4:7]` = uint32 circleAngle, 0.01°/LSB, 0..35999 |
| Clear angle loop (not yet available per vendor) | `0x95` | all NULL | echo |
| Read state 1 + error state | `0x9A` | all NULL | `DATA[1]` temp(int8,1°C/LSB), `DATA[2:3]` voltage(uint16,**0.01V/LSB**, see note below), `DATA[7]` errorState bitmask |
| Clear error state | `0x9B` | all NULL | same layout as state1 (cmd byte reported as `0x9A` in the reply per the vendor doc) |

> **Voltage field correction (2026-08-31, confirmed against real MG8016E-i6 hardware):**
> the vendor PDF documents this as `DATA[3:4]` at 0.1V/LSB with `DATA[2]` as
> NULL/reserved. Real replies from all 4 hip drives on this project's bench
> (via `poll hips`, cross-checked byte-by-byte against a known 41V supply)
> contradict that: `DATA[3]` sat at a constant `0x10` across every unit
> regardless of actual bus voltage, while `DATA[2]` (documented as NULL)
> tracked small run-to-run sensor noise exactly like a real ADC reading.
> `DATA[2:3]` (LSB order) at **0.01V/LSB** put all 4 units at 41.8-42.1V
> against the known 41V rail — one byte earlier and 10x finer resolution
> than documented. `RmdMotor.cpp` and `motor_tool.py`'s `decode_rmd_9a()`
> both use the corrected layout; trust the hardware over this PDF table for
> this field specifically.
| Read state 2 (temp/iq/speed/encoder) | `0x9C` | all NULL | temp(int8), iq(int16, −2048..2048), speed(int16,1dps/LSB), encoder(uint16,14bit) |
| Read state 3 (temp/phase currents) | `0x9D` | all NULL | temp(int8), iA/iB/iC each int16, 1A/64LSB |

`errorState` bitmask (from state1/clear-error replies):

| bit | meaning | 0 | 1 |
|---|---|---|---|
| 0 | voltage | normal | under-voltage protect |
| 3 | temperature | normal | over-temperature protect |
| 1,2,4,5,6,7 | invalid/unused | — | — |

### 1.4 Multi-motor broadcast (not wired up in this tool)

- Identifier `0x280`, torque-only, up to 4 motors (IDs #1–#4, no repeats),
  `DATA[0:1]/[2:3]/[4:5]/[6:7]` = int16 iqControl for motors 1–4
  respectively (range −2000..2000, ±32 A). Replies come back individually
  on each motor's normal `0x140+ID` reply address, in ID order 1→4.
- Requires all participating drives set to the same broadcast baud (1 Mbps
  or 500 kbps) and IDs 1–4 specifically. This project's firmware currently
  only issues per-motor commands (see `RmdMotor.cpp`) — broadcast mode is
  documented here for completeness but not implemented.

### 1.5 Position control: three different reference frames — pick deliberately

The protocol exposes three distinct position-control families, and they are
**not interchangeable** — picking the wrong one for what you actually mean
looks exactly like "the motor isn't responding," which cost real debugging
time on this project (2026-08-31) before the distinction was clear:

| Family | Commands | Reference frame | Direction |
|---|---|---|---|
| Multi Loop Angle Control | `0xA3`/`0xA4` | **Absolute, multi-turn** — a cumulative count that can be thousands of degrees after normal handling/testing | Automatic (sign of target − current) |
| Single Loop Angle Control | `0xA5`/`0xA6` | **Absolute, single-turn** — 0..359.99°, wraps every revolution | You specify CW/CCW explicitly (`DATA[1]`) |
| Increment Angle Control | `0xA7`/`0xA8` | **Relative** — a delta from wherever the shaft currently is | Sign of the delta |

`0x90` (Read Encoder) reports a **single-turn** position — the same frame
as Single Loop, *not* the frame `0xA3`/`0xA4` operate in. Commanding "0deg"
via `0xA4` after the shaft has accumulated many revolutions of multi-turn
count can mean a very large real move that looks like no response at all if
you're watching the single-turn encoder and comparing against 0xA4's
target. If what you actually want is "go to N degrees, where N is what
`poll hips`/`0x90` reports," use Single Loop (`0xA6` in this firmware,
`rmd_position_single_turn()`), not Multi Loop.

### 1.6 What this project's driver actually uses

See the header comment in `src/RmdMotor.hpp` for the authoritative list of
confirmed vs. placeholder fields. Currently wired up: motor off/on/stop
(`0x80/0x88/0x81`), torque (`0xA1`, both raw ratio and a placeholder
Nm-scaled wrapper), speed (`0xA2`), position — multi-turn absolute
(`0xA4`), single-turn absolute (`0xA6`), and relative (`0xA7`) — read
state1 (`0x9A`), clear error (`0x9B`), read encoder (`0x90` — confirmed
against real hardware, with one correction from the vendor doc; see the
note under that row above). Everything else in the table above (PID/accel
read-write — `0x30`/`0x33` sent fine over CAN but this drive did not reply
to either, so those specific reads appear unsupported on this firmware —
write-encoder-offset, no-speed-limit angle variants `0xA3`/`0xA5`/`0xA8`,
state2/3, broadcast) exists in the vendor spec but is not sent by this
firmware today.

---

## 2. SteadyWin GIM6010-6 — Motor Driver Protocol rev2.2 ("GIM" driver)

### 2.1 Bus parameters

- CAN: baud rate < 1 Mbps, ID customized, standard frame, DLC 8, no frame
  header (payload is the whole 8 bytes), byte order LSB.
- RS485/RS232 also exist (921600 baud, not used on this bus) with a 4-byte
  header the CAN framing does not have — don't mix that framing into CAN
  frames.

### 2.2 Command byte reference

`DATA[0]` = COMMAND byte. Reply frame echoes the same command byte plus a
`RES` result code (see 2.3).

| Category | Command | Byte |
|---|---|---|
| Configuration | Reset Configuration | `0x81` |
| | Refresh Configuration | `0x82` |
| | Modify Configuration | `0x83` |
| | Retrieve Configuration | `0x84` |
| Control | Start Motor | `0x91` |
| | Stop Motor | `0x92` |
| | Torque Control | `0x93` |
| | Speed Control | `0x94` |
| | Position Control | `0x95` |
| | PTS Control | `0x96` (not supported) |
| | Stop Control (halt current command) | `0x97` |
| Parameter | Modify Parameter | `0xA1` |
| | Retrieve Parameter | `0xA2` |
| Status | Get Version | `0xB1` |
| | Get Fault | `0xB2` |
| | Acknowledge Fault | `0xB3` |
| | Retrieve Indicator | `0xB4` |
| | Calibration | `0xB5` |
| Update | Update Firmware | `0xC1` (no response expected) |

### 2.3 Result codes (`RES` byte, most command replies)

| RES | Meaning |
|---|---|
| `0x00` | Success |
| `0x01` | Failure |
| `0x02` | Failure, Unknown Command |
| `0x03` | Failure, Unknown ID |
| `0x04` | Failure, Read-Only Register |
| `0x05` | Failure, Unknown Register |
| `0x06` | Failure, String Format |
| `0x07` | Failure, Data Format Error |
| `0x0B` | Failure, Write-Only Register |

### 2.4 Control command payloads

| Command | Host→Drive payload | Drive→Host reply |
|---|---|---|
| Start Motor `0x91` | all NULL | `DATA[1]` = RES |
| Stop Motor `0x92` | all NULL | `DATA[1]` = RES |
| Torque `0x93` | `DATA[1:4]` = IEEE float torque (N·m), `DATA[5:7]` = 24-bit uint duration (ms) | `DATA[1]`=RES, `DATA[2]`=temp, `DATA[3:4]`=packed pos (16-bit), `DATA[5:7]`=ST0-2 packed speed+torque |
| Speed `0x94` | `DATA[1:4]` = IEEE float target RPM, `DATA[5:7]` = duration (ms) | same reply layout as torque |
| Position `0x95` | `DATA[1:4]` = IEEE float target RAD, `DATA[5:7]` = duration (ms) | same reply layout as torque |
| Stop Control `0x97` | all NULL | `DATA[1]` = RES |

**Reply decode formulas (shared by Torque/Speed/Position replies):**

```
pos_float   = pos_int   * 25   / 65535 - 12.5                 # RAD, from Pos0..Pos1 (16-bit)
speed_float = speed_int * 130  / 4095  - 65                   # RAD/s, 12-bit: ST0 (hi8) + ST1[7:4] (lo4)
torque_float = torque_int * (450 * Kt * gear_ratio) / 4095
               - 225 * Kt * gear_ratio                        # N·m, 12-bit: ST1[3:0] (hi4) + ST2 (lo8)
```
`Kt` (torque constant, N·m/A) and `gear_ratio` are motor-specific and not
returned by these commands — read them separately via Retrieve Configuration
(ConfID 0x03 float, ConfID 0x11 int respectively) if you need accurate
torque feedback. Torque *commands* are already real N·m (IEEE float), only
the feedback decode needs these constants.

### 2.5 Configuration (`0x83` Modify / `0x84` Retrieve)

Payload: `DATA[1]`=ConfType, `DATA[2]`=ConfID, `DATA[3]`=NULL,
`DATA[4:7]`=DATA0-3 (value, LSB order). Reply: same `ConfType/ConfID` echoed,
`DATA[3]`=RES, then DATA0-3 for Retrieve (Modify reply has no value bytes).

ConfType:

| ConfType | Meaning |
|---|---|
| `0x00` | 32-bit signed integer |
| `0x01` | 32-bit signed float (IEEE) |

ConfID (0x00 = integer configs):

| ConfID | Name | Perm | Effective |
|---|---|---|---|
| 0x00 | Pole Pairs | R/W | Immediately |
| 0x01 | Rated Current (A) | R/W | Immediately |
| 0x02 | Max Speed (RPM) | R/W | Immediately |
| 0x06 | Rated Voltage (V) | R/W | Immediately |
| 0x07 | PWM Frequency (Hz) | R/W | After reboot |
| 0x08 | Default KP Current Loop | R/W | Immediately |
| 0x09 | Default KI Current Loop | R/W | Immediately |
| 0x0C | Default KP Speed Loop | R/W | Immediately |
| 0x0D | Default KI Speed Loop | R/W | Immediately |
| 0x0E | Default KP Position Loop | R/W | Immediately |
| 0x0F | Default KI Position Loop | R/W | Immediately |
| 0x10 | Default KD Position Loop | R/W | Immediately |
| 0x11 | Gear Ratio | R/W | Immediately |
| 0x12 | **CAN ID** | R/W | Immediately |
| 0x13 | Host/Master CAN ID | R/W | Immediately |
| 0x14 | Zero Position (output shaft) | R/W | Immediately |
| 0x15 | Power-Off Position (output shaft) | R | Immediately |
| 0x16 | Over Voltage Threshold (V) | R/W | Immediately |
| 0x17 | Under Voltage Threshold (V) | R/W | Immediately |
| 0x18 | CAN Baud Rate | R/W | Immediately |
| 0x19 | Default KP Flux Weakening | R/W | Immediately |
| 0x1A | Default KI Flux Weakening | R/W | Immediately |
| 0x1C | Protocol over CAN (0=SteadyWin GIM default, 1=MIT) | R/W | Immediately |
| 0x20 | Over Temperature Threshold | R/W | Immediately |

ConfID (0x01 = float configs): 0x00 Rs (Ω), 0x01 Ls (H), 0x02 Back EMF
Constant (Vrms/kRPM), 0x03 Torque Constant (N·m/A), 0x04 Sampling Resistor
(Ω), 0x05 Amplification Gain — all R/W, immediate.

`Zero Position`/`Power-Off Position` are 16-bit ints convertible to RAD via
`position(RAD) = position(int) * 2π/65536`.

**Note:** `0x1C` (Protocol over CAN) must stay `0` (SteadyWin GIM, default)
for `GimMotor.cpp`'s framing to apply — if a drive was ever put in MIT mode
during UART setup, flip it back before testing on this bus.

### 2.6 Parameter (`0xA1` Modify / `0xA2` Retrieve) — runtime PID, not persisted like Configuration

Payload: `DATA[1]`=ParaID, `DATA[4:7]`=DATA0-3 (32-bit unsigned int, LSB).

| ParaID | Name |
|---|---|
| 0x00 | Runtime KP Current Loop |
| 0x01 | Runtime KI Current Loop |
| 0x02 | Runtime KP Speed Loop |
| 0x03 | Runtime KI Speed Loop |
| 0x04 | Runtime KP Position Loop |
| 0x05 | Runtime KI Position Loop |
| 0x06 | Runtime KD Position Loop |
| 0x07 | Runtime KP Flux Weakening |
| 0x08 | Runtime KI Flux Weakening |

### 2.7 Status commands

**Get Fault `0xB2`** — `DATA[1]`=RES, `DATA[2]`=FaultNo (bitmask):

| FaultNo | Meaning |
|---|---|
| 0x00 | No Fault |
| 0x01 | FoC Frequency Too High |
| 0x02 | Over Voltage |
| 0x04 | Under Voltage |
| 0x08 | Over Temperature |
| 0x10 | Start Failure |
| 0x40 | Over Current |
| 0x80 | Software Exception |

**Acknowledge Fault `0xB3`** — clears latched fault; while unacknowledged the
drive declines all other commands. Motor stops running on any fault.

**Retrieve Indicator `0xB4`** — `DATA[1]`=IndID (request), reply
`DATA[1]`=IndID, `DATA[2]`=RES, `DATA[4:7]`=IEEE float value:

| IndID | Indicator | IndID | Indicator |
|---|---|---|---|
| 0x00 | Bus Voltage (V) | 0x0D | Vq (V) |
| 0x01 | Driver Board Temp | 0x0E | Vd (V) |
| 0x02 | Motor Temperature | 0x0F | Valpha (V) |
| 0x03 | Power (W) | 0x10 | Vbeta (V) |
| 0x04 | Ia (A) | 0x11 | Electrical Angle of Rotor (RAD) |
| 0x05 | Ib (A) | 0x12 | Mechanical Angle of Rotor (RAD) |
| 0x06 | Ic (A) | 0x13 | Mechanical Angle of Output Shaft (RAD) |
| 0x07 | Ialpha (A) | 0x14 | Speed, output shaft (RPM) |
| 0x08 | Ibeta (A) | 0x15 | Output Power (W) |
| 0x09 | Iq (A) | | |
| 0x0A | Id (A) | | |
| 0x0B | Target Iq (A) | | |
| 0x0C | Target Id (A) | | |

**Get Version `0xB1`** — `DATA[4:7]` = uint32 version.

**Calibration `0xB5`** — `DATA[1]`=CaliType (0=phase order, 1=encoder). Note
the vendor doc's reply table for this command uses `0xB4` as the echoed
command byte, which looks like a documentation typo (compare against 0x83's
reply table) — verify empirically before relying on it.

### 2.8 What this project's driver actually uses

See `src/GimMotor.hpp` for the confirmed/placeholder breakdown. Wired up:
Start/Stop/Stop-Control (`0x91/0x92/0x97`), Torque/Speed/Position
(`0x93/0x94/0x95`), Get Fault/Ack Fault (`0xB2/0xB3`), Retrieve Indicator
(`0xB4`). Not wired up: all Configuration/Parameter/Calibration/Firmware
commands, Get Version — send those manually via `CAN,send` if needed during
bring-up (e.g. to double check ConfID 0x12 CAN ID or 0x1C protocol mode).
The **reply arbitration ID for GIM CAN frames is an assumption, not
vendor-stated** — the spec only documents a reply-ID scheme for RS485; this
driver assumes CAN replies use the same ID as the command, by analogy with
RS485 and the now-confirmed LK-TECH behavior. Confirm with `CAN,monitor`
during bring-up rather than trusting it blind.

---

## Cross-cutting notes

- **Termination:** with 6 nodes on one physical bus, only the two nodes at
  the physical ends of the backbone should have their 120 Ω terminator
  enabled (LK-TECH: DIP switch `R`; GIM: check its own config/jumper) — not
  every drop. Enabling it on every node will kill signal integrity.
- **Shared bus:** RMD (1–4) and GIM (10–11) IDs don't collide with each
  other or with the tool's own `0x140+ID`/GIM-ID addressing, so they can
  coexist on one physical bus (`BUS,1`) as currently wired. CAN bus 2 is
  reserved for the IMX5 IMU only in this firmware.
- **Reply-ID assumptions:** LK-TECH's `0x140+ID` reply address is confirmed
  against hardware; GIM's same-ID-as-command reply address is inferred, not
  vendor-stated — treat GIM silence as ambiguous (could be wrong reply-ID
  assumption, not necessarily a wiring fault) until confirmed with
  `CAN,monitor`.
- **41 V bench supply:** fine for the DG80R/C7 (12–60 V rated); GIM6010-6
  drive voltage range isn't in the provided doc — if a GIM won't Start,
  check its Over/Under Voltage Threshold config (0x16/0x17) and Get Fault
  (0xB2) for `0x02`/`0x04` before assuming a wiring problem.
