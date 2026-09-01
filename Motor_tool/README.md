# Motor_tool

Standalone ChibiOS firmware for the CubePilot CubeOrange+ (STM32H743ZI) that
turns the board into a dedicated **CAN motor test/setup bench tool** —
independent of the BPRL_balance robot firmware in the parent directory. Flash
it to a spare Cube (or the robot's own Cube, temporarily, before it's wired
into the full robot) to sniff CAN traffic and drive individual motors from a
laptop while bringing up or debugging:

- **LK-TECH MG8016E-i6** hip actuators (DG80R/C7 drive, confirmed CAN protocol)
- **SteadyWin GIM6010-6** wheel motors (confirmed CAN protocol)

It reuses this repo's `boards/CubeOrangePlus`, `cfg/`, and
`third_party/ChibiOS` unchanged (same MCU, same clock tree) but has its own
`main.cpp` and a small, purpose-built set of sources under `src/` — no EKF,
controllers, SBUS, IMUs, or SD logging.

---

## ⚠️ Protocol confidence — read this before commanding real torque

| Motor | Confidence | Basis |
|---|---|---|
| **MG8016E-i6** — CAN byte layout, response ID, all confirmed command bytes | **Confirmed** | Vendor's "CAN PROTOCOL V2.35" (LK-TECH / Shanghai LingKong Technology, user-provided). Verified live against real hardware — see `RmdMotor.hpp` for exactly which fields. |
| **MG8016E-i6** — 0x9A voltage field | **Confirmed, corrected from vendor doc** | Real hardware puts it at `data[2:3]` (LSB) at 0.01V/LSB, not the doc's `data[3:4]` at 0.1V/LSB — verified against a known 41V supply across all 4 hips. See `CAN_config.md` §1.3. |
| **MG8016E-i6** — 0x90 encoder field | **Confirmed, corrected from vendor doc** | 16-bit (0..65535), not the doc's generic 14-bit example — this motor's own GUI reports "16Bit Encoder." Also: this reads the **motor/rotor side, before the 1:6 gearbox** — divide by the gear ratio for output-shaft angle (`encoder <id>` does this automatically). Confirmed via hand-rotation tests, see `CAN_config.md` §1.3/§1.5. |
| **MG8016E-i6** — Nm↔ratio scale for `RMD,<id>,TORQUE` | **Placeholder** | The protocol confirms the raw command range (±2048, -33..33A for MG series) but not the motor's torque constant (Nm/A), so the Nm convenience wrapper is still a guess. Prefer `RMD,<id>,TORQUERAW,<ratio>`. |
| **GIM6010-6** — CAN byte layout, all command bytes, feedback decode | **Confirmed** | Vendor's "STEADYWIN MOTOR DRIVER PROTOCOL SPECIFICATION rev2.2" (SteadyWin / Skyline Innovation, user-provided). Replaces an earlier best-effort "MIT motor mode" guess that had no real basis. |
| **GIM6010-6** — reply arbitration ID | **Confirmed — NOT same as command ID** | A GIM's reply lands on its configured **Host/Master CAN ID** (ConfID 0x13), which is a separate value from its own CAN ID (ConfID 0x12) — confirmed via `poll_wheels` against real hardware (this project's wheels: CAN ID 15→reply 16, CAN ID 20→reply 21). The driver tracks this per-motor via `gim_set_reply_id()` / `GIM,<id>,MASTERID,<reply_id>`; `motor_tool.py`'s `ensure_gim_reply_ids()` configures the known mapping before every poll. **Check each new GIM's actual CAN ID vs. Host/Master CAN ID via the GUI — don't assume they match.** |
| **GIM6010-6** — Nm↔raw-feedback torque decode | **Placeholder** | The packed torque feedback field needs this motor's torque constant (Nm/A) and gear ratio, neither of which this driver reads automatically — see `GIM,KT,<Nm_per_A>` / `GIM,GEAR,<ratio>`. Torque *commands* are real Nm already (IEEE float, confirmed), only the *feedback decode* is a placeholder. |

**Real, non-obvious bugs this surfaced** (worth remembering if a future
motor's behavior looks impossible rather than just broken):
- The MG8016E-i6's reply frame uses the *same* CAN identifier as the
  command (`0x140+id`), not `0x240+id` like the MyActuator/RMD-X convention
  this project originally guessed from. That one-line mistake meant the
  driver could never have seen a real reply, regardless of wiring.
- The GIM6010-6's reply does **not** land on the same ID as the command —
  it lands on a separately-configured Master CAN ID, which is not
  necessarily equal to the motor's own CAN ID (see table above). A GIM can
  ack every request `OK` (frame sent fine) and still show total silence in
  `STATUS` forever, purely because the tool was listening on the wrong ID —
  indistinguishable from a wiring fault without checking this specifically.
- A vendor-doc byte position/scale can simply be wrong for real hardware —
  the 0x9A voltage field and 0x90 encoder bit-width both were. `poll hips`/
  `poll gim` exist specifically to cross-check a decode against raw bytes
  before trusting it.

**Open issue as of 2026-08-31 — unresolved:** every RMD *and* GIM command
suddenly started failing at the firmware TX level (`can_send()` itself
returning `ERR`, not just "no reply"), after all 6 motors were connected to
bus 1 simultaneously for the first time. Affects both drivers identically,
which point to a bus-1-wide condition (likely bus-off) rather than a
software bug in either driver — see `plan.md`'s "Current status" section
for the full detail and the next diagnostic step (`bus 1` → `selftest`).

Practical safety behavior baked into the firmware, independent of protocol
confidence:
- Every GIM command is clamped to a conservative default torque limit
  (`GIM_DEFAULT_TORQUE_LIMIT_NM = 3.3` Nm, the continuous rating) and refuses
  to move at all until `GIM,<id>,START` is sent.
- A **500 ms host watchdog** runs regardless of motor type: if this tool stops
  hearing from your PC (cable unplugged, script crashed, etc.), it actively
  stops/disables every motor it has ever touched. This does not replace a
  physical e-stop.
- The MG8016E-i6 drive itself also has "Lose Input Protection" (LIP) enabled
  by default per its own manual — leave it enabled as a second layer.

---

## Hardware

| Item | Detail |
|---|---|
| MCU / board | STM32H743ZI, CubePilot CubeOrange+ (same as BPRL_balance) |
| CAN | FDCAN1 + FDCAN2, both 1 Mbit/s by default — matches the MG8016E-i6 drive's default CAN baud rate |
| Termination | 120 Ω at both ends of the bus (per the LK-TECH manual's wiring diagram) |
| USB | CDC-ACM, `/dev/ttyACMx`, VID:PID `0483:5740`, enumerates as "BPRL Motor Tool" |

**MG8016E-i6 wiring:** `A/H`/`B/L` = CAN-H/CAN-L (same pins are also
RS485-A/B — the drive's "Bus Type" setting picks which protocol they carry),
`V+`/`V-` = 12–60 V power (drive rating; check the actual MG8016E-i6/DG80R
combination you have). Node ID 1–32, set via the 4-position DIP switch
(`#1`–`#8` direct-encoded, see the manual) or the software "Driver ID" field
when the DIP is set to 0 — a new ID only takes effect after Save + power
cycle. The DIP's 4th switch (`R`) enables the drive's own 120 Ω terminator.

**GIM6010-6 wiring:** power/physical pinout not documented here — sniff the
bus to confirm empirically. CAN addressing specifically: each GIM has a
separate **CAN ID** (what commands are sent to) and **Host/Master CAN ID**
(what its replies are addressed to) — these are independently configurable
and are *not* guaranteed to match (this project's two wheels: 15→16,
20→21). Check both via the GUI for any new GIM before assuming they're the
same — see the confidence table above.

**Current sensor (bus 2):** present on this project's hardware but **not
supported by this firmware** — no protocol reference obtained yet. `bus 2`
+ `scan`/`monitor` will show its raw traffic; see `plan.md` for the plan to
add real support once a protocol doc (or a known reference measurement to
reverse it against) is available.

---

## Build and flash

Same toolchain as BPRL_balance (`arm-none-eabi-gcc`, `make`, `python3`).

```bash
cd Motor_tool
make                             # -> build/MOTOR_TOOL.bin/.hex/.elf
make flash PORT=/dev/ttyACM0     # Cube USB bootloader (../tools/flash_upload.py)
make flash-stlink                # ST-Link / OpenOCD, if you have one wired up
```

This tool and BPRL_balance are separate firmware images for the same board —
flashing one replaces the other. There's no dual-boot; pick whichever you
need on the bench at a given time.

---

## USB command reference

One command per line, `\n`-terminated, 115200 (ignored by USB CDC). Send
`HELP` for the firmware's own copy of this list.

| Command | Effect |
|---|---|
| `PING` | → `PONG` |
| `BOOT` | Reset (into bootloader) |
| `BUS,<1\|2>` | Select CAN bus for RMD/GIM/scan commands |
| `STATUS` | Dump state of every motor that has reported feedback |
| `STOP,ALL` | Zero/disable every motor immediately |
| `CAN,status` / `CAN,diag` | FDCAN1+2 register/counter snapshot |
| `CAN,scan,start` / `CAN,scan,stop` | ID scanner on the active bus |
| `CAN,monitor,start` / `CAN,monitor,stop` | Live raw frame stream, both buses (`MON,...` lines) |
| `CAN,send,<id_hex>,<ext0\|1>,b0..b7` | Inject a raw 8-byte standard frame |
| `CAN,selftest` | Internal-loopback test of the active bus — no external wiring needed, see below |
| `RMD,<id>,TORQUE,<Nm>` | MG8016E-i6 torque (placeholder Nm scale — see warning) |
| `RMD,<id>,TORQUERAW,<ratio>` | MG8016E-i6 torque, raw ±2048 (confirmed, -33..33A for MG series) |
| `RMD,SCALE,<ratio_per_Nm>` | Get/set the Nm→ratio scale used by `TORQUE` |
| `RMD,<id>,VELOCITY,<rad/s>` | MG8016E-i6 velocity command (clamped ±24000 dps) |
| `RMD,<id>,POSITION,<rad>,<maxspeed>` | MG8016E-i6 position command — **ABSOLUTE multi-turn** target (0xA4); can require traveling hundreds of degrees if the shaft has accumulated turns, see `CAN_config.md` §1.5 |
| `RMD,<id>,SINGLETURN,<rad>,<maxspeed>[,<cw0\|1>]` | MG8016E-i6 position command — **ABSOLUTE single-turn** (0..360°, 0xA6), same reference frame as `ENCODER`; direction defaults to CW |
| `RMD,<id>,INCREMENT,<rad>` | MG8016E-i6 position command — **RELATIVE** to current position (0xA7), uses the drive's own configured Max Speed |
| `RMD,<id>,STOP` / `OFF` / `RESUME` | Zero output / disable / re-enable |
| `RMD,<id>,STATUS` | Request status1 (temp/voltage/error) |
| `RMD,<id>,CLEARERR` | Clear latched error flags |
| `RMD,<id>,ENCODER` | Request current single-turn encoder position (confirmed, motor-side/pre-gearbox, 16-bit — see confidence table) |
| `GIM,<id>,START` / `STOP` / `PAUSE` | Enter/exit running state / stop the current control command |
| `GIM,<id>,TORQUE,<Nm>[,<duration_ms>]` | Torque command (real Nm, confirmed) |
| `GIM,<id>,VELOCITY,<rad/s>[,<duration_ms>]` | Velocity command |
| `GIM,<id>,POSITION,<rad>[,<duration_ms>]` | Position command |
| `GIM,<id>,FAULT` / `ACKFAULT` | Request / clear fault status |
| `GIM,<id>,MASTERID,<reply_id>` | Set/query which SID this id's replies actually arrive on (Host/Master CAN ID) — confirmed to differ from the CAN ID on this project's wheels, see confidence table |
| `GIM,<id>,IND,<ind_id>` | Request one runtime indicator (0=bus V, 2=motor temp, 14=speed RPM, ...) |
| `GIM,LIMIT,<Nm>` | Get/set the GIM torque clamp |
| `GIM,KT,<Nm_per_A>` / `GIM,GEAR,<ratio>` | Set the constants used to decode GIM torque feedback |
| `IMU,status` | Decode the IMX5 INS on bus 2 (roll/pitch/yaw, rates, accel) |

## Python ground tool

```bash
pip install pyserial rich
python3 tools/motor_tool.py                # auto-detects the port, starts a REPL
```

Type `help` inside the REPL for the interactive command list — it now
wraps a lot more than the raw USB commands above: raw-byte diagnostics per
device (`poll hips`, `poll gim`, `poll wheels`), motion tests (`test hips`,
`test drift`), a live gearbox-corrected encoder feed (`encoder`), plus
`monitor`/`scan`/`selftest`. **See `plan.md`'s "Tool inventory" section for
the full list organized by which hardware each command exercises** — that's
the maintained reference now that the tool has grown well past what fits
here; this file stays focused on building/flashing and the raw protocol.
`CAN_config.md` has the full LK-TECH/SteadyWin protocol reference these
commands are built on, including every vendor-doc discrepancy found and
fixed this bring-up.

---

## Testing the IMX5 IMU (bus 2)

`src/Imx5.hpp/.cpp` decodes the Inertial Sense IMX5 INS on CAN bus 2 (standard
IDs 0x01-0x04) — ported directly from BPRL_balance's `src/coms/CAN.cpp`
(`imx5_can_cb`), so unlike the motor drivers this byte layout is not a guess.
That makes it useful as a **known-good reference** while debugging: if the
IMX5 has worked before but shows nothing here, the problem is more likely in
this tool's CAN path than in wiring; if it decodes cleanly here while a motor
bus stays silent, that isolates the problem to the motor bus/drives instead.

```
motor_tool> bus 2
motor_tool> imu           # one reading
motor_tool> imu 15        # watch it update live for 15s — rotate the sensor by hand
```
Raw commands: `BUS,2` then `IMU,status`.

---

## Isolating a bus problem: internal loopback self-test

If a bus shows total silence — no `scan` hits, no `monitor` traffic, error
counters that never move even after sending a command — it's ambiguous
whether the problem is in this tool's code or downstream of the STM32 pins
(transceiver, wiring, termination, the other end never powered up). `CAN,selftest`
resolves that ambiguity directly: it reconfigures the active bus into the
FDCAN core's internal loopback + bus-monitoring mode (`CCCR.TEST|MON`,
`TEST.LBCK`), sends one frame, and checks whether it comes back through the
normal RX/dispatch path — entirely inside the MCU, with no dependency on
external wiring, a transceiver, termination, or any other live node.

```
motor_tool> bus 1
motor_tool> selftest
CAN,SELFTEST,PASS,bus=1
```

- **PASS** → the STM32 FDCAN peripheral and this tool's driver/dispatch code
  are both working correctly. A silent bus is therefore a hardware/wiring
  question from that point outward: transceiver power, CAN-H/L wiring,
  termination, common ground, or the far-end device itself.
- **FAIL** → something's wrong before the pins even matter — worth
  double-checking the peripheral clock config (`STM32_FDCANSEL`/`PLL2` in
  `cfg/mcuconf.h`) and, for bus 2 specifically, that `boards/CubeOrangePlus/board.c`
  actually configures `GPIOB` pins 6/12 for the FDCAN2 alternate function (this
  was missing until it was added alongside this self-test — see git history).
