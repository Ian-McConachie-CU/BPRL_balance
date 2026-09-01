# Motor_tool: Status, Tool Reference, and Testing Plan

## Current status (2026-08-31)

Every device has been individually confirmed working over CAN at some point
this bring-up. All hardware is now physically connected simultaneously for
the first time:

| Device | Bus | Addressing | Status |
|---|---|---|---|
| Hip 1–4 (LK-TECH MG8016E-i6) | 1 | CAN ID 1–4, reply on same ID (`0x140+ID`) | Confirmed individually and together; **currently affected by the open bus-1 issue below** |
| Wheel 1 (SteadyWin GIM6010-6) | 1 | CAN ID 15, **reply on ID 16** (Host/Master CAN ID; moved from 10/11) | Currently affected by the open bus-1 issue below |
| Wheel 2 (SteadyWin GIM6010-6) | 1 | CAN ID 20, **reply on ID 21** | Same as Wheel 1 |
| IMX5 IMU | 2 | Standard IDs 0x01–0x04, passive stream (no request needed) | Confirmed working throughout |
| Current sensor | 2 | Unknown | **Not supported by this tool** — no protocol reference obtained yet, see below |

**Open issue (2026-08-31, unresolved):** every RMD *and* GIM request now
fails at the firmware TX level (`can_send()`/`canTransmitTimeout()` itself
returning `ERR`, not just "no reply") — 16/16 RMD sends failed identically
across two consecutive `poll hips` runs, right after GIM showed the same
signature in the prior session. Since this now affects **both** drivers,
which don't share any command code, the original "IMU traffic preempting
the GIM command thread" hypothesis is superseded — this looks like a
**bus-1-wide condition**, most likely bus-off (accumulated TX errors →
the CAN controller refuses to transmit at all, to any ID, until reset or
recovered), plausibly triggered by a wiring/termination change when all 6
motors were connected together for the first time. `selftest` on bus 1 was
requested as the next diagnostic (internal loopback, no external wiring
dependency, and resets the peripheral as a side effect — would clear a
bus-off condition if that's what this is) but the result hadn't come back
as of this note. **Do not trust "not working" reports for any RMD/GIM
command until this is resolved** — it isn't informative about that specific
motor while the whole bus is down.

---

## Tool inventory — what tests what hardware

### Bus-level / infrastructure (no specific device)

| Command | What it does |
|---|---|
| `selftest` | Internal loopback test of the *active* bus (`bus 1`/`bus 2` selects which) — proves the STM32 FDCAN peripheral and firmware are healthy, independent of any external wiring. Run this first when anything looks dead. |
| `scan [seconds]` | Passively counts distinct CAN IDs seen on the active bus. First thing to run against unfamiliar/new wiring — e.g. the current sensor. |
| `monitor [seconds]` | Live raw frame dump, **both buses** simultaneously (Ctrl-C to stop). The rawest possible view; everything else is built on top of this. |
| `send <id_hex> <ext0\|1> <b0>..<b7>` | Inject one raw 8-byte CAN frame. Use this to probe anything not wrapped by a higher-level command yet (e.g. the current sensor, once you know or are guessing at a command byte). |
| `bus <1\|2>` | Selects which bus `scan`/`selftest`/raw `send` target. RMD and GIM commands always target bus 1 internally regardless of this setting. |
| `status` | Passive — dumps everything currently cached: every hip and wheel that has ever reported, plus the current IMU reading. Doesn't trigger new requests itself (RMD/GIM only refresh via `poll` or a direct command). |
| `stop` | `STOP,ALL` — zero/disable every RMD and GIM motor immediately. Safety command, not a test. |

### Hips — LK-TECH MG8016E-i6 (bus 1, CAN IDs 1–4)

| Command | Tests |
|---|---|
| `poll [ids]` | Active status check across the whole fleet (defaults to hips 1–4 + both wheels), ending in the aggregate `status` table. Your default "is everything alive" command. |
| `poll hips [ids]` | **Raw-byte diagnostic.** Sends 0x9A (status), 0x90 (encoder), 0x30 (PID), 0x33 (acceleration) and shows exactly what came back per ID, with an independent decode cross-checked against the firmware's own. Start here for anything hip-related that looks wrong — this is what found and fixed the voltage and encoder-scale bugs. |
| `test hips [ids]` | **Moves** a hip (one at a time, confirms first) to 0° then 120°, single-turn absolute (0xA6). Real motion. |
| `test drift [ids]` | Passive — samples the encoder for 5s with the motor left undisturbed, reports whether the reading moves at rest. |
| `encoder <id> [s] [ratio]` | Live gearbox-corrected (÷6 by default) position feed, printed continuously — turn the shaft by hand and watch it track in real time. |
| `rmd <id> torque\|torqueraw\|vel\|pos\|singleturn\|increment\|stop\|off\|resume\|status\|clearerr\|encoder` | Direct low-level commands, one motion/read primitive at a time. `pos` is absolute *multi-turn*; `singleturn` is absolute within one revolution (matches what `encoder` reports); `increment` is relative to wherever it currently is. |
| `rmd scale [ratio_per_Nm]` | Get/set the Nm→ratio scale `rmd torque` uses — still an uncalibrated placeholder, see `CAN_config.md`. |

### Wheels — SteadyWin GIM6010-6 (bus 1, CAN ID 15/20, **reply ID 16/21**)

| Command | Tests |
|---|---|
| `poll [ids]` | Same fleet check as above. Automatically pushes the CAN-ID→reply-ID mapping (`ensure_gim_reply_ids`) before polling, so this now works correctly with the confirmed differing reply IDs. |
| `poll gim [ids]` | Raw-byte diagnostic for GIM — watches **both** a wheel's CAN ID and its reply ID, decodes 0xB2 (fault) / 0xB4 (indicator) replies. |
| `poll wheels [pairs]` | Purpose-built for the CAN-ID-vs-reply-ID question specifically — shows a plain-language verdict per wheel ("replies land on Master CAN ID X, not CAN ID Y"). This is what found the reply-ID bug in the first place. |
| `gim <id> start\|stop\|pause\|torque\|velocity\|position\|fault\|ackfault\|ind\|masterid` | Direct low-level commands. `masterid` sets/queries the reply-ID override for that motor (`GIM,<id>,MASTERID,<reply_id>`) — this is what `ensure_gim_reply_ids` calls under the hood. |
| `gim limit [Nm]` / `gim kt [Nm/A]` / `gim gear [ratio]` | Global (not per-id) torque clamp and torque-feedback-decode constants. |

### IMX5 IMU (bus 2)

| Command | Tests |
|---|---|
| `imu [seconds]` | Decodes the passively-streamed quaternion/rate/accel — once, or watched live for N seconds. No request is sent; the IMU broadcasts continuously on its own, so this (and `status`) just read whatever's already arrived. |

### Current sensor (bus 2) — not supported yet

Nothing in this firmware decodes it — there's no protocol reference for it
in this project yet. What you *can* do today:
- `bus 2` then `scan 5` — see its arbitration ID show up distinctly from the IMX5's 0x01–0x04.
- `bus 2` then `monitor 10` — watch its raw payload bytes live.

That's the same starting point every other device here started from
(raw bytes first, decoder once the layout's confirmed against something
known). If you want this supported properly, the fastest path is whatever
datasheet/protocol doc the sensor vendor provides — hand it over and I'll
wire it up the same way as everything else in this tool. Failing that, we
can reverse it from raw bytes against a known reference reading (e.g.
compare its output to a multimeter/clamp-meter reading on the same rail),
the same way the RMD voltage field's real byte layout got confirmed.

---

## Full-system integration test (next actual step)

Everything is physically connected now — this is the check that's actually
new territory, since it's the first time all 6 motors + both bus-2 devices
have been live together.

1. `bus 1` → `selftest`, `bus 2` → `selftest` — confirm both FDCAN
   peripherals are healthy before adding real traffic to either.
2. `poll` (no args) — hits all 4 hips and both wheels with the now-correct
   default addressing (CAN IDs 1–4, 15, 20; GIM reply IDs 16/21 configured
   automatically). Confirm all 6 show up in the `status` table with
   plausible values and no stale/garbled entries.
3. `imu` — confirm the IMU is still decoding cleanly with the full 6-motor
   bus 1 also carrying traffic. Bus 1 and bus 2 are separate FDCAN
   peripherals, so bus-1 load shouldn't affect bus-2 decode directly, but
   worth confirming under real combined load rather than assuming it.
4. `status` — single aggregate view of every hip, wheel, and the IMU
   reading, all in one table. Make this your standard "is everything alive"
   check going forward now that the fleet is complete.
5. **Resolve the open bus-1 TX-failure issue first** (see Current status
   above) before trusting any other result in this list — as of the last
   update it affects RMD and GIM alike, so nothing else here is meaningful
   until `bus 1` → `selftest` has been run and the result checked. If it
   passes, move to physically checking bus-1 wiring/termination (changed
   when all 6 motors were connected together); if it fails, that's a
   firmware/peripheral question instead.
6. Stress check: issue a burst of small motion commands across all 6 motors
   in quick succession (e.g. `rmd 1..4 torqueraw 0`, `gim 10/20 torque 0`)
   and compare `CAN,diag` counters before/after for drops, now that the bus
   is at full expected load.
7. Watchdog re-check: with a motor holding a nonzero command, stop sending
   anything for >500ms and confirm the firmware's host watchdog zeroes
   every motor — same check as before, now worth repeating with the full
   fleet present in case per-motor `rmd_stop_all()`/`gim_stop_all()` takes
   meaningfully longer with 6 motors instead of 1–2.

---

## Quick reference — command cheatsheet

| Purpose | LK-TECH (RMD) | SteadyWin (GIM) |
|---|---|---|
| Status/fault read | `rmd <id> status` | `gim <id> fault` |
| Encoder/position read | `rmd <id> encoder` | (comes back with torque/speed/position commands only) |
| Clear latched error | `rmd <id> clearerr` | `gim <id> ackfault` |
| Enable | `rmd <id> resume` | `gim <id> start` |
| Disable | `rmd <id> off` | `gim <id> stop` |
| Zero/hold | `rmd <id> stop` | `gim <id> pause` |
| Zero-torque test | `rmd <id> torqueraw 0` | `gim <id> torque 0` |
| Absolute position (matches encoder frame) | `rmd <id> singleturn <rad> <maxspd>` | `gim <id> position <rad>` |
| Relative move | `rmd <id> increment <rad>` | — |
| Runtime indicator | — | `gim <id> ind <ind_id>` |
| Reply-ID override | — | `gim <id> masterid <reply_id>` |
| Bus self-test | `selftest` | `selftest` |
| Raw sniff | `monitor` | `monitor` |
| Raw-byte diagnostic | `poll hips [ids]` | `poll gim [ids]` / `poll wheels [pairs]` |

---

## Historical bring-up phases (completed — kept for reference)

Everything below is the original one-motor-at-a-time bring-up sequence this
project followed to get from "nothing confirmed" to the current status
table at the top. All phases are done; kept here since the reasoning (why
non-motion before motion, why one device at a time, what each check
actually validates) is still the right approach for bringing up *new*
hardware in the future, or re-verifying after a hardware change.

### Phase 0 — Bench and safety setup
1. Confirm the 41 V bench supply has a sane current limit set — this is
   what actually protects hardware during bring-up, not the firmware's
   software clamps alone.
2. Confirm physical E-stop / supply disconnect is reachable before
   anything is powered.
3. Wire the CAN bus 1 backbone with 120 Ω termination at the two physical
   ends only (see `CAN_config.md` → Cross-cutting notes).
4. Confirm each LK-TECH drive's DIP switches are at `0` (so the
   GUI-configured ID actually takes effect) and that the "R" termination
   bit is only enabled on physical end-of-bus units.
5. Flash Motor_tool firmware (`make flash PORT=/dev/ttyACM0`), confirm USB
   enumerates.

### Phase 1 — Firmware/MCU self-check (no motors powered)
`bus 1` → `selftest` before connecting any drive, to separate "the FDCAN
peripheral and firmware are fine" from "something downstream is wrong."

### Phase 2 — First LK-TECH motor alone (ID 1)
Non-motion reads (`rmd 1 status`) before any motion command
(`rmd 1 torqueraw 0`, then a small nonzero value), confirming direction,
stop, and the enable/disable cycle before moving on.

### Phase 3 — Remaining LK-TECH motors, one at a time (IDs 2–4)
Repeat Phase 2 per motor, then poll all connected IDs together to catch ID
collisions (which look fine individually but garble when multiple motors
reply at once).

### Phase 4 — First GIM motor alone (ID 10)
Non-motion reads first (`gim 10 fault`, `gim 10 ind 0/2`) — this is also
where the GIM reply-ID assumption first got tested (and, much later,
turned out to be wrong for this specific hardware — see `poll wheels`).

### Phase 5 — Second GIM motor (ID 20 — originally assumed ID 11)
Repeat Phase 4, then poll both GIM IDs together.

### Phases 6–7 — Full bus integration, then loaded/mounted verification
Superseded by the "Full-system integration test" section above, which
reflects the current, larger, more-capable tool set. Phase 7's calibration
notes (RMD torque scale, GIM Kt/gear ratio) are still open and unrelated to
bus integration — worth doing once before trusting torque *feedback*
(commands are already real) in any closed-loop controller.
