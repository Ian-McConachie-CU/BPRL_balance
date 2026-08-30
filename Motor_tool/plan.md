# CAN Bring-Up Test Plan

Goal: get all 6 motors (4× LK-TECH MG8016E-i6 @ CAN ID 1–4, 2× SteadyWin
GIM6010-6 @ CAN ID 10–11) working reliably over CAN on this Motor_tool
firmware, starting from "each motor already verified individually over
UART/RS485 and configured for CAN via its own GUI." See `CAN_config.md` for
the underlying protocol reference this plan cites commands from.

Ground rules for every phase below:
- **One new variable at a time.** Don't add a second motor to the bus, or
  move from a status read to a motion command, until the previous step is
  clean. Most CAN bring-up problems (wrong ID, reply-ID assumption, wiring)
  are far easier to isolate with 1 device talking than 6.
- **Non-motion commands before motion commands, always.** Status/fault/read
  commands can't hurt anything — exhaust those before sending torque/speed/
  position on a given motor.
- Keep `CAN,monitor,start` (or the Python tool's `monitor`) running in a
  second terminal/window throughout bring-up — seeing the raw frames is the
  fastest way to tell "no reply" apart from "reply on an ID I'm not
  expecting."
- Rotor free to spin (unloaded, off the robot / off the ground) for every
  motion test until Phase 6. Mounted/loaded testing is a separate, later
  step.

---

## Phase 0 — Bench and safety setup

1. Confirm 41 V bench supply has a sane current limit set (not full rail) —
   this is what actually protects hardware during bring-up, not the
   firmware's software clamps alone.
2. Confirm physical E-stop / supply disconnect is reachable before anything
   is powered.
3. Wire CAN bus 1 (FDCAN1) backbone with 120 Ω termination at the two
   physical ends only (see `CAN_config.md` → Cross-cutting notes). Leave all
   6 motor drives *disconnected from power* for now — CAN bus wiring can be
   done cold.
4. Confirm each drive's DIP switches:
   - LK-TECH ×4: DIP set to `0` (so the software-configured ID 1–4 actually
     takes effect) unless you're deliberately using DIP-encoded IDs — the
     DIP overrides the GUI setting when non-zero.
   - Confirm each LK-TECH drive's "R" (4th DIP) termination bit is only
     enabled on physical end-of-bus units, not all four.
5. Flash Motor_tool firmware (`make flash PORT=/dev/ttyACM0` or
   `make flash-stlink`) to the CubeOrangePlus bench board. Confirm USB
   enumerates (`PING` → `PONG` via the Python tool or a serial terminal).

## Phase 1 — Firmware/MCU self-check (no motors powered)

Do this before connecting any drive — it separates "the STM32 FDCAN
peripheral and this firmware are fine" from "something downstream is wrong,"
so later silence on the bus has one less possible cause.

```
motor_tool> bus 1
motor_tool> selftest
CAN,SELFTEST,PASS,bus=1
```

If this fails, stop — fix the peripheral/clock config (see README's
troubleshooting note on `STM32_FDCANSEL`/`PLL2` and FDCAN2 GPIO AF) before
touching any motor. A pass here means any future silence on bus 1 is a
wiring/drive-side question, not a firmware question.

## Phase 2 — First LK-TECH motor alone (ID 1)

Power only Hip 1's drive (41 V) with just that one node on the CAN bus.

1. `CAN,scan,start` on bus 1 — confirm you see traffic/an ID appear only
   when you poke it (LK-TECH drives don't transmit unsolicited).
2. `RMD,1,STATUS` — requests 0x9A (read state1). Confirm a reply arrives
   with sane temp/voltage (~41 V ± a bit) and `err=0`. This alone validates:
   correct ID, correct reply-ID assumption (already confirmed for LK-TECH,
   but re-verify per physical unit), wiring, termination, power.
   - If no reply: check `CAN,monitor` for *any* traffic (dead silence with a
     Phase-1 selftest pass points at wiring/power/ID mismatch, not
     firmware); double check DIP=0 and GUI-configured ID actually = 1.
   - If `err != 0`: decode via the errorState bit table in `CAN_config.md`
     (bit0 = under-voltage, bit3 = over-temp) before proceeding.
3. `RMD,1,CLEARERR` if any latched error, then re-`STATUS` to confirm clear.
4. `RMD,1,TORQUERAW,0` — zero-torque command, motor should stay put. Confirm
   the reply decodes to non-garbage pos/speed/torque (near-zero speed and
   torque, plausible position). This is the first motion-class command but
   at zero magnitude — validates the 0xA1 payload path without any risk.
5. `RMD,1,TORQUERAW,<small value>` (start with the smallest ratio that
   produces visible motion, well under the 33 A MG-series ceiling) with the
   rotor free — confirm it turns, direction matches expectation, and it
   stops turning when you send `RMD,1,STOP`.
6. `RMD,1,OFF` then `RMD,1,RESUME` — confirm the enable/disable cycle works
   before moving on.

Only proceed to Phase 3 once ID 1 is clean on all of the above.

## Phase 3 — Add remaining LK-TECH motors one at a time (IDs 2, 3, 4)

For each of ID 2, then 3, then 4 — power up and connect **one additional**
drive to the already-working bus:

1. Repeat Phase 2 steps 2–6 for the new ID only.
2. Then re-run `RMD,<id>,STATUS` for **every** ID connected so far in quick
   succession (e.g. via the Python tool's `poll` command, or scripted) and
   confirm each reply is correctly attributed to its own ID — this is the
   check that actually catches an ID collision (two drives sharing an ID
   looks fine individually but garbles when both reply at once).

By the end of Phase 3: all 4 LK-TECH motors individually verified for
status read, zero-torque, small motion, stop, and simultaneous polling with
no cross-talk.

## Phase 4 — First GIM motor alone (ID 10)

Power only Wheel 1's drive (41 V), added to the now-4-motor bus.

1. `GIM,10,FAULT` (0xB2) first — pure status read, safest possible first
   contact. Confirm a reply arrives. This is also where the **unconfirmed
   GIM reply-ID assumption** gets its first real test (see `CAN_config.md`)
   — if this is silent while LK-TECH replies are fine, suspect the reply-ID
   assumption before suspecting wiring.
   - If genuinely silent: use `CAN,monitor` to look for a GIM reply landing
     on an *unexpected* ID before concluding it's a wiring fault.
2. `GIM,10,IND,0` (bus voltage indicator) — confirm it reads back ~41 V.
   `GIM,10,IND,2` (motor temperature) — confirm plausible ambient reading.
   These two both round-trip the 0xB4 path without moving anything.
3. If `FAULT` returned nonzero: `GIM,10,ACKFAULT`, then re-check `FAULT`
   is clear. If it won't clear or immediately re-faults, check Over/Under
   Voltage Threshold config against the 41 V supply per the note in
   `CAN_config.md` (§ Cross-cutting notes) before going further.
4. `GIM,10,START` (0x91) — required before any motion command will do
   anything, per both the vendor spec and this firmware's own safety gate.
5. `GIM,10,TORQUE,0` — zero-torque, confirm reply decodes to sane
   temp/pos/speed/torque (near-zero).
6. `GIM,10,TORQUE,<small Nm>` with rotor free — confirm direction and that
   it responds to `GIM,10,STOP` (0x92, exits running state) or
   `GIM,10,PAUSE` (0x97, halts current command but stays running).
7. Note: torque feedback decode needs the motor's real torque constant and
   gear ratio (`GIM,KT` / `GIM,GEAR`) to read accurately — the position and
   raw motion behavior are valid without this, but don't trust the N·m
   feedback number yet. Command-side N·m is real from the start.

## Phase 5 — Second GIM motor (ID 11)

Repeat all of Phase 4 for ID 11, then poll both GIM IDs together (Python
tool `poll`) to rule out ID collision, same as the Phase 3 cross-check.

## Phase 6 — Full 6-motor bus integration

1. All 6 motors powered, all on bus 1. Run the Python tool's `poll` command
   (hits `RMD,<id>,STATUS` for 1–4 and `GIM,<id>,FAULT` + a couple of
   indicators for 10–11) and confirm the `STATUS` table populates correctly
   for all six with plausible values and no stale/garbled entries.
2. Stress the shared bus a little: issue small motion commands to all 6 in
   quick succession (e.g. loop `RMD,1..4,TORQUERAW,<small>` then
   `GIM,10/11,TORQUE,<small>`) and confirm no dropped replies / bus errors
   show up in `CAN,diag` counters, compared to a baseline snapshot taken
   before this phase.
3. **Watchdog check:** with a motor commanded to a nonzero torque/speed,
   stop sending commands entirely (or unplug USB) and confirm the firmware's
   500 ms host watchdog actually zeroes/disables every motor. This is a
   safety-critical behavior to verify once per bring-up session, not just
   trust from the README.

## Phase 7 — Loaded / mounted verification (once mechanically assembled)

Only after Phases 0–6 are clean:

1. Re-run the zero-torque and small-torque checks per motor with the
   mechanism assembled/loaded — confirm no direction reversal or unexpected
   binding compared to the free-spinning tests.
2. Calibrate the LK-TECH Nm placeholder scale (`RMD,SCALE,<ratio_per_Nm>`)
   against a known load or reference torque measurement — this value is
   currently a guess per the README/`CAN_config.md`.
3. Read and set the GIM torque constant/gear ratio (`GIM,KT`, `GIM,GEAR`) —
   either from the motor's Retrieve Configuration (ConfID 0x03 float / 0x11
   int, see `CAN_config.md` §2.5) or the SteadyWin GUI, so torque *feedback*
   (not just commands) is trustworthy before relying on it in a controller.
4. Only after 1–3: proceed to whatever closed-loop balance/drive testing is
   the actual end goal for this hardware — that's out of scope for this
   plan, which only covers getting CAN comms themselves solid.

---

## Quick reference — command cheatsheet used above

| Purpose | LK-TECH (RMD) | SteadyWin (GIM) |
|---|---|---|
| Status/fault read | `RMD,<id>,STATUS` | `GIM,<id>,FAULT` |
| Clear latched error | `RMD,<id>,CLEARERR` | `GIM,<id>,ACKFAULT` |
| Enable | `RMD,<id>,RESUME` | `GIM,<id>,START` |
| Disable | `RMD,<id>,OFF` | `GIM,<id>,STOP` |
| Zero/hold | `RMD,<id>,STOP` | `GIM,<id>,PAUSE` |
| Zero-torque test | `RMD,<id>,TORQUERAW,0` | `GIM,<id>,TORQUE,0` |
| Small motion test | `RMD,<id>,TORQUERAW,<n>` | `GIM,<id>,TORQUE,<Nm>` |
| Runtime indicator | — | `GIM,<id>,IND,<ind_id>` |
| Bus self-test | `CAN,selftest` | `CAN,selftest` |
| Raw sniff | `CAN,monitor,start` | `CAN,monitor,start` |
