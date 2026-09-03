# Full motor/leg/power telemetry + state estimation fusion

## Context

`controls_plan.md` already committed to part of this: a `StateIdx` extension
for averaged leg length/pitch (§2), a `FiveBarIK` module design (§3), and a
recommendation to wire leg state into `StateManager` via a second entry
point (§4) — all still marked "pending" in that file's own status line. The
wheel-only half of EKF velocity fusion (`EKF::update_wheel_velocity` feeding
`U`) is already built and working.

This plan finishes those pending pieces **and** goes beyond them, per this
session's request: individual hip/wheel telemetry at control-loop rate,
per-leg (not just averaged) FK output with Cartesian velocity components,
fusing *both* legs and wheels into *both* `U` and `W` (not just `U`),
battery voltage from averaged hip telemetry, total current from the Matek
power monitor, and one consolidated place to read all of it for logging and
future control use.

Two real, independently-verified problems surfaced during research and are
fixed as part of this work, not just designed around:

1. **Wheel encoder telemetry is currently passive-only.** Nothing in this
   firmware has ever sent an RTR (remote-request) frame — confirmed by
   `grep -rn RTR src/` returning exactly one hit, `CAN.cpp`'s
   `can_send()` hardcoding `txf.common.RTR = 0`. Wheel `pos_rad`/`vel_rads`
   update only whenever the ODrive happens to broadcast
   `Get_Encoder_Estimates` on its own internally-configured schedule — a
   rate this codebase doesn't control and may not currently be anywhere
   near 400 Hz. **Decision (confirmed with the user): fix this with active
   RTR polling**, not by relying on external ODrive config.
2. **`CANPower.cpp`'s DroneCAN decode has a real byte-offset bug.** DroneCAN
   multi-frame transfers prepend a 2-byte little-endian transfer CRC to the
   first frame, before the actual payload. The existing decoder doesn't
   account for this, so today's "`voltage_V`" is actually decoding the
   *temperature* field, "`current_A`" is actually decoding *voltage*, and
   real current is never read at all (it spans the frame-1/frame-2
   boundary, and continuation frames are explicitly skipped). Verified
   against `~/Documents/ardupilot`'s actual DroneCAN source (this Matek
   module ships stock AP_Periph firmware) and the public DroneCAN spec.
3. **The Matek is very likely not transmitting real telemetry at all yet.**
   It defaults to `CAN_NODE=0` (dynamic node allocation) and this firmware
   has no DNA server. The 2 Hz extended-ID frame observed on bus 2 during
   this session is much more likely the module's own periodic "give me a
   node ID" allocation-request retry than `BatteryInfo` (which AP_Periph's
   firmware publishes at ~10 Hz, not 2 Hz, and only once allocated). See
   item B below for the concrete fix.

---

## Architecture

Two containers, not one — the existing 19-dim `g_state[]` (`StateIdx`) is
explicitly the *Kalman-filtered* output (per its own header comment); most
of what's being added here is directly computed/measured, not something
that benefits from Kalman fusion.

- **`StateIdx` (`src/RobotState.hpp`), extended per `controls_plan.md` §2**:
  indices 19-22 = `LEG_L, LEG_L_DOT, LEG_PITCH, LEG_PITCH_DOT` — **averaged
  across both legs**, `N` becomes 23. This is the piece
  `LqrBalanceController`'s future gain scheduling needs at full
  `StateEstThread` rate (500 Hz), and it's what the already-written
  `controls_plan.md` §5/§6 design reads (`state[StateIdx::LEG_L]` etc.) —
  implementing it as literally speced keeps that prior plan intact rather
  than contradicting it.
- **New `RobotTelemetry` (`src/RobotTelemetry.{hpp,cpp}`)**: everything else
  requested that isn't a Kalman state — per-leg (not averaged) length/rate/
  Cartesian foot velocity, individual hip angle+velocity ×4, individual
  wheel velocity ×2, battery voltage (hip average), Matek voltage + total
  current, world-frame `x_dot`/`z_dot` (derived by rotation, not a new
  Kalman state). RC stick inputs (`g_input[]`/`InputIdx`) and robot mode
  (`RobotStateMachine::mode()`) are **not duplicated** here — they already
  exist and are already at full rate; this plan just notes them as
  already-satisfied.

Populated once per `StateEstThread` tick (500 Hz) — same "compute once,
publish under a mutex, everyone reads a snapshot" pattern `g_state`/
`g_input` already use — so `ControlThread` (400 Hz) always has fresh-enough
data for anything gain-scheduling or VMC needs later.

---

## A. Stale constant fix — do first, independent of everything else

**File:** `src/state_estimator/StateManager.hpp`

`STATEMGR_WHEEL_RADIUS_M` is `0.070f`; `MatLab_controls/wheeled_biped.m`'s
current (recently updated) `p.R = 0.092`. The comment on the constant
already says "must match wheeled_biped.m's p.R" — it just wasn't updated
when the MATLAB file was. This alone is a ~31% error in every
wheel-derived velocity estimate today. Fix independent of the rest of this
work; it's a one-line change that improves the *existing* fusion
immediately.

---

## B. `CANPower.cpp` — fix the DroneCAN decode, note the node-ID prerequisite

**File:** `src/coms/CANPower.cpp` (and header comment in `.hpp`)

**The fix.** DroneCAN `BatteryInfo` (DTID 1092) fields, in order:
`temperature`(float16,K) `voltage`(float16,V) `current`(float16,A)
`average_power_10sec`(float16,W) ... (verified against
`~/Documents/ardupilot`'s `modules/DroneCAN/DSDL/uavcan/equipment/power/1092.BatteryInfo.uavcan`
and the generated `dsdlc_generated` header). On the wire, frame 1 of a
multi-frame transfer is `[CRC_lo, CRC_hi, payload[0..4], tail]` — the first
2 bytes are a little-endian CRC-16-CCITT-FALSE of the whole payload
(seeded with the message's 64-bit data-type signature), **not** payload
data. So:
- `wire[0..1]` = CRC (currently misread as temperature)
- `wire[2..3]` = real `temperature` (currently misread as `voltage_V`)
- `wire[4..5]` = real `voltage` (currently misread as `current_A`)
- `wire[6]` = low byte of real `current` — not read at all today
- current's high byte is the **first byte of frame 2**, which the existing
  code explicitly skips (`if (!(tail & 0x80U)) return;`)

Fix: skip the 2 CRC bytes on the start-of-transfer frame, correctly assign
`temperature`/`voltage` from wire bytes 2-5, and reassemble through frame 2
(tail byte: bit7=SOT, bit6=EOT, bit5=toggle, bits4-0=transfer ID —
toggle starts 0 on frame 1 and alternates; verify transfer ID stays
constant across frames of one transfer before trusting reassembled bytes)
to capture current's high byte from the first byte of frame 2. Frame 2's
own tail byte will have EOT set once `average_power_10sec` etc. are the
only fields left (not needed) — stop reassembling once `current` is
complete, no need to parse further into `remaining_capacity_wh` etc.
`f16_to_f32()` (already implemented, correct) applies to all three fields
the same way.

**Node-ID prerequisite (do this or nothing above matters).** The Matek
defaults to `CAN_NODE=0` (dynamic allocation); this firmware has no DNA
server, so an unallocated module never transmits `BatteryInfo` at all.
Recommended: configure the module with a **static, nonzero `CAN_NODE`**
once via the DroneCAN GUI Tool / Mission Planner over a USB-CAN adapter —
a one-time device-config action, no firmware change, and it sidesteps
needing to implement DroneCAN's allocation sub-protocol in this codebase.
(A minimal DNA responder is possible if you'd rather never need a laptop
step, but it's real added scope — a unique-ID exchange state machine — not
included here unless you want it added.) Whatever the 2 Hz frame observed
this session actually is, this fix is the same either way — it doesn't
depend on diagnosing that frame further, though checking it is cheap: its
EID should NOT match the `0x00044400`/`0x00FFFF80` `BatteryInfo` mask this
project's registration already uses if it's the allocation broadcast
instead.

---

## C. Wheel telemetry rate — active RTR polling

**Status update (2026-09-02, on real hardware): implemented, bench-tested,
then DISABLED.** The bandwidth sanity check below correctly flagged this as
a real risk, not a hypothetical one. What actually happened: enabling
per-tick RTR polling (2 extra blocking `can_send_rtr()` calls every 2.5 ms
`ControlThread` tick) coincided with hip motors 2-4 losing live
responsiveness. A bus scan showed hip1 (first in the per-tick send order,
highest CAN arbitration priority — lower numeric SID wins arbitration)
getting far more traffic through than hips 2-4, and total observed bus
throughput far below what 1 Mbit/s should sustain — the signature of CAN
TX-mailbox contention inside `ControlThread` itself (too many blocking
`can_send`/`can_send_rtr` calls competing for a handful of FDCAN TX
mailboxes within one tick), not raw bus bandwidth or a decode bug.
Separately, a passive bus scan via Motor_tool (zero RTR requests, before
this code existed) already showed `Get_Encoder_Estimates` broadcasting
natively at a healthy rate on its own — confirming the **fallback** noted
below was not just viable but already the dominant real source of wheel
telemetry. Given the RTR requests were both the likely cause of the
regression AND redundant, the calls were removed from `ControlThread`
(`can_motor_request_encoder()` itself is left in `CANMotor.cpp`/`.hpp` for
future on-demand use, just not called from the 400 Hz hot path). Wheel
telemetry now relies entirely on the drive's own periodic broadcast, same
as the fallback path always intended.

**Files:** `src/coms/CAN.hpp`/`.cpp`, `src/coms/CANMotor.hpp`/`.cpp`,
`src/threads.cpp`

1. **`CAN.cpp`**: `can_send()` currently hardcodes `txf.common.RTR = 0`.
   Add an RTR-capable variant — either a new parameter (`bool rtr = false`)
   or a small `can_send_rtr(bus, sid, timeout_ms)` wrapper (no data payload
   needed for a remote-request frame; DLC still indicates expected reply
   length per the ODrive CAN Simple convention — set it to 8 to match
   `Get_Encoder_Estimates`' actual reply size).
2. **`CANMotor.cpp`**: new function, e.g. `can_motor_request_encoder(uint8_t id)`
   — RMD-only-style gate but for `CAN_MOTOR_ODRIVE` entries: sends an RTR
   frame at `odrive_arb_id(wire_id, ODRIVE_CMD_GET_ENCODER_EST)`. Existing
   RX registration (`bprl_can_register(..., odrive_arb_id(wire_id,
   ODRIVE_CMD_GET_ENCODER_EST), odrive_rx_cb, ...)`) already dispatches the
   reply correctly — no RX-side change needed, only a new TX call.
3. **`threads.cpp` (`ControlThread`)**: call `can_motor_request_encoder(5)`
   and `can_motor_request_encoder(6)` once per tick, right alongside the
   existing torque-command loop. **Make the divisor configurable** (a
   constant, not hardcoded to "every tick") so the rate can be dialed back
   if bus load proves too tight — see bandwidth note below. Start at every
   tick (400 Hz) and back off only if the empirical check says to.

**Bandwidth sanity check (rough, verify empirically before trusting it):**
at 1 Mbit/s, an 8-byte standard data frame costs roughly ~130 µs including
worst-case bit-stuffing overhead; an RTR frame (no data payload) costs
roughly ~55 µs. Bus 1 today, per 2.5 ms control tick: 4 RMD commands + 4
RMD replies + 2 ODrive commands ≈ 10 data frames ≈ 1.3 ms (~53% of the
tick). Adding 2 RTR requests + 2 encoder-estimate replies ≈ +0.38 ms →
~68% of the tick. Workable but not hugely comfortable headroom — **verify
with `CAN,diag` (Motor_tool or the firmware's own USB command) before and
after enabling this**, and if bus-1 shows sustained near-saturation or
torque-command delivery starts looking marginal, use the rate divisor from
step 3 to poll wheels at 200 Hz instead of 400 Hz (still far better than
today's uncontrolled rate) rather than risk crowding out torque commands.

**Fallback if RTR turns out unsupported by this ODrive firmware version**
(cheap to check on the bench first): fall back to raising the ODrive's own
`can.*` broadcast-rate config externally instead — no firmware change to
`CAN.cpp` needed in that case, just skip step 1 and rely on passive receipt
at whatever rate the drive is configured for.

---

## D. `FiveBarIK` module (new)

**Files:** `src/kinematics/FiveBarIK.hpp`, `src/kinematics/FiveBarIK.cpp`

Port from `MatLab_controls/wheeled_biped.m` (the leg-geometry section),
essentially verbatim — closed-form, no iteration:

```cpp
struct FiveBarParams { float l1, l2, l3, l4, l5; };   // rear thigh, rear shin, front shin, front thigh, hip spacing
struct FiveBarPose   { float L0, thL; };                // virtual leg length [m], leg angle [rad]
struct FiveBarVel    { float L0_dot, thL_dot; };         // from the Jacobian, see below
struct FiveBarJac    { float m[2][2]; };                 // d[L0;thL]/d[phi1;phi4]

bool fk(const FiveBarParams& p, float phi1, float phi4, FiveBarPose& out);
bool ik(const FiveBarParams& p, float L0, float thL, float& phi1, float& phi4);
FiveBarJac jac(const FiveBarParams& p, float phi1, float phi4);
FiveBarVel jac_to_vel(const FiveBarJac& J, float phi1_dot, float phi4_dot);
```

**FK** ("lower branch", from `wheeled_biped.m`):
```
A = (-l5/2, 0), E = (+l5/2, 0)         hip pivots
B = A + l1*(cos phi1, sin phi1)         driven by phi1 (rear thigh)
D = E + l4*(cos phi4, sin phi4)         driven by phi4 (front thigh)
C = circle(B,l2) ∩ circle(D,l3), lower-y branch     foot / wheel-axle point
L0  = |O - C|,  O = midpoint(A,E)
thL = atan2(x_C, -y_C)                  thL=0 straight down, thL>0 = foot forward
```
Return `false` (no solution) if the two circles don't intersect — guards
against out-of-range joint angles.

**IK** (also closed-form): `C = (L0*sin(thL), -L0*cos(thL))` in the
solver's own local frame, then `B = circleIntersect(A,l1, C,l2, "left")`
(knee out the back), `D = circleIntersect(E,l4, C,l3, "right")` (knee out
the front), `phi1 = atan2(B-A)`, `phi4 = atan2(D-E)`.

**Jacobian**: `wheeled_biped.m` already has an *exact* symbolic Jacobian
(not finite-difference) — transcribe it directly from `Jfk_sym` in that
file's `getSymbolicModel()` if you want bit-for-bit match; a hand-derived
or finite-difference C++ version is an acceptable fallback, regression-test
either against the MATLAB numbers.

**Cartesian foot-point velocity** (new — not in `controls_plan.md`'s
original minimal API, needed for this session's per-leg `x_velo`/`z_velo`
request and for the EKF fusion in item F). Convert `(L0_dot, thL_dot)` to
the foot point's velocity **relative to the body, in NED convention
(Z positive down)**:
```
x_C =  L0 * sin(thL)
z_C =  L0 * cos(thL)              // NOTE: +cos, not -cos — this is the
                                    // NED-Z-down sign flip from the
                                    // solver's internal Y-up local frame;
                                    // VERIFY on the bench (see Open Items)
x_C_dot =  L0_dot*sin(thL) + L0*thL_dot*cos(thL)
z_C_dot =  L0_dot*cos(thL) - L0*thL_dot*sin(thL)
```

**Sign relation** (already verified/corrected in `wheeled_biped.m`):
absolute leg angle `theta = phi - thL` (body pitch minus hip-relative leg
angle) — needed when assembling `LEG_PITCH` for the `StateIdx` extension
in item E.

Unit-test against `wheeled_biped.m`'s own `selfcheck()` round-trip numbers
(FK→IK→FK should return the original angles; `jac` should match a
finite-difference check) — regression fixtures the MATLAB file already
provides.

---

## E. `StateIdx` extension — averaged leg states

**File:** `src/RobotState.hpp`

Exactly per `controls_plan.md` §2:
```cpp
constexpr int LEG_L         = 19;  // m,    both legs averaged
constexpr int LEG_L_DOT     = 20;  // m/s
constexpr int LEG_PITCH     = 21;  // rad,  NED convention (theta = phi - thL)
constexpr int LEG_PITCH_DOT = 22;  // rad/s
constexpr int N = 23;               // was 19
```
Update the `static_assert(StateIdx::N == 19, ...)` in
`StateManager::get_state()` (`src/state_estimator/StateManager.cpp`) to 23.

---

## F. `StateManager`/`EKF` — leg+wheel fusion into U and W

**Files:** `src/state_estimator/StateManager.hpp`/`.cpp`,
`src/state_estimator/EKF.hpp`/`.cpp`

### Hip-to-leg wiring (high confidence, verify once on the bench)

`main.cpp`'s own motor map — `ID1=Hip FL, ID2=Hip FR, ID3=Hip RL, ID4=Hip RR`
— directly gives the pairing: **left leg = {phi1(rear thigh)=id 3 (RL),
phi4(front thigh)=id 1 (FL)}, right leg = {phi1=id 4 (RR), phi4=id 2 (FR)}**.
Since `l1==l4` and `l2==l3` numerically, a `phi1`/`phi4` mixup specifically
produces a *mirrored* solution (right length, wrong angle sign) — a useful
bench discriminator: rotate one hip by hand, confirm the matching leg's
`thL` telemetry moves the expected direction.

### New `EKF` method — `update_leg_wheel_velocity()`

Physics (re-derived independently from rigid-body kinematics, confirmed to
match the existing wheel-only fusion's assumptions): for a point (the wheel
axle / foot point `C`) attached to a body rotating at pitch rate `Q` and
translating at body-frame velocity `(U, W)`, with the point's
body-frame-relative position `(x_C, z_C)` and body-frame-relative velocity
`(x_C_dot, z_C_dot)` from item D:
```
v_C_world_x = U + Q*z_C + x_C_dot
v_C_world_z = W - Q*x_C + z_C_dot
```
Rolling-without-slip gives `v_C_world_x = u_roll = wheel_sign * wheel.vel_rads * WHEEL_RADIUS_M`
directly from the wheel encoder. Ground contact (flat, stationary,
maintained) gives `v_C_world_z ≈ 0` as a constraint, not a direct sensor
reading. Solving both for the body-velocity terms gives the two pseudo-
measurements to fuse:
```
U_meas = u_roll - Q*z_C - x_C_dot
W_meas = Q*x_C - z_C_dot
```
Average across both legs (when both are valid) before fusing, same pattern
the existing wheel-only code already uses for averaging L/R wheels.
`Q` should be the previous tick's blended pitch rate (`_blended_q` — 2 ms
stale at 500 Hz, negligible) rather than reordering the existing pipeline.

```cpp
// EKF.hpp/.cpp — same pattern as update_wheel_velocity, extended to 2 rows
void EKF::update_leg_wheel_velocity(float u_meas, float w_meas, float Ru, float Rw)
{
    if (!_initialized) return;
    float H[2][N] = {};
    H[0][iU] = 1.0f;
    H[1][iW] = 1.0f;
    const float R_diag[2] = { Ru, Rw };
    const float innov[2]  = { u_meas - _x[iU], w_meas - _x[iW] };
    _update(2, H, R_diag, innov);
}
```
Placeholder noise constants (retune on the bench, same status as every
other `R_*` in this file): `R_LEG_WHEEL_U ≈ 5e-3` (m/s)², looser than the
existing wheel-only `R_WHEEL_VEL=2e-3` since this stacks FK/Jacobian noise
on top; `R_LEG_WHEEL_W ≈ 2e-2` (m/s)², deliberately loose — it's a soft
pseudo-measurement (ground-contact assumption), not direct sensing.

### `StateManager` changes

**Replace**, not duplicate, the existing wheel-only fusion step ("2.5" in
`update()`) — fusing wheel-only `U` *and* the new combined leg+wheel
`U`/`W` every tick would double-count the same wheel encoder information
and artificially over-shrink the filter's confidence. New method:

```cpp
void StateManager::update_legs_and_wheels(const CanMotorState hips[4], const CanMotorState wheels[2]);
```
Called from `StateEstThread` right after the existing `update(...)` call
(keeps the single-owner/single-thread invariant `EKF`/`StateManager` already
rely on — no new locking needed, same as today).

Per leg: run `fk()` on that leg's two hip readings (offset-corrected, see
Open Items), run `jac()`/`jac_to_vel()` on the velocities, compute
`(L0,thL,L0_dot,thL_dot,x_C,z_C,x_C_dot,z_C_dot)`, store into a new private
`LegState _leg[2]` member (exposed via a small `get_leg_state(int leg,
LegState& out) const` accessor for `RobotTelemetry` to read). Average both
legs' `(L0,L0_dot,theta,theta_dot)` — `theta = phi - thL` per item D's sign
relation, `phi` = current blended body pitch — into the values that get
written to `StateIdx::LEG_L`/etc. in `get_state()`. Compute `U_meas`/
`W_meas` per the formulas above (averaged across legs with valid wheel
data) and call `update_leg_wheel_velocity()` on every valid lane. **Graceful
degradation**: if no leg is valid but a wheel is, fall back to the old
wheel-only `update_wheel_velocity()` math (don't lose all velocity fusion
just because a hip encoder glitched) — this is the one place the deleted
step-2.5 logic should live on, as a fallback branch, not a parallel path.

Remember to update `get_state()` to also write `out[19..22]` from the new
averaged leg values, gated by the `N==23` static_assert from item E.

---

## G. `RobotTelemetry` (new)

**Files:** `src/RobotTelemetry.hpp`, `src/RobotTelemetry.cpp`

```cpp
struct RobotTelemetry {
    float leg_L[2], leg_L_dot[2];       // m, m/s        (0=left, 1=right) -- PER-LEG, not averaged
    float leg_thL[2], leg_thL_dot[2];   // rad, rad/s
    float leg_x_dot[2], leg_z_dot[2];   // m/s, foot-point velocity rel. to body
    bool  leg_valid[2];

    float hip_pos_rad[4], hip_vel_rads[4];   // ids 1-4, FL/FR/RL/RR
    float wheel_vel_rads[2];                  // ids 5-6, L/R

    float battery_voltage_V;         // average of 4 hips' voltage_V (0x9A telemetry) -- literal per user's ask
    float power_monitor_voltage_V;   // Matek CAN-L4-BM voltage -- independent cross-check, once item B lands
    float total_current_A;           // Matek CAN-L4-BM current -- once item B lands

    float x_dot, z_dot;              // m/s, world/NED-frame velocity -- derived, NOT a new Kalman state

    bool valid;
};
extern mutex_t         telemetry_mtx;
extern RobotTelemetry  g_telemetry;

void telemetry_update(const float state[StateIdx::N], const StateManager& state_mgr,
                       const CanMotorState hips[4], const CanMotorState wheels[2]);
```
`MUTEX_DECL(telemetry_mtx)` static init, matching `state_mtx`'s convention.

`telemetry_update()`: per-leg fields copied from `state_mgr.get_leg_state()`
(item F); hip/wheel fields copied directly from the `hips[]`/`wheels[]`
snapshots already being fetched for item F, no new CAN reads;
`battery_voltage_V` = mean of `hips[i].voltage_V` over `valid` hips;
`power_monitor_voltage_V`/`total_current_A` read from the existing
`g_power`/`power_mtx` (item B) — lock, copy, unlock; `x_dot`/`z_dot`: rotate
`state[U],state[V],state[W]` by the quaternion in `state[]` using the same
`quat_to_rot_body2ned()` helper `EKF::predict()` already uses internally
for position integration (no new math, just exposing an existing
computation as a standalone output):
```
x_dot = R[0][0]*U + R[0][1]*V + R[0][2]*W
z_dot = R[2][0]*U + R[2][1]*V + R[2][2]*W
```

**`threads.cpp` (`StateEstThread`)**: fetch all 4 hips (new — currently
only wheels are fetched here) alongside the existing wheel fetch, call
`state_mgr.update(...)` then `state_mgr.update_legs_and_wheels(hip_snap,
wheel_snap)` then `state_mgr.get_state(g_state)` (publish under
`state_mtx` as today) then `telemetry_update(g_state, state_mgr, hip_snap,
wheel_snap)` (publish under `telemetry_mtx`) — all still inside the 500 Hz
loop.

**Consumers**: `ControlThread` can snapshot `g_telemetry` under
`telemetry_mtx` each tick once `LqrBalanceController`'s gain scheduling
needs it (that controller rewrite itself is `controls_plan.md` §5/§6 —
out of scope here; this plan only guarantees the data exists at 400 Hz+).
`LogThread` — see item H.

---

## H. Logging

**Files:** `src/logging/LogMessages.hpp`, `src/threads.cpp` (`LogThread`)

New message IDs 0x0B-0x0F (0x0A is taken by `LIN`, 0x80 reserved for
`FMT`), sized the same way the existing table's comments compute sizes
(format ≤16 chars, labels ≤64 chars):

| ID | Name | Format | Labels |
|---|---|---|---|
| 0x0B | `LEGS` | `QHffffffff` | `TimeUS,Rate,L0,L0dot,ThL0,ThL0dot,L1,L1dot,ThL1,ThL1dot` |
| 0x0C | `LEGV` | `QHffff` | `TimeUS,Rate,Leg0Xdot,Leg0Zdot,Leg1Xdot,Leg1Zdot` |
| 0x0D | `MOTV` | `QHffffff` | `TimeUS,Rate,HipVel0,HipVel1,HipVel2,HipVel3,WheelVel0,WheelVel1` |
| 0x0E | `PWR`  | `QHfff` | `TimeUS,Rate,BattV,PmonV,TotalI` |
| 0x0F | `VNED` | `QHff` | `TimeUS,Rate,XdotWorld,ZdotWorld` |

(`LEGS`+`LEGV` split rather than merged — combined labels would be 91
chars, over the 64-char cap.) All five map 1:1 onto `RobotTelemetry`
fields. `LogThread` snapshots `g_telemetry` once under `telemetry_mtx`
(same pattern as the existing `g_state`/`g_input` snapshot at the top of
the loop), then five `logger.write(LOG_MSG_*, msg)` calls. Also extend the
existing `RCIN` message with one new `uint8_t mode` field (robot state-
machine mode, 0/1/2) — cheaper than a 6th message type for one byte, and
`RCIN` already logs everything else RC-related.

`OUTP` currently only logs the 4 hip torque commands, not the 2 wheel
torque commands — worth a follow-up fix but it's about *commanded* torque,
not the telemetry this plan is about; noted, not scoped here.

---

## Open items to verify on the bench (not blocking implementation, blocking *trust*)

1. Hip-to-leg pairing (item F) — high-confidence from naming, not yet
   physically confirmed.
2. `z_C` NED sign flip (item D) — flagged inline, verify against a known
   crouch height.
3. **Done (2026-09-02):** hip encoder zero-offsets now exist as
   `HIP_OFFSET_RAD[4]` in `src/coms/CANMotor.cpp` (hardcoded, see its "Hip
   zero-offset + safety bounds" section and `src/controllers/MotorTest.*`
   for the bench tooling this shipped alongside) — still all `0.0f`
   placeholders pending real measurement, so FK output still isn't
   physically meaningful yet, but the mechanism itself is in place.
4. `R_LEG_WHEEL_U`/`R_LEG_WHEEL_W` — untuned placeholders.
5. **Resolved (2026-09-02): RTR polling caused CAN TX-mailbox contention on
   real hardware** (hips 2-4 losing responsiveness) and was removed — see
   item C's status update. Wheel telemetry rate now depends entirely on the
   ODrive's own broadcast rate, which passive observation confirmed is
   already healthy; not further tuned.
6. Matek static node-ID configuration (item B) — external action, not
   firmware.

---

## Build order

```
A, B(bugfix), C ─── independent, low-risk, any order/parallel
                                                      D (FiveBarIK)
                                                        │
                                                        ▼
                                        E (StateIdx) ─┐
                                                        ├─▶ F (fusion) ─▶ G (telemetry) ─▶ H (logging)
                                     B(node-ID, external) ┘  (G needs B's real values to be meaningful)
```

## Verification

- Build (`make`) after each lettered item — this project's convention is a
  clean build with no new warnings at every step, not just at the end.
- `Motor_tool`'s `find odrive`/`poll wheels`-equivalent workflow (or a raw
  `CAN,diag` before/after) to confirm RTR polling actually raises wheel
  telemetry rate and to check bus-1 load (item C).
- `FiveBarIK` unit tests against `wheeled_biped.m`'s `selfcheck()` numbers
  (item D) before trusting any fused output built on top of it.
- Bench sign/direction checks per the Open Items above — do these before
  trusting `StateIdx::LEG_*` or `RobotTelemetry`'s leg fields for anything
  control-facing.
- Once B's node-ID prerequisite is resolved, confirm `POWER,status` (the
  existing USB debug command) shows plausible voltage/current, then confirm
  the new `PWR` log record matches it.
