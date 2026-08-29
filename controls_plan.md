# Controls Implementation Plan

Turning `MatLab_controls/wheeled_biped.m` (5-bar IK, gain-scheduled LQR, verified
nonlinear sim) into the real onboard controller. This is a plan, not code —
it lays out the new files/state, the coordinate-frame contract between the
MATLAB model and the firmware, and the order to build it in.

**Status:** §6 (`BalanceController` + two sub-controllers) and the wheel-
encoder half of the EKF fusion are implemented — `PidBalanceController`
(SLC PID cascade, hips locked via `HipLock`) and `LqrBalanceController`
(stub, `K` all-zero) are both wired up and selectable at runtime via
`input[InputIdx::CTRL_SEL]`, and `StateManager`/`EKF` now fuse wheel-encoder
velocity as described in §1's `xdot ≈ U` note. `FiveBarIK` (§3), the
leg-angle `StateIdx` extension (§2/§4), and the real LQR gain table (§5)
are still pending — `LqrBalanceController` reads `theta`/`thetadot` as a
hardcoded 0 placeholder until those exist (see that class's header).

---

## 0. Bugs found while reviewing wheeled_biped.m (fixed / flagged)

While checking the sim's coordinate conventions against the firmware, two
real bugs turned up and got fixed in place (confirmed by `wb.selfcheck()`,
which now passes every section except one flagged below):

- **Syntax error** — `pargs(p){:}` (chaining `{}` directly onto a function
  call's `()` result) is invalid MATLAB at 8 call sites; the file could not
  run at all in real MATLAB. Fixed by assigning `c = pargs(p);` first.
- **Swapped `icare` outputs** — `lqrGain()` called `[K, P] = icare(A,B,Q,R)`,
  but `icare` returns `[X, K, L]` (Riccati solution *first*, gain *second*).
  Whenever Control System Toolbox was present, every LQR gain the file
  produced was silently wrong (`K` held the Riccati solution, not the
  feedback gain). Fixed to `[P, K] = icare(A, B, Q, R)`. This is the matrix
  that will eventually get compiled into firmware, so it needed to be right
  before anything downstream depends on it — reverified: CARE residual and
  cross-check against the toolbox-free `careSchur` fallback both pass,
  and the full nonlinear closed-loop recovery sim (section 9) now settles
  for every tested leg length / initial tilt.
- **Known-broken, not fixed:** `rigidLegModel()` / the `'rigid'` schedule
  mode (selfcheck section 5). Its closed-form mass matrix is only exact in a
  frame co-rotating with the body, not the world-Cartesian `x` used
  everywhere else in the file, so it doesn't match the general symbolic
  reduction for `phi != 0` — needs a real re-derivation, independent of the
  coordinate-convention rewrite below. **Not used by the default `'6state'`
  schedule**, which is what this plan builds on and which passes every
  check, so it doesn't block anything here — just don't reach for `'rigid'`
  mode or `rigidLegModel()` until it's independently fixed.
- **Coordinate convention rewritten to be NATIVE NED** (superseding an
  earlier version of this plan that used a `wb.toNed`/`wb.fromNed`
  conversion layer — that layer no longer exists). See §1.
- **The 5-bar Jacobian (`jac()`) is now exact, not finite-difference** —
  derived symbolically alongside the dynamics in `getSymbolicModel()` and
  evaluated via `matlabFunction`, same as everything else in the file. See §3.

---

## 1. Coordinate-frame contract (read this before writing any C++)

`wheeled_biped.m`'s state — `x`, `theta` (leg angle), `phi` (body pitch), and
their rates — is now defined **natively in the same convention the firmware
uses**: `+x` forward (matches NED `+X`), and `theta`/`phi` positive in the
same sense as the firmware's pitch (`quat_to_euler` in `src/math/math.cpp`,
standard aerospace ZYX Euler from `q_NED→Body`) — positive nose-up, i.e. for
this robot's geometry, the top of the leg/body tilting **backward**. There is
**no conversion layer** (an earlier version of this file used
`wb.toNed`/`wb.fromNed`; that's gone — the sim's own state IS the NED state
now, by construction, verified in `wb.selfcheck()` section 12). Wire these
straight across:

| Sim quantity | Real (NED / StateIdx) quantity | Relationship |
|---|---|---|
| `x` | `state[StateIdx::X]` | identical |
| `xdot` | `state[StateIdx::U]` | identical (see caveat below) |
| `phi` (body pitch) | firmware `pitch()` (`StateManager::pitch()`) | identical |
| `phidot` | `state[StateIdx::Q]` | identical |
| `theta` (leg angle) | new leg-pitch state (§2) | identical |
| `thetadot` | new leg-pitch-rate state (§2) | identical |

Caveat on `xdot ≈ U`: the sim's `xdot` is the wheel-contact point's inertial
forward velocity; `StateIdx::U` is the IMU/EKF's body-frame forward velocity.
For a balancing robot operating near-upright these coincide to first order;
if the controller ever needs to run at large sustained pitch angles, revisit
this (either feed `U` through a `cos(pitch)`-type correction, or accept the
error — TBD once real logs exist).

`T` (wheel torque) and `Tp` (lumped hip torque) are unaffected by any of the
above — splitting `Tp` into signed per-motor commands is a VMC/IK problem
with its own hardware-mounting sign convention (which leg is "left", which
way the motor spins for positive torque), not something the sagittal model
can resolve. That gets calibrated on the bench (§7).

The animation (`robotFrame`, used by `plot_response.m`) still draws a
normal, upright-looking picture for a human to look at — it negates
`theta`/`phi` once, purely where it converts state to screen pixels. That's
a rendering convenience local to `robotFrame`, not a second coordinate
system to track when wiring up the real controller.

---

## 2. New state: leg length, leg pitch, and their rates

`RobotState.hpp` already anticipates this ("Future: joint angles and joint
rates will be appended beyond index N (19)"). Add, in order:

```cpp
namespace StateIdx {
    // ... existing 0–18 unchanged ...
    constexpr int LEG_L      = 19;  // virtual leg length, both legs averaged   [m]
    constexpr int LEG_L_DOT  = 20;  // leg extension rate                      [m/s]
    constexpr int LEG_PITCH  = 21;  // leg angle, NED sign convention (see §1) [rad]
    constexpr int LEG_PITCH_DOT = 22;                                        // [rad/s]

    constexpr int N = 23;
}
```

`wheeled_biped.m`'s dynamics are a single lumped sagittal model (both legs'
masses combined into one virtual leg) — so a single averaged `(L, theta)`
pair per side-pair is the direct match, and is enough to drive the balance
controller in §6. Two real legs means two independent 5-bar FK solutions
(§3) that get averaged into this pair; keeping each leg's own `(L, thL)`
around too (not necessarily as StateIdx entries — could just be
locals/telemetry) is worth it for a future roll/turn extension, but the
6-state LQR from the sim has no use for it yet. Note this as a deliberate
scope cut, not an oversight, if it comes up later.

`static_assert(StateIdx::N == 19, ...)` in `StateManager::get_state()`
(`src/state_estimator/StateManager.cpp`) needs its literal bumped to 23
alongside this.

---

## 3. New IK/FK module: `src/kinematics/FiveBarIK.{hpp,cpp}`

Port `fk`, `ik`, `jointTorques`, `taskStiffness` from `wheeled_biped.m`
(§"5-bar linkage" section) essentially verbatim — closed-form circle
intersection, no iteration, no toolbox dependency, easy to unit test against
the MATLAB `selfcheck()` numbers (round-trip error, Jacobian consistency) as
regression fixtures. `jac()` itself is now an *exact* symbolic Jacobian
(derived alongside the dynamics in `getSymbolicModel()`, matlabFunction'd —
no finite differences), so if you want the ported C++ `jac` to match it
bit-for-bit rather than re-deriving by hand, the closed-form expression is
short enough to read straight off `Jfk_sym` in that function and transcribe;
otherwise a hand-differentiated or finite-difference C++ version is fine and
can be regression-tested against the MATLAB one's numbers either way.

```cpp
struct FiveBarParams {
    float l1, l2, l3, l4, l5;   // from wb.params(): rear thigh, rear shin,
};                              // front shin, front thigh, hip spacing

struct FiveBarPose { float L0, thL; };   // virtual leg length, leg angle (hip->foot, foot-fwd+)

FiveBarPose fk(const FiveBarParams& p, float phi1, float phi4);
void        ik(const FiveBarParams& p, float L0, float thL, float& phi1, float& phi4);
// jac, jointTorques as needed once the VMC split (§6) is written
```

Runs **per leg** (left, right), each fed that leg's own two hip-encoder
readings (`CanMotorState::pos_rad` for CAN ids 1–4, via
`can_motor_get_state()`, `src/coms/CANMotor.hpp`). Left/right FK outputs
`(L, thL)` get averaged and sign-converted (`theta = phi_ned_convention - thL`,
per the `theta = phi - thL` relation already derived and verified in
`wb.fk`/`robotFrame`) into the `LEG_L` / `LEG_PITCH` states.

---

## 4. Wiring leg state into StateManager

`StateManager::update()` currently only consumes IMU/CAN-IMU/mocap; it knows
nothing about the CAN motor bus. Two reasonable shapes, pick one:

- **(a)** Give `StateManager` a second entry point,
  e.g. `update_legs(const CanMotorState hips[4], float dt)`, called from
  `StateEstThread` right after `state_mgr.update(...)` — keeps EKF state and
  leg state under one owner (matches "adding it to the state manager"), lets
  `get_state()` fill indices 19–22 in the same call that fills 0–18.
- **(b)** A separate small `LegEstimator` class that `StateEstThread` calls
  independently and writes directly into `g_state[StateIdx::LEG_*]` —
  keeps `StateManager` purely EKF/IMU, avoids give it a CAN-bus dependency.

Recommend **(a)**: it's a smaller diff, it keeps the "one function fills the
whole state vector" invariant `get_state()` already documents, and
differentiating `L` for `LEG_L_DOT` / `theta` for `LEG_PITCH_DOT` can reuse
the exact same `derivative()` + `lowpass()` pattern StateManager already
uses for `p_dot`/`q_dot`/`r_dot` (`src/math/math.hpp`) — one more pair of
`_prev_*`/`*_filt` members, no new filtering machinery.

---

## 5. Gain table: compute offline, evaluate onboard

`wheeled_biped.m`'s `scheduleGains()` (Riccati solve + per-gain cubic fit
over a leg-length grid) needs the Symbolic Math Toolbox and a Riccati solver
— **do not port this to the MCU.** Instead:

1. Once `params()` reflects real measured/CAD values (masses, inertias, the
   5-bar link lengths — everything in `p.*` is currently a placeholder per
   its own comment), run `wb.schedule(p, Q, R, Lgrid, 3)` in MATLAB and take
   `sched.c` (the fitted polynomial coefficients), `sched.Lmid`, `sched.Lhalf`.
2. Export those as a generated C++ header, e.g.
   `src/controllers/LqrGainTable.hpp` — a `constexpr` coefficient array plus
   `Lmid`/`Lhalf`.
3. Port `evalGains()` itself (a `polyval` per gain entry — a few
   multiply-adds, nothing symbolic) to run at 400 Hz in `BalanceController`.

This keeps the expensive/toolbox-dependent part offline and one-shot, and
the onboard controller only ever does cheap polynomial evaluation + a
matrix-vector multiply — matching the file's own "one derivation, thin
numeric wrappers" philosophy, just split across the MATLAB/firmware
boundary instead of within one file.

---

## 6. BalanceController rewrite

Replace the stub body (`src/controllers/BalanceController.cpp`) with, per
400 Hz tick:

1. **Assemble the 6-state** from `state[]` + the new leg states — per §1
   there's no sign flip needed any more, it's a straight read:
   `x_sim = [state[X]; state[LEG_PITCH]; pitch(); state[U]; state[LEG_PITCH_DOT]; state[Q]]`.
2. **`K = evalGains(state[LEG_L])`** — the ported polynomial evaluator (§5).
3. **`u = -K * x_sim`**, saturate against wheel/hip torque limits
   (`p.tau_wheel_peak`, `p.tau_hip_cont` equivalents) → `[T, Tp]`.
4. **Wheel torque `T`** → CAN ids 5/6 (`can_motor_set_torque`), split evenly
   between the two wheels plus a differential term from
   `input[InputIdx::YAW_RATE]` for turning (yaw control isn't in the
   sagittal model at all — this is a separate, simple loop layered on top).
5. **Hip torque `Tp`** → split per leg via the VMC mapping
   (`jointTorques`/`jac` from §3), evaluated at each leg's *own* current
   `(phi1, phi4)` — nominally `Tp/2` per leg for the symmetric case, each
   leg's Jacobian then maps that into its two motor torques (CAN ids 1–4,
   FL/FR/RL/RR per `main.cpp`'s registration order).
6. **Height/crouch outer loop** — `L` is a *schedule parameter* in the sim,
   not a controlled state; the 6-state LQR balances attitude/position at
   whatever `L` currently is, it does not drive `L` toward a commanded
   height. A commanded leg length (from a new input, or a fixed default)
   needs its own light control action — e.g. a virtual leg force from
   `taskStiffness`/`jac` on `(L - L_cmd)`, superposed with the
   balance-derived per-leg torques before sending to the motors. This is a
   real design decision, not a mechanical port — needs its own gain and its
   own tuning pass, separate from the LQR schedule.

---

## 7. Bench validation before closed-loop testing

Before ever letting this balance:

- With the robot on a stand (wheels off the ground, legs free to move),
  command small positive `T` and `Tp` individually and confirm the physical
  motor directions match what the sagittal model assumes — this is exactly
  the "hardware-mounting sign convention" the MATLAB file explicitly
  disclaims responsibility for (§1). Get this wrong and the LQR's negative
  feedback becomes positive feedback.
- Verify FK/IK against physically measured hip angles at a few known crouch
  heights (cross-check against `wb.selfcheck()`'s round-trip numbers).
- Only then run the full loop, starting from the sim's already-verified
  gentle recovery cases (5–10° initial tilt, mid-range `L`) before pushing
  toward the sim's tested limits (20°, `L` ∈ [0.18, 0.32]).
- Before that, `wb.simulate`'s `opts.noiseStd` / `opts.delay` (rough
  IMU/encoder-class placeholders — see its help) are worth running against
  the same gain schedule to get a feel for how much noise/latency margin the
  current tuning has, before it meets the real, noisier StateManager output.
  `plot_response.m` demonstrates this live, plus tracking a commanded
  forward velocity rather than just regulating to standstill — useful as a
  template for exercising `input[InputIdx::THRUST]`-style velocity commands
  once `BalanceController` is real.

---

## 8. Suggested build order

1. `FiveBarIK` (§3) + unit tests against `wheeled_biped.m`'s own
   `selfcheck()` numbers.
2. `StateIdx` extension (§2) + `StateManager` leg-state wiring (§4).
3. Measure real `params()` values on the actual hardware; regenerate
   `wb.schedule()` and the gain table (§5).
4. `BalanceController` rewrite (§6) — start with `Tp`/`T` zeroed and just
   log the computed control to verify the state pipeline before touching
   motors.
5. Bench sign/direction validation (§7).
6. First closed-loop attempt, small tilt, robot restrained.
