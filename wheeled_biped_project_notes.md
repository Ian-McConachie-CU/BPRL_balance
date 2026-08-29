# Wheeled Biped: Simulation Goal, Robot Details, and Sources

## 1. Goal of the simulation

The purpose of `wheeled_biped.m` / `plot_response.m` is a **first-pass, quick-to-build
control model** for a 5-bar-legged wheeled biped, to validate a balance controller
architecture before committing to hardware or a full 3D physics simulation. Specifically:

- Derive a **planar (sagittal-plane) dynamic model** of the robot — wheel, leg, and
  body — with the equations of motion produced symbolically (Symbolic Math Toolbox)
  from a single Lagrangian, rather than hand-transcribed, so there is exactly one
  source of truth for the physics and no risk of two versions of the equations
  silently drifting apart.
- Derive **closed-form forward/inverse kinematics and the Jacobian** for the actual
  5-bar leg linkage, separately from the simplified "virtual leg" used in the
  dynamics, so the two can be cross-checked against each other.
- Design a **gain-scheduled LQR balance controller** (gains fit as a function of leg
  length) as the baseline controller, validated against both the linearized model and
  the full nonlinear equations of motion (via `ode45`).
- Provide a **live 2D animation** of the actual 5-bar linkage (not a simplified
  pendulum) driven by the closed-loop simulation, to sanity-check behavior visually.
- Every numerical claim in the model (kinematics round-trip, Jacobian consistency,
  controllability, closed-loop stability, actuator torque budgets, animation geometry)
  is checked in `wb.selfcheck()` rather than just asserted.

This is explicitly **Stage 0** of a longer-term plan: a rigid-leg / position-controlled-hip
simplification good enough to validate the balance architecture and get a real
prototype standing and rolling. Torque-controlled hips, Virtual Model Control (VMC),
a full 3D model, and more advanced controllers (MPC) are intended as later stages, not
covered by this document.

## 2. Robot details this model targets

**Configuration:** two-wheeled, wheel-leg ("wheeled biped") robot, three actuators per
side — one hub-drive wheel motor, two hip motors driving a parallel 5-bar linkage per
leg. Sagittal-plane balancing, similar in spirit to designs from the "RoboMaster
balancing infantry" community and academic wheel-legged robot literature (Section 4).

**Actuators modeled:**

| Component | Part | Key spec used in the model |
|---|---|---|
| Wheel drive | Steadywin GIM6010-6 | 3.3 N·m continuous, 8 N·m stall (per motor) |
| Hip actuator (×2 per leg) | LKMTECH MG8016E-i6 | ~12 N·m continuous rating used for sizing checks |

**5-bar leg geometry** (symmetric parallel 5-bar; placeholder link lengths in
`params()`, meant to be replaced with the builder's actual CAD values):
rear thigh, rear shin, front shin, front thigh, and hip-pivot spacing, with the wheel
axle at the coupler point where the two shins meet.

**CAN bus notes carried into the architecture discussion** (not part of the dynamics
model itself, but informed the recommendation to move toward torque-mode hips):
- The MyActuator/RMD-family protocol (the closest documented reference available for
  this class of LK/RMD-style motor; LKMTECH's own firmware should be verified against
  actual hardware before relying on protocol details) is a cascaded position-PI →
  speed-PI → current-PI structure with 8-bit gain resolution — not a true
  spring/damper, and not adjustable to a specific N·m/rad stiffness.
- A "Motion Mode Control" command (MIT-style: position + velocity + kp + kd +
  feedforward torque in one CAN frame) exists in the documented protocol and, if
  supported by the actual LKMTECH firmware, would give real joint impedance control
  in one frame instead of the position-mode cascade above.
- The documented multi-motor CAN frame supports at most 4 motors per bus and carries
  torque commands only, which favors torque-mode control for bus-bandwidth reasons
  once all four hip motors are in use.

## 3. Prior art informing the architecture

The general recipe used here — model the leg as an equivalent variable-length
inverted pendulum, balance with LQR, and (for a full torque-controlled version) map
the virtual-leg wrench to joint torques with Virtual Model Control (VMC) — is the
standard approach in both the academic wheel-legged-robot literature and the hobbyist
community building similar robots:

- **Ascento** (ETH Zürich) — the canonical reference for a compact 5-bar/4-bar-legged
  two-wheeled balancing robot using LQR as the whole-body controller, plus a
  follow-up paper adding hierarchical whole-body control for uneven terrain.
- A cluster of recent papers on **wheel-legged robots combining LQR with Virtual
  Model Control (VMC)** for the parallel-linkage leg, several explicitly analyzing
  five-bar or four-bar leg geometry — this is the specific pattern this model
  follows (virtual leg abstraction + LQR + VMC-style torque mapping for the
  eventual torque-controlled version).
- The RoboMaster hobbyist/competition community's informal "SJTU model" — treat the
  5-bar as an equivalent telescoping pendulum, gain-schedule the LQR on leg length,
  use VMC to convert the pendulum's virtual wrench into joint torques. This document's
  gain-scheduling-on-leg-length approach follows that pattern directly.

## 4. Sources

**Academic / reference control architecture:**
- Klemm et al., *"Ascento: A Two-Wheeled Jumping Robot,"* ICRA 2019.
  https://arxiv.org/pdf/2005.11435
- Klemm et al., *"LQR-Assisted Whole-Body Control of a Wheeled Bipedal Robot with
  Kinematic Loops,"* IEEE RA-L 2020. https://arxiv.org/abs/2005.11431
- *"Design and dynamic analysis of jumping wheel-legged robot in complex terrain
  environment"* (parallel four-bar wheel-leg, LQR + fuzzy PD jump control).
  https://pmc.ncbi.nlm.nih.gov/articles/PMC9755738/
- *"Design and Control of a Wheeled Bipedal Robot Based on Hybrid Linear Quadratic
  Regulator and Proportional-Derivative Control"* (four-bar linkage, LQR + PD + VMC
  for leg-height compensation), MDPI Sensors 2025.
  https://www.mdpi.com/1424-8220/25/17/5398
- *"Modelling and Control of a Two-Wheeled Foot-Balanced Robot Based on Simplified
  VMC and LQR,"* Springer. https://link.springer.com/chapter/10.1007/978-981-96-3592-4_54
- *"Research on wheel-legged robot based on LQR and ADRC,"* Scientific Reports 2023
  (surveys several five-bar / seven-link wheel-leg control approaches in its related
  work). https://www.nature.com/articles/s41598-023-41462-1

**Actuator hardware:**
- Steadywin GIM6010-6 product page (rated/stall torque figures used in `params()`):
  https://aifitlab.com/products/steadywin-gim6010-6-planetary-reducer-servo-motor
- LKMTECH MG8016E-i6-V2 product page:
  https://aifitlab.com/products/lkmtech-mg8016e-i6-v2-motor
- MyActuator RMD-X *Servo Motor Control Protocol V4.01* (used as the closest available
  documented reference for the LK/RMD-family CAN protocol structure — cascaded PID
  loops, multi-motor frame, Motion Mode command; **verify against LKMTECH's own
  documentation/firmware before relying on specifics**):
  https://cdn.robotshop.com/rbm/acdeb3e1-04dc-4138-8b99-88509ddc4e94/4/4501bc19-2742-44e8-8d9e-8a4b398c18c4/e6052050_rmd-x-motor-motion-protocol-v4.01.pdf

## 5. Known simplifications (read before trusting numbers)

- **The dynamics model lumps the entire 5-bar leg into one rigid rod** (constant
  length `L`, single mass `m_l`, single inertia `I_l`, single COM offset `l_c`) rather
  than modeling the two individual links with their own Jacobian. This is standard for
  a first LQR pass and the error is second-order (leg mass << body mass), but it means
  the exact magnitude of leg-swing-to-wheel-roll coupling is approximate, not exact.
- **Leg length `L` is a scheduled parameter, not a feedback state** — the balance LQR
  assumes `L` changes slowly compared to the balance dynamics (a separate, stiffer
  height-control loop). This matches the intended architecture but is an assumption,
  not something the model enforces.
- **Position-mode hips are treated as ideal within this stage** — the model does not
  yet capture the actual position-PI/speed-PI cascade dynamics or the coarse feedback
  resolution discussed in Section 2; the intended fix is to move to torque control
  once available (Motion Mode / VMC).
- **No wheel slip, no terrain, no lateral/roll dynamics** — purely sagittal-plane,
  flat ground, rolling without slipping.
- Default `params()` values (masses, inertias, link lengths) are placeholders and
  must be replaced with real CAD/measured numbers before trusting any gain this
  model produces.
