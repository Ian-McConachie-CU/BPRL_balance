#pragma once

/*
 * WheelBalanceLQR — gain-SCHEDULED 4-state discrete LQR balance law
 * against the velocity-controlled-wheel reduced model, gains computed
 * offline by MatLab_controls/export_wheel_balance_gains.m (wb.schedule /
 * wb.evalGains under the hood, model='velwheel') and exported into
 * src/controllers/WheelBalanceGainTable.hpp — see that generated header
 * for the exact Q/R/Lgrid/dt the design used. K is evaluated fresh every
 * call against the current averaged leg length, matching
 * wb.evalGains(sched, L) exactly (same cubic-in-normalized-L fit) —
 * mirrors the OLD (torque-wheel) LqrBalanceController's eval_gains()
 * pattern, just against the new reduced model instead.
 *
 * State vector ordering matches wheeled_biped.m's linearModelVel() exactly:
 *   x = [theta, thetadot, phi, phidot],  u = [ax, Tp]
 * theta/thetadot come from StateIdx::LEG_PITCH/LEG_PITCH_DOT (both-legs-
 * averaged FiveBarIK output — see StateManager.cpp); phi/phidot from the
 * quaternion + StateIdx::Q, same as the old LqrBalanceController.
 *
 * ax is a commanded WHEEL ACCELERATION [m/s^2], NOT a torque — the wheel
 * motors (GIM6010-8, ODrive) run in velocity mode (see CANMotor.cpp /
 * controls_plan.md), so this firmware never computes a wheel torque
 * directly for balancing. Callers (StandUpController, LqrBalanceController)
 * integrate ax into a velocity setpoint themselves (v_cmd = v_meas +
 * ax*dt), each wheel using ITS OWN measured velocity — this function only
 * produces the balance law's contribution to that acceleration, not the
 * final per-wheel command.
 *
 * Tp is a lumped hip torque, split across the 4 hip motors via a leg-
 * height hold (see HipLock's dynamic-target overload).
 */
struct WheelBalanceOutput {
    float ax;   // m/s^2, balance-law contribution only — see header comment
    float Tp;   // N.m, lumped hip torque
};

// L: current averaged leg length [m] (state[StateIdx::LEG_L]) — the gain
// table is evaluated (and clamped to its fitted range) at this value each
// call, see WheelBalanceGainTable.hpp.
WheelBalanceOutput wheel_balance_lqr(float L, float theta, float thetadot, float phi, float phidot);
