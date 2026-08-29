#pragma once
#include "src/RobotState.hpp"
#include "src/controllers/HipLock.hpp"

/*
 * LqrBalanceController — Stage 1 balance controller: gain-scheduled LQR
 * state feedback, gains computed offline by MatLab_controls/wheeled_biped.m
 * (wb.schedule / wb.evalGains) and pasted in here once tuned.
 *
 * STUBBED: K is all zeros below (produces zero torque) until real gains
 * from the MATLAB sim are pasted in — see controls_plan.md section 5 for
 * the intended offline-gain / onboard-polynomial-evaluation split. This
 * class currently uses a single fixed K (one leg length); gain scheduling
 * (evalGains-equivalent polynomial evaluation vs. leg length) is not yet
 * wired in, since there's no leg-length feedback state yet either — same
 * TODO as the leg-angle one below.
 *
 * State vector ordering matches wheeled_biped.m exactly:
 *   x_lqr = [x, xdot, theta, thetadot, phi, phidot],  u = [T; Tp]
 * (see wheeled_biped.m's CONVENTIONS block / controls_plan.md — this
 * firmware's pitch/pitch-rate ARE phi/phidot directly, no sign conversion).
 *
 * TODO (controls_plan.md sections 2-4): theta/thetadot (leg pitch and its
 * rate) aren't real states yet — StateIdx has no leg-angle entry until the
 * FiveBarIK + StateManager leg-state wiring described there exists. Until
 * then theta/thetadot read as 0, which is only approximately right while
 * the hips are actually being held near that angle by HipLock below.
 */
class LqrBalanceController {
public:
    LqrBalanceController();

    void update(const float state[StateIdx::N],
                const float input[InputIdx::N_INPUTS],
                float       motor_torques[6]);

    void reset();

private:
    // ── Tuning knobs ────────────────────────────────────────────────────
    // Gain matrix K, rows = [T, Tp], cols = [x, xdot, theta, thetadot, phi,
    // phidot] — paste wb.evalGains(sched, L) here for your chosen L once
    // wheeled_biped.m is tuned against real hardware params(). ALL ZERO is
    // an intentional placeholder (zero torque from this term), not a guess.
    static constexpr float K[2][6] = {
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },   // T  row
        { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },   // Tp row
    };

    // input[InputIdx::VEL_TGT] is [-1,1]; scaled to m/s by this. Position
    // (x) is deliberately dropped from the feedback and replaced by the
    // velocity error, same trick wheeled_biped.m's simulate() uses to track
    // a velocity command with a regulator-style LQR — there is no absolute
    // position setpoint.
    static constexpr float MAX_VELOCITY_MPS = 1.0f;

    static constexpr float WHEEL_TORQUE_LIMIT_NM = 4.0f;
    static constexpr float HIP_TORQUE_LIMIT_NM   = 4.0f;

    HipLock _hip_lock;
};
