#pragma once
#include "src/RobotState.hpp"
#include "src/controllers/PID.hpp"
#include "src/controllers/HipLock.hpp"

/*
 * PidBalanceController — Stage 0 balance controller: a single-loop-cascade
 * (SLC) PID pair, hips held at a fixed angle by HipLock (no leg motion).
 *
 *   outer loop: forward-velocity error  -> commanded body-pitch setpoint
 *   inner loop: body-pitch error        -> wheel torque
 *
 * See PidBalanceController.cpp for the sign-convention derivation -- the
 * outer loop's sign is NOT the naive "lean forward to go forward" story
 * you'd expect, because this codebase's pitch is NED-native (positive =
 * nose-up = body tilts BACKWARD, see wheeled_biped.m's CONVENTIONS block
 * and controls_plan.md). Both loop signs were verified against
 * wheeled_biped.m's linearized model (closed-loop eigenvalues, swept across
 * hip-lock stiffness and leg length) and its full nonlinear simulation
 * before being written here -- don't "fix" an apparently-backwards sign
 * without re-deriving it the same way.
 */
class PidBalanceController {
public:
    PidBalanceController();

    // Called at 400 Hz from BalanceController when the PID cascade is selected.
    void update(const float state[StateIdx::N],
                const float input[InputIdx::N_INPUTS],
                float       motor_torques[6]);

    void reset();

private:
    // ── Tuning knobs — placeholders, tune on the bench ─────────────────────
    // input[InputIdx::VEL_TGT] is [-1,1]; scaled to m/s by this.
    static constexpr float MAX_VELOCITY_MPS = 1.0f;
    // Outer-loop output clamp -- how far the pitch setpoint is allowed to
    // swing away from upright to chase a velocity error.
    static constexpr float MAX_LEAN_RAD = 0.35f;   // ~20 deg
    // Inner-loop output clamp, PER WHEEL (both wheels get the same torque
    // command, no differential/yaw mixing in this controller).
    static constexpr float WHEEL_TORQUE_LIMIT_NM = 4.0f;

    // Outer loop: velocity error -> pitch setpoint. Gains are POSITIVE,
    // ordinary "bigger = snappier" tuning knobs -- the sign inversion that
    // makes this correct is applied once, internally (see .cpp).
    static constexpr float VEL_KP   = 0.15f;
    static constexpr float VEL_KI   = 0.0f;
    static constexpr float VEL_KD   = 0.0f;
    static constexpr float VEL_IMAX = MAX_LEAN_RAD;

    // Inner loop: pitch error -> wheel torque.
    static constexpr float PITCH_KP   = 15.0f;
    static constexpr float PITCH_KI   = 0.0f;
    static constexpr float PITCH_KD   = 0.5f;
    static constexpr float PITCH_IMAX = WHEEL_TORQUE_LIMIT_NM;

    PID     _vel_pid;
    PID     _pitch_pid;
    HipLock _hip_lock;
};
