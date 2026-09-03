#pragma once

/*
 * ActuatorSafety — final gate between whatever produced motor_torques[6]
 * (PidBalanceController, LqrBalanceController, a future controller, or the
 * IDLE zero-torque path) and the CAN commands actually sent. Called
 * unconditionally from ControlThread right before the can_motor_set_torque()
 * loop (see threads.cpp) — no controller-mode code path runs after it, so
 * there's no way to reach the motors without going through this.
 *
 * Enforces, per motor, using live CAN feedback (CanMotorState):
 *   - Wheels (ids 5-6) ONLY: a progressive-reduction velocity soft limit,
 *     protecting against commanding an actuator to accelerate further past
 *     its rated speed (wheels have no angle limit — continuous rotation).
 *   - Wheels: a hard torque clamp, independent of the above — the
 *     actuator's absolute rating, not a control-tuning limit.
 *   - All six motors: fail-safe — a motor with no valid CAN feedback yet
 *     gets zero torque; its state can't be verified, so nothing is
 *     commanded.
 *
 * Hip motors (ids 1-4) are NOT limited here — that responsibility moved to
 * CANMotor.cpp's hip_soft_scale()/hip_clamp_velocity()/
 * hip_clamp_position_target(), which gate every hip torque/velocity/
 * position command at the source (can_motor_set_torque() etc.) regardless
 * of caller, not just calls that happen to go through ControlThread. See
 * CANMotor.hpp's header comment. Duplicating that logic here would only
 * risk the two copies silently diverging.
 *
 * ALL limits below are placeholders — measure the real hardware's ratings
 * and set these accordingly before trusting this, same as every other
 * placeholder constant in this codebase (see wheeled_biped.m's params()
 * for the established convention).
 */
class ActuatorSafety {
public:
    // motor_torques[6]: [0..3] hip FL/FR/RL/RR, [4..5] wheel L/R — same
    // ordering as everywhere else (main.cpp's can_motor_register calls).
    // Modifies in place. Hips pass through unmodified here (see class
    // comment) but still get fail-safe-zeroed if their feedback is invalid.
    void apply(float motor_torques[6]) const;

private:
    // ── Velocity soft limit (wheels only) ──────────────────────────────────
    static constexpr float WHEEL_VEL_LIMIT_RADS       = 60.0f;  // ~570 rpm
    static constexpr float WHEEL_VEL_SOFT_MARGIN_RADS = 10.0f;

    // ── Hard torque clamp (wheels only) — actuator rating, see wheeled_biped.m's params() ──
    static constexpr float WHEEL_TORQUE_LIMIT_NM = 8.0f;   // GIM6010-6 stall

    // Scale in [0,1] to apply to `torque` given the actuator is currently at
    // `value` with hard bounds [lo, hi] and a soft margin before each bound.
    // 1.0 = unrestricted; ramps to 0.0 exactly at the bound in whichever
    // direction `torque` points; the opposite direction is never restricted.
    static float limit_scale(float value, float lo, float hi, float margin, float torque);
};
