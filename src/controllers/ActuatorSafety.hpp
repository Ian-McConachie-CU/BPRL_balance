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
 *   - Hip motors (ids 1-4) ONLY: soft angle limits. As a joint approaches a
 *     hard stop, torque that would drive it FURTHER toward that stop is
 *     progressively reduced to exactly zero over a margin before the stop
 *     is reached — never a hard on/off snap. Torque moving back toward
 *     center is never restricted by this. Wheels have no angle limit
 *     (continuous rotation).
 *   - All six motors: the same progressive-reduction shape applied to
 *     velocity, protecting against commanding an actuator to accelerate
 *     further past its rated speed.
 *   - All six motors: a hard torque clamp, independent of the above —
 *     the actuator's absolute rating, not a control-tuning limit (compare
 *     to e.g. PidBalanceController::WHEEL_TORQUE_LIMIT_NM, which is a
 *     smaller, separate "how hard should THIS loop be allowed to push"
 *     knob; this one is "never exceed the motor's rating," full stop).
 *   - Fail-safe: a motor with no valid CAN feedback yet gets zero torque —
 *     its state can't be verified, so nothing is commanded.
 *
 * ALL limits below are placeholders. The hip angle hard stops in
 * particular are just a wide, safe-sounding guess — measure the real
 * linkage's mechanical limits and set HIP_ANGLE_MIN/MAX_RAD accordingly
 * before trusting this on hardware, same as every other placeholder
 * constant in this codebase (see wheeled_biped.m's params() for the
 * established convention).
 */
class ActuatorSafety {
public:
    // motor_torques[6]: [0..3] hip FL/FR/RL/RR, [4..5] wheel L/R — same
    // ordering as everywhere else (main.cpp's can_motor_register calls).
    // Modifies in place.
    void apply(float motor_torques[6]) const;

private:
    // ── Hip angle hard stops + soft margin — CALIBRATE against real hardware ──
    static constexpr float HIP_ANGLE_MIN_RAD[4] = { -1.57f, -1.57f, -1.57f, -1.57f };
    static constexpr float HIP_ANGLE_MAX_RAD[4] = {  1.57f,  1.57f,  1.57f,  1.57f };
    static constexpr float HIP_ANGLE_SOFT_MARGIN_RAD = 0.175f;  // ~10 deg before the stop

    // ── Velocity soft limits ────────────────────────────────────────────────
    static constexpr float HIP_VEL_LIMIT_RADS         = 20.0f;  // ~190 rpm
    static constexpr float HIP_VEL_SOFT_MARGIN_RADS   = 4.0f;
    static constexpr float WHEEL_VEL_LIMIT_RADS       = 60.0f;  // ~570 rpm
    static constexpr float WHEEL_VEL_SOFT_MARGIN_RADS = 10.0f;

    // ── Hard torque clamps — actuator ratings, see wheeled_biped.m's params() ──
    static constexpr float HIP_TORQUE_LIMIT_NM   = 12.0f;  // MG8016E-i6 continuous
    static constexpr float WHEEL_TORQUE_LIMIT_NM = 8.0f;   // GIM6010-6 stall

    // Scale in [0,1] to apply to `torque` given the actuator is currently at
    // `value` with hard bounds [lo, hi] and a soft margin before each bound.
    // 1.0 = unrestricted; ramps to 0.0 exactly at the bound in whichever
    // direction `torque` points; the opposite direction is never restricted.
    static float limit_scale(float value, float lo, float hi, float margin, float torque);
};
