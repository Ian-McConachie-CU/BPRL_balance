#pragma once
#include "src/RobotState.hpp"
#include "src/controllers/HipLock.hpp"

/*
 * StandUpController — ROBOT_STANDING_UP: the transitional state entered
 * when arming into (or mode-switching into) balance mode from IDLE/CAR
 * (see RobotStateMachine, README.md's planned mode state machine).
 * Balances to upright (theta/phi -> 0) and zero velocity (no VEL_TGT
 * input read at all — see "why no stick input" below) while slowly
 * extending the leg from wherever it currently is toward L_STAND_M (the
 * "-10 deg on both hips" height setpoint — see
 * MatLab_controls/collapse_recovery_demo.m; this class is that MATLAB
 * scenario's stand-up phase, ported). RobotStateMachine transitions to
 * ROBOT_BALANCING once is_standing() reports true.
 *
 * Deliberately reads NEITHER VEL_TGT NOR HEIGHT_SET — per the requirement
 * that those commands "not be present when the robot is not in the
 * balancing state": velocity regulates to zero (a fixed reference, not a
 * moving target) and leg length ramps to a fixed constant (not the
 * stick), unconditionally, regardless of where either stick physically
 * sits. LqrBalanceController (ROBOT_BALANCING) is where both become live.
 *
 * Wheels run in ODrive VELOCITY mode (RobotStateMachine switches this on
 * STANDING_UP entry, same as it already does for ROBOT_CAR — see
 * RobotStateMachine.cpp's wants_wheel_velocity()) — this class outputs a
 * velocity target via wheel_vel_L()/_R(), NOT motor_torques[4]/[5] (left
 * zero here, unused — same convention ROBOT_CAR already established, see
 * CarController.hpp). Hips stay torque-controlled, via the ordinary
 * motor_torques[0..3] path (a leg-height hold, see HipLock's dynamic-
 * target overload — NOT the fixed-angle hold PidBalanceController uses).
 */
class StandUpController {
public:
    StandUpController();

    // Writes hip torques into motor_torques[0..3] (caller should have
    // already zeroed [4]/[5] — RobotStateMachine's memset-then-dispatch
    // pattern already does this). Updates the internal leg-length ramp and
    // wheel velocity targets, read via wheel_vel_L()/_R() below.
    void update(const float state[StateIdx::N], float motor_torques[6]);

    // True once upright, near-zero velocity, and the leg has reached
    // L_STAND_M closely enough — RobotStateMachine's cue to switch to
    // ROBOT_BALANCING.
    bool is_standing(const float state[StateIdx::N]) const;

    // Seeds the leg-length ramp from the CURRENT measured leg length (not
    // always from zero) and clears the hip-lock PIDs/wheel targets — call
    // once on ROBOT_STANDING_UP entry (RobotStateMachine).
    void reset(const float state[StateIdx::N]);

    float wheel_vel_L() const { return _wheel_vel_L; }
    float wheel_vel_R() const { return _wheel_vel_R; }

private:
    // ── Tuning knobs — placeholders, tune on the bench ─────────────────
    static constexpr float L_STAND_M    = 0.152f;  // "-10deg on both hips",
                                                      // see collapse_recovery_demo.m
    static constexpr float TAU_L_S      = 1.0f;    // leg-extension time
                                                      // constant [s], matches
                                                      // the MATLAB demo
    static constexpr float DT_S         = 0.01f;   // 100 Hz, matches
                                                      // ControlThread's compute
                                                      // rate (main.cpp kRates.control)

    static constexpr float VEL_DAMP_KV        = 1.5f;  // ax per m/s of velocity
                                                          // error, regulates
                                                          // toward ZERO (no
                                                          // target input)
    static constexpr float WHEEL_ACCEL_LIMIT  = 4.0f;   // m/s^2
    static constexpr float WHEEL_VEL_LIMIT_RADS = 55.0f; // final clamp on the
                                                          // resulting velocity
                                                          // command -- ActuatorSafety
                                                          // doesn't gate ODrive
                                                          // velocity-mode commands
                                                          // (see CarController.hpp's
                                                          // identical caveat), so
                                                          // this is the only limit
                                                          // protecting the wheels
                                                          // here; matches
                                                          // CarController's own value.
    static constexpr float HIP_TORQUE_LIMIT_NM = 4.0f;  // N.m, lumped Tp clamp
                                                          // (leg-height hold's own
                                                          // TORQUE_LIMIT_NM in
                                                          // HipLock is separate)

    // Upright/still/extended thresholds for is_standing() -- placeholders.
    static constexpr float STAND_PHI_MAX_RAD = 0.0872665f;  // ~5 deg
    static constexpr float STAND_VEL_MAX_MPS = 0.05f;
    static constexpr float STAND_L_TOL_M     = 0.005f;      // 5 mm

    float _L_cmd;
    bool  _L_cmd_init;
    float _wheel_vel_L;
    float _wheel_vel_R;
    HipLock _hip_lock;
};
