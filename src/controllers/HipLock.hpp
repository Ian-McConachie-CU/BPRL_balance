#pragma once
#include "src/controllers/PID.hpp"

/*
 * HipLock — holds all four hip motors at fixed joint angles via independent
 * position PIDs, one per motor. Used by both balance controllers while the
 * robot is in its Stage 0 "position-controlled hips" configuration (see
 * wheeled_biped_project_notes.md section 1) -- torque-controlled hips / VMC
 * are a later stage, not implemented here.
 *
 * Why independent per-motor PIDs are enough (no whole-body state needed):
 * each hip encoder (phi1, phi4 in wheeled_biped.m's 5-bar notation) reads a
 * joint angle relative to the BODY, not to world vertical. Holding phi1 and
 * phi4 each at a fixed target therefore holds the leg's shape relative to
 * the body fixed too (both leg length L and the "legSplay" angle between
 * leg and body, wheeled_biped.m's theta = phi - legSplay), with no
 * dependence on the body's own pitch. This was verified against
 * wheeled_biped.m's linearized model: holding the equivalent lumped
 * quantity (phi - theta) via a plain PD, with NO feedback from body pitch
 * or its rate, is exactly what stabilizes the coupled system (see
 * PidBalanceController.cpp's derivation comment) -- the coupling from wheel
 * torque onto the leg (the wheel motor's stator is mounted on the leg, so
 * it reacts directly onto it) is real and significant, which is why the
 * gains below need to be reasonably STIFF, not just "soft and forgiving" --
 * a too-soft hip lock was the failure mode found during that verification.
 */
class HipLock {
public:
    HipLock();

    // hip_torques[4]: FL, FR, RL, RR — matches motor_torques[0..3] and CAN
    // ids 1..4 (see main.cpp's can_motor_register calls).
    void update(float hip_torques[4]);
    void reset();

private:
    // ── Tuning knobs — placeholders, tune on the bench ─────────────────────
    // Target joint angles [rad]. All default to 0 (encoder zero) because the
    // real zero reference depends on how each motor is mechanically mounted
    // and hasn't been calibrated against the physical linkage yet -- treat
    // 0 as "whatever the encoder reads at power-on with the leg in its
    // resting/reference pose" until that calibration is done.
    static constexpr float TARGET_FL_RAD = 0.0f;
    static constexpr float TARGET_FR_RAD = 0.0f;
    static constexpr float TARGET_RL_RAD = 0.0f;
    static constexpr float TARGET_RR_RAD = 0.0f;

    // PID gains — shared across all four joints for now. Needs to be
    // reasonably STIFF (see class comment above); start here and increase
    // kp/kd together if the leg visibly droops or oscillates under wheel
    // torque disturbances.
    static constexpr float KP   = 8.0f;    // Nm / rad
    static constexpr float KI   = 0.0f;    // Nm / (rad*s)
    static constexpr float KD   = 0.3f;    // Nm / (rad/s)
    static constexpr float IMAX = 2.0f;    // Nm, integrator clamp

    // Output clamp — keep comfortably under LKMTECH MG8016E-i6's continuous
    // rating (12 Nm, see wheeled_biped.m's p.tau_hip_cont) until bench-tested.
    static constexpr float TORQUE_LIMIT_NM = 8.0f;

    PID _pid[4];  // FL, FR, RL, RR
};
