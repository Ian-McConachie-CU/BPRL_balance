#pragma once
#include "src/RobotState.hpp"
#include "src/controllers/HipLock.hpp"

/*
 * LqrBalanceController — Stage 1 balance controller (ROBOT_BALANCING),
 * gain-scheduled LQR against the VELOCITY-CONTROLLED-WHEEL reduced model
 * (see MatLab_controls/wheeled_biped.m's Part 3 / linearModelVel, and
 * WheelBalanceLQR.hpp for the gain table this evaluates) — a full rewrite
 * of an earlier version of this class that used the old torque-wheel
 * 6-state model; that model doesn't match real hardware, which runs the
 * wheel motors (GIM6010-8) in ODrive VELOCITY mode, not torque (see
 * CANMotor.cpp).
 *
 * Balance law: x = [theta, thetadot, phi, phidot], u = [ax, Tp] — see
 * WheelBalanceLQR.hpp. ax is a commanded wheel ACCELERATION, integrated
 * here into a velocity setpoint (v_cmd = v_meas + ax*dt) per wheel, not
 * sent as a torque. Two things are layered on top of the bare balance
 * law, both ONLY active here (ROBOT_BALANCING) — StandUpController
 * (ROBOT_STANDING_UP) reads neither stick, per the requirement that these
 * commands not apply outside the balancing state:
 *   - input[InputIdx::VEL_TGT]: forward-velocity TARGET (StandUpController
 *     only ever regulates toward zero).
 *   - input[InputIdx::HEIGHT_SET]: leg-height stick, offsetting the base
 *     standing leg length L_STAND_M (StandUpController only ever ramps
 *     toward that fixed constant, ignoring the stick entirely).
 *
 * Hips stay torque-controlled (leg-height hold via HipLock's dynamic-
 * target overload, same mechanism StandUpController uses); wheels run via
 * wheel_vel_L()/_R() (ODrive velocity mode — RobotStateMachine keeps this
 * mode engaged across the STANDING_UP -> BALANCING transition, see its
 * own header), motor_torques[4]/[5] unused.
 */
class LqrBalanceController {
public:
    LqrBalanceController();

    void update(const float state[StateIdx::N],
                const float input[InputIdx::N_INPUTS],
                float       motor_torques[6]);

    // Seeds the leg-height ramp from the CURRENT measured leg length --
    // call once on the ROBOT_STANDING_UP -> ROBOT_BALANCING transition
    // (RobotStateMachine) so the height stick's offset applies on top of
    // wherever standing up actually left the leg, not a discontinuous
    // jump. Mirrors StandUpController::reset()'s identical pattern.
    void reset(const float state[StateIdx::N]);

    float wheel_vel_L() const { return _wheel_vel_L; }
    float wheel_vel_R() const { return _wheel_vel_R; }

private:
    // ── Tuning knobs — placeholders, tune on the bench ─────────────────
    static constexpr float L_STAND_M      = 0.152f;  // matches StandUpController
    static constexpr float HEIGHT_RANGE_M = 0.05f;    // +/- at full stick deflection
    static constexpr float L_MIN_M        = 0.10f;    // hard clamp, independent of
    static constexpr float L_MAX_M        = 0.20f;    // stick scaling -- matches
                                                         // WheelBalanceGainTable's
                                                         // fitted Lgrid=[0.09,0.22]
                                                         // exactly, both well inside
                                                         // the confirmed-reachable
                                                         // [0.08, 0.34] workspace
    static constexpr float TAU_L_S        = 0.5f;     // leg-height stick response
                                                         // time constant [s] -- faster
                                                         // than StandUpController's
                                                         // 1.0s since this tracks live
                                                         // manual input, not an
                                                         // unattended stand-up ramp
    static constexpr float DT_S           = 0.01f;    // 100 Hz, matches ControlThread

    static constexpr float MAX_VELOCITY_MPS    = 1.0f;
    static constexpr float VEL_KP               = 1.5f;  // ax per m/s of velocity error
    static constexpr float WHEEL_ACCEL_LIMIT    = 4.0f;  // m/s^2
    static constexpr float WHEEL_VEL_LIMIT_RADS = 55.0f; // final velocity-command
                                                            // clamp -- see
                                                            // StandUpController.hpp's
                                                            // identical constant/comment
    static constexpr float HIP_TORQUE_LIMIT_NM  = 4.0f;  // N.m, lumped Tp clamp

    float _L_cmd;
    bool  _L_cmd_init;
    float _wheel_vel_L;
    float _wheel_vel_R;
    HipLock _hip_lock;
};
