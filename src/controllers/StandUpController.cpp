#include "src/controllers/StandUpController.hpp"
#include "src/controllers/WheelBalanceLQR.hpp"
#include "src/coms/CANMotor.hpp"
#include "src/state_estimator/StateManager.hpp"   // STATEMGR_WHEEL_RADIUS_M
#include "src/math/math.hpp"
#include <cmath>

StandUpController::StandUpController()
    : _L_cmd(L_STAND_M), _L_cmd_init(false),
      _wheel_vel_L(0.0f), _wheel_vel_R(0.0f), _hip_lock()
{}

void StandUpController::reset(const float state[StateIdx::N])
{
    _hip_lock.reset();
    // Seed the ramp from the CURRENT measured leg length so standing up
    // from a collapsed pose actually ramps (not jumps) toward L_STAND_M --
    // see collapse_recovery_demo.m's Lref(). Falls back to L_STAND_M
    // itself if the leg state isn't valid/populated yet (state[LEG_L]==0
    // would otherwise ramp FROM zero, a much larger and physically
    // meaningless jump).
    const float measured_L = state[StateIdx::LEG_L];
    _L_cmd = (measured_L > 0.01f) ? measured_L : L_STAND_M;
    _L_cmd_init = true;
    _wheel_vel_L = 0.0f;
    _wheel_vel_R = 0.0f;
}

void StandUpController::update(const float state[StateIdx::N], float motor_torques[6])
{
    if (!_L_cmd_init) reset(state);

    Quat q = { state[StateIdx::Q0], state[StateIdx::Q1], state[StateIdx::Q2], state[StateIdx::Q3] };
    float roll, phi, yaw;
    quat_to_euler(q, roll, phi, yaw);
    (void)roll; (void)yaw;
    const float phidot    = state[StateIdx::Q];
    const float theta     = state[StateIdx::LEG_PITCH];
    const float thetadot  = state[StateIdx::LEG_PITCH_DOT];
    const float vel_meas  = state[StateIdx::U];

    const WheelBalanceOutput bal = wheel_balance_lqr(state[StateIdx::LEG_L], theta, thetadot, phi, phidot);

    // Zero-velocity REGULATION, not tracking -- no VEL_TGT read here, see
    // class header.
    const float ax_vel = -VEL_DAMP_KV * vel_meas;
    const float ax = constrain_float(bal.ax + ax_vel, -WHEEL_ACCEL_LIMIT, WHEEL_ACCEL_LIMIT);
    const float Tp = constrain_float(bal.Tp, -HIP_TORQUE_LIMIT_NM, HIP_TORQUE_LIMIT_NM);

    // Leg-length ramp: single-pole discrete exponential approach toward
    // L_STAND_M, time constant TAU_L_S -- the firmware-native equivalent
    // of collapse_recovery_demo.m's Lref(t) = L_target + (L_start-L_target)
    // *exp(-t/tau), expressed as a per-tick recurrence instead of a
    // function of elapsed time (no "time since state entry" bookkeeping
    // needed this way).
    _L_cmd += (DT_S / TAU_L_S) * (L_STAND_M - _L_cmd);

    float hip_torques[4];
    _hip_lock.update(hip_torques, _L_cmd);   // thL_target defaults to 0 (straight)
    // Tp split evenly across all 4 hips, added on top of the height hold --
    // same convention the old LqrBalanceController used for its Tp row.
    motor_torques[0] = hip_torques[0] + 0.25f * Tp;
    motor_torques[1] = hip_torques[1] + 0.25f * Tp;
    motor_torques[2] = hip_torques[2] + 0.25f * Tp;
    motor_torques[3] = hip_torques[3] + 0.25f * Tp;

    // ax -> wheel velocity setpoint: v_cmd = v_meas + ax*dt, each wheel
    // using ITS OWN measured velocity (not a shared body estimate) so the
    // two wheels can't fight each other over an estimate that's locally
    // wrong for one side -- see WheelBalanceLQR.hpp's header. No verified
    // feedback yet -> command zero, same fail-safe philosophy as
    // CANMotor.cpp's hip gate.
    CanMotorState wl = {}, wr = {};
    const bool wl_ok = can_motor_get_state(5, &wl) && wl.valid;
    const bool wr_ok = can_motor_get_state(6, &wr) && wr.valid;
    const float dv_rads = ax * DT_S / STATEMGR_WHEEL_RADIUS_M;
    _wheel_vel_L = wl_ok ? constrain_float(wl.vel_rads + dv_rads, -WHEEL_VEL_LIMIT_RADS, WHEEL_VEL_LIMIT_RADS) : 0.0f;
    _wheel_vel_R = wr_ok ? constrain_float(wr.vel_rads + dv_rads, -WHEEL_VEL_LIMIT_RADS, WHEEL_VEL_LIMIT_RADS) : 0.0f;

    motor_torques[4] = 0.0f;   // wheels driven via velocity, see wheel_vel_L()/_R()
    motor_torques[5] = 0.0f;
}

bool StandUpController::is_standing(const float state[StateIdx::N]) const
{
    Quat q = { state[StateIdx::Q0], state[StateIdx::Q1], state[StateIdx::Q2], state[StateIdx::Q3] };
    float roll, phi, yaw;
    quat_to_euler(q, roll, phi, yaw);
    (void)roll; (void)yaw;
    const float vel = state[StateIdx::U];
    return _L_cmd_init
        && fabsf(phi) < STAND_PHI_MAX_RAD
        && fabsf(vel) < STAND_VEL_MAX_MPS
        && fabsf(state[StateIdx::LEG_L] - L_STAND_M) < STAND_L_TOL_M;
}
