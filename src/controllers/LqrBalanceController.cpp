#include "src/controllers/LqrBalanceController.hpp"
#include "src/controllers/WheelBalanceLQR.hpp"
#include "src/coms/CANMotor.hpp"
#include "src/state_estimator/StateManager.hpp"   // STATEMGR_WHEEL_RADIUS_M
#include "src/math/math.hpp"

LqrBalanceController::LqrBalanceController()
    : _L_cmd(L_STAND_M), _L_cmd_init(false),
      _wheel_vel_L(0.0f), _wheel_vel_R(0.0f), _hip_lock()
{}

void LqrBalanceController::reset(const float state[StateIdx::N])
{
    _hip_lock.reset();
    const float measured_L = state[StateIdx::LEG_L];
    _L_cmd = (measured_L > 0.01f) ? measured_L : L_STAND_M;
    _L_cmd_init = true;
    _wheel_vel_L = 0.0f;
    _wheel_vel_R = 0.0f;
}

void LqrBalanceController::update(const float state[StateIdx::N],
                                   const float input[InputIdx::N_INPUTS],
                                   float       motor_torques[6])
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
    const float vel_tgt   = input[InputIdx::VEL_TGT] * MAX_VELOCITY_MPS;

    const WheelBalanceOutput bal = wheel_balance_lqr(state[StateIdx::LEG_L], theta, thetadot, phi, phidot);

    const float ax_vel = VEL_KP * (vel_tgt - vel_meas);
    const float ax = constrain_float(bal.ax + ax_vel, -WHEEL_ACCEL_LIMIT, WHEEL_ACCEL_LIMIT);
    const float Tp = constrain_float(bal.Tp, -HIP_TORQUE_LIMIT_NM, HIP_TORQUE_LIMIT_NM);

    // input[InputIdx::HEIGHT_SET] in [-1,1]; positive (stick up) commands a
    // TALLER stance. Deliberately computed directly in leg-length space (L
    // is a positive scalar, not NED-signed) rather than via any Z/height
    // offset -- side-steps the "-height because NED down is positive" sign
    // trap entirely: there's no NED-Z quantity anywhere in this mapping to
    // get backwards.
    const float L_target_raw = constrain_float(
        L_STAND_M + input[InputIdx::HEIGHT_SET] * HEIGHT_RANGE_M, L_MIN_M, L_MAX_M);
    _L_cmd += (DT_S / TAU_L_S) * (L_target_raw - _L_cmd);

    float hip_torques[4];
    _hip_lock.update(hip_torques, _L_cmd);
    motor_torques[0] = hip_torques[0] + 0.25f * Tp;
    motor_torques[1] = hip_torques[1] + 0.25f * Tp;
    motor_torques[2] = hip_torques[2] + 0.25f * Tp;
    motor_torques[3] = hip_torques[3] + 0.25f * Tp;

    // ax -> wheel velocity setpoint: v_cmd = v_meas + ax*dt, each wheel
    // using ITS OWN measured velocity -- see WheelBalanceLQR.hpp's header.
    // No verified feedback yet -> command zero, same fail-safe philosophy
    // as CANMotor.cpp's hip gate.
    CanMotorState wl = {}, wr = {};
    const bool wl_ok = can_motor_get_state(5, &wl) && wl.valid;
    const bool wr_ok = can_motor_get_state(6, &wr) && wr.valid;
    const float dv_rads = ax * DT_S / STATEMGR_WHEEL_RADIUS_M;
    _wheel_vel_L = wl_ok ? constrain_float(wl.vel_rads + dv_rads, -WHEEL_VEL_LIMIT_RADS, WHEEL_VEL_LIMIT_RADS) : 0.0f;
    _wheel_vel_R = wr_ok ? constrain_float(wr.vel_rads + dv_rads, -WHEEL_VEL_LIMIT_RADS, WHEEL_VEL_LIMIT_RADS) : 0.0f;

    motor_torques[4] = 0.0f;   // wheels driven via velocity, see wheel_vel_L()/_R()
    motor_torques[5] = 0.0f;
}
