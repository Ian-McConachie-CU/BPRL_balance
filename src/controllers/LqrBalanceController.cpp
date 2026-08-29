#include "src/controllers/LqrBalanceController.hpp"
#include "src/math/math.hpp"

// Out-of-class definition required pre-C++17 for a constexpr array member
// that's odr-used (indexed in a loop, as below) rather than just read as a
// compile-time constant.
constexpr float LqrBalanceController::K[2][6];

LqrBalanceController::LqrBalanceController()
    : _hip_lock()
{}

void LqrBalanceController::reset()
{
    _hip_lock.reset();
}

void LqrBalanceController::update(const float state[StateIdx::N],
                                   const float input[InputIdx::N_INPUTS],
                                   float       motor_torques[6])
{
    Quat q = { state[StateIdx::Q0], state[StateIdx::Q1], state[StateIdx::Q2], state[StateIdx::Q3] };
    float roll, phi, yaw;
    quat_to_euler(q, roll, phi, yaw);
    (void)roll; (void)yaw;

    const float vel_meas = state[StateIdx::U];
    const float vel_tgt  = input[InputIdx::VEL_TGT] * MAX_VELOCITY_MPS;

    // x_lqr = [x, xdot, theta, thetadot, phi, phidot] -- x dropped (velocity
    // tracking, not position holding), theta/thetadot are TODO placeholders
    // (see class header) until real leg-angle state exists.
    const float x_lqr[6] = {
        0.0f,
        vel_meas - vel_tgt,
        0.0f,
        0.0f,
        phi,
        state[StateIdx::Q],
    };

    float T = 0.0f, Tp = 0.0f;
    for (int j = 0; j < 6; ++j) {
        T  -= K[0][j] * x_lqr[j];
        Tp -= K[1][j] * x_lqr[j];
    }
    T  = constrain_float(T,  -WHEEL_TORQUE_LIMIT_NM, WHEEL_TORQUE_LIMIT_NM);
    Tp = constrain_float(Tp, -HIP_TORQUE_LIMIT_NM,   HIP_TORQUE_LIMIT_NM);

    // Hips are still position-held (see class header — no VMC/leg motion
    // yet); Tp from the LQR is added on top of the hold as an interim
    // measure so the K matrix's hip-torque row isn't simply discarded once
    // real gains are pasted in, split evenly across all four hip motors.
    float hip_torques[4];
    _hip_lock.update(hip_torques);

    motor_torques[0] = hip_torques[0] + 0.25f * Tp;
    motor_torques[1] = hip_torques[1] + 0.25f * Tp;
    motor_torques[2] = hip_torques[2] + 0.25f * Tp;
    motor_torques[3] = hip_torques[3] + 0.25f * Tp;
    motor_torques[4] = T;
    motor_torques[5] = T;
}
