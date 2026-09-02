#include "src/controllers/PidBalanceController.hpp"
#include "src/math/math.hpp"

/*
 * SIGN-CONVENTION DERIVATION — read this before touching either loop's sign.
 *
 * This codebase's pitch is NED-native (see wheeled_biped.m's CONVENTIONS
 * block / controls_plan.md): positive pitch = nose-up = the body's top
 * tilting BACKWARD. That inverts the usual "lean forward to accelerate
 * forward" intuition most balancing-robot writeups use (which assume the
 * opposite, tilt-forward-positive convention) -- AND this plant is
 * genuinely non-minimum-phase (Ascento-style wheeled bipeds have to lean
 * backward briefly to accelerate forward even in the textbook convention),
 * so guessing the sign from intuition is unreliable either way.
 *
 * Both loop signs below were instead determined by testing candidate signs
 * against wheeled_biped.m's own verified model:
 *   1. Closed-loop eigenvalues of the linearized 6-state model with this
 *      exact cascade structure (outer: velocity->pitch setpoint, inner:
 *      pitch->wheel torque, plus a hip-lock term holding the leg's angle
 *      relative to the body), swept across hip-lock stiffness and leg
 *      length -- exactly one sign combination is stable, robustly, once
 *      the hip lock is reasonably stiff.
 *   2. That sign combination was then run through wheeled_biped.m's full
 *      NONLINEAR closed-loop simulation for forward, zero, and backward
 *      velocity targets, all three converging to the commanded velocity
 *      with the plant staying upright.
 * The result:
 *     pitch_sp  = -Kp_vel * (vel_tgt - vel_meas)      [note the minus sign]
 *     wheel_tq  = +Kp_pitch * (pitch_sp - pitch) + D-term
 * with Kp_vel, Kp_pitch both POSITIVE, ordinary-sense tuning gains. If this
 * ever needs re-deriving (new gain structure, different plant assumptions),
 * redo it the same way -- against the model, not by inspection.
 *
 * Steering/yaw is NOT handled by this controller (input[YAW_STICK] is
 * unused) -- it was out of scope for "outer loop = velocity, inner loop =
 * pitch" as specified. Add a differential-wheel-torque term (or its own
 * loop) separately if turning is needed before the LQR controller replaces
 * this one.
 */

PidBalanceController::PidBalanceController()
    : _vel_pid(-VEL_KP, -VEL_KI, -VEL_KD, VEL_IMAX),
      _pitch_pid(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_IMAX),
      _hip_lock()
{}

void PidBalanceController::reset()
{
    _vel_pid.reset();
    _pitch_pid.reset();
    _hip_lock.reset();
}

void PidBalanceController::update(const float state[StateIdx::N],
                                   const float input[InputIdx::N_INPUTS],
                                   float       motor_torques[6])
{
    Quat q = { state[StateIdx::Q0], state[StateIdx::Q1], state[StateIdx::Q2], state[StateIdx::Q3] };
    float roll, pitch, yaw;
    quat_to_euler(q, roll, pitch, yaw);
    (void)roll; (void)yaw;

    // Velocity estimate comes from the state manager (wheel-encoder-primary
    // fusion, see StateManager::update step 2.5 / controls_plan.md).
    const float vel_meas = state[StateIdx::U];
    const float vel_tgt  = input[InputIdx::VEL_TGT] * MAX_VELOCITY_MPS;
    const float vel_err  = vel_tgt - vel_meas;

    // Outer loop: velocity error -> pitch setpoint (sign per the derivation
    // above -- the PID is constructed with NEGATIVE gains to realize it).
    float pitch_sp = _vel_pid.update(vel_err);
    pitch_sp = constrain_float(pitch_sp, -MAX_LEAN_RAD, MAX_LEAN_RAD);

    // Inner loop: pitch error -> wheel torque, split evenly across both
    // wheels (no differential/yaw mixing -- see class header).
    const float pitch_err   = pitch_sp - pitch;
    const float wheel_torque = constrain_float(_pitch_pid.update(pitch_err),
                                                -WHEEL_TORQUE_LIMIT_NM, WHEEL_TORQUE_LIMIT_NM);

    float hip_torques[4];
    _hip_lock.update(hip_torques);

    motor_torques[0] = hip_torques[0];
    motor_torques[1] = hip_torques[1];
    motor_torques[2] = hip_torques[2];
    motor_torques[3] = hip_torques[3];
    motor_torques[4] = wheel_torque;
    motor_torques[5] = wheel_torque;
}
