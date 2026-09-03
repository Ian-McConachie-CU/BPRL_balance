#pragma once
#include "src/RobotState.hpp"

/*
 * CarController — ROBOT_CAR mode direct wheel drive (see RobotStateMachine /
 * README.md's "planned mode state machine"). No balancing, no leg motion:
 * hips are left idling (zero torque -- RobotStateMachine::update() leaves
 * motor_torques[0..3] at the zero it memsets on entry, this class never
 * touches them) and HEIGHT_SET/LEANOVER are not read (no leg-length control
 * yet, see controls_plan.md sections 2-3 / README.md's roadmap).
 *
 * Wheels are driven via the ODrive's OWN native velocity mode -- same
 * rationale MotorTest's wheel sweep found on real hardware (see its
 * header comment and CANMotor.hpp's can_motor_set_odrive_mode()): a
 * software torque-PID velocity loop stalled under the wheel's own static
 * friction, where commanding the ODrive's own velocity controller
 * directly tracked well. ROBOT_STANDING_UP and ROBOT_BALANCING (under
 * BALANCE_CTRL_LQR) now use the same velocity-mode wheel path for the
 * same reason -- see StandUpController.hpp / LqrBalanceController.hpp;
 * only BALANCE_CTRL_PID's HipLock-based cascade still uses the torque
 * path. RobotStateMachine::update() is responsible for switching ids 5/6
 * into/out of velocity mode on every relevant mode entry/exit (see its
 * wants_wheel_velocity()) -- this class only DECIDES the target rad/s,
 * mirroring MotorTest's own split (see its header comment): WheelSendThread
 * reads RobotStateMachine::wheel_vel_L()/_R() every tick (which forwards to
 * this class's left()/right() while in ROBOT_CAR) and sends
 * can_motor_set_velocity() itself, kept off the torque path in
 * g_motor_torques[4]/[5] entirely -- see threads.cpp.
 *
 * Mixing: VEL_TGT scales directly to a common-mode wheel speed (both wheels
 * the same sign/magnitude); YAW_STICK adds a differential term on top,
 * positive yaw (right turn) speeding up the LEFT wheel and slowing the
 * RIGHT -- see update()'s comment. id5=L, id6=R (main.cpp's
 * can_motor_register calls); each drive's own `sign` param there already
 * makes a positive command mean the same physical forward direction for
 * both wheels, so no extra mirroring is needed here.
 */
class CarController {
public:
    void update(const float input[InputIdx::N_INPUTS]);
    void reset();

    // Last computed wheel velocity targets [rad/s]. Valid only while
    // RobotStateMachine::mode() == ROBOT_CAR; WheelSendThread reads these
    // (via RobotStateMachine::wheel_vel_L()/_R()) every tick regardless of
    // whether that tick recomputed them (same ZOH pattern as
    // MotorTest::wheel_velocity_target()).
    float left()  const { return _vel_L; }
    float right() const { return _vel_R; }

private:
    // ── Tuning knobs — placeholders, tune on the bench ─────────────────────
    // Common-mode wheel speed at full VEL_TGT deflection.
    static constexpr float MAX_WHEEL_VEL_RADS = 10.0f;
    // Differential (per-wheel) contribution at full YAW_STICK deflection.
    static constexpr float YAW_MIX_RADS       = 5.0f;
    // Output clamp, PER WHEEL, applied after mixing (so a combined vel+yaw
    // command can't exceed this even though neither term alone would).
    // Deliberately under ActuatorSafety's WHEEL_VEL_LIMIT_RADS (60 rad/s,
    // ActuatorSafety.hpp) -- CAR mode's velocity-mode commands bypass that
    // gate entirely (same as MotorTest's wheel sweep does), so this is the
    // only limit protecting the wheels here.
    static constexpr float WHEEL_VEL_LIMIT_RADS = 55.0f;

    float _vel_L = 0.0f;
    float _vel_R = 0.0f;
};
