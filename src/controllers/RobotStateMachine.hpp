#pragma once
#include "src/RobotState.hpp"

/*
 * Robot state machine for the wheeled biped.
 *
 * States:
 *   ROBOT_IDLE        — disarmed: all motors zero-torque, waiting for arm
 *   ROBOT_STANDING_UP — armed, MODE_SW (ch6/AuxB) ≥ 0, just entered from
 *                        IDLE/CAR: transitional state (StandUpController)
 *                        — balances to upright and zero velocity while
 *                        slowly extending the leg toward the standing
 *                        height setpoint. Neither VEL_TGT nor HEIGHT_SET
 *                        is read here (see StandUpController.hpp) — the
 *                        robot regulates to zero velocity and ramps to a
 *                        fixed leg length, unconditionally. Transitions to
 *                        ROBOT_BALANCING once StandUpController::is_standing()
 *                        reports true.
 *   ROBOT_BALANCING   — active balance controller running (BalanceController
 *                        — PID or LQR per its own compile-time flag). Only
 *                        reachable via ROBOT_STANDING_UP, never entered
 *                        directly from IDLE/CAR — see update()'s transition
 *                        logic. VEL_TGT and HEIGHT_SET become live here.
 *   ROBOT_CAR         — armed, MODE_SW < 0: direct wheel drive, no
 *                        balancing (CarController) — hips left idling
 *                        (zero torque; no crouch hold, needs leg-length
 *                        control from a car-mode height stick if that's
 *                        ever wanted — not in scope here), wheels driven
 *                        from InputIdx::YAW_STICK / VEL_TGT via the
 *                        ODrive's own native velocity mode (see
 *                        CarController.hpp / CANMotor.hpp's
 *                        can_motor_set_odrive_mode()). HEIGHT_SET/LEANOVER
 *                        are not read.
 *
 * On every wheel-velocity-mode entry/exit edge (ROBOT_CAR, ROBOT_STANDING_UP,
 * and ROBOT_BALANCING-when-BalanceController::USES_WHEEL_VELOCITY all want
 * it; ROBOT_IDLE and PID-selected ROBOT_BALANCING don't — see .cpp's
 * wants_wheel_velocity()), update() itself switches ids 5/6 (the wheels)
 * into/out of ODrive velocity mode. WheelSendThread (threads.cpp) reads
 * wheel_vel_L()/_R() every tick while wheel_velocity_mode() is true instead
 * of the normal torque path.
 *
 * Called from ControlThread at 400 Hz.
 */

enum RobotMode { ROBOT_IDLE = 0, ROBOT_STANDING_UP, ROBOT_BALANCING, ROBOT_CAR };

class RobotStateMachine {
public:
    // Update the state machine.  Reads g_state[], g_input[], g_armed.
    // Writes desired motor torques into motor_torques[6]. Wheel entries
    // ([4]/[5]) stay zero and are UNUSED whenever wheel_velocity_mode() is
    // true -- see wheel_vel_L()/_R() below.
    void update(const float state[StateIdx::N],
                const float input[InputIdx::N_INPUTS],
                bool        armed,
                float       motor_torques[6]);

    RobotMode mode() const { return _mode; }

    // True while the wheels should be driven via ODrive velocity mode
    // (can_motor_set_velocity(), through wheel_vel_L()/_R()) rather than
    // the torque path (motor_torques[4]/[5]) -- see .cpp's
    // wants_wheel_velocity(). WheelSendThread (threads.cpp) uses this.
    bool  wheel_velocity_mode() const;
    float wheel_vel_L() const;
    float wheel_vel_R() const;

private:
    RobotMode _mode = ROBOT_IDLE;
};
