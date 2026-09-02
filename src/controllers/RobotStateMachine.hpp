#pragma once
#include "src/RobotState.hpp"

/*
 * Robot state machine for the wheeled biped.
 *
 * States:
 *   ROBOT_IDLE       — disarmed: all motors zero-torque, waiting for arm
 *   ROBOT_BALANCING  — armed, MODE_SW (ch6/AuxB) ≥ 0: active balance
 *                       controller running
 *   ROBOT_CAR        — armed, MODE_SW < 0. STUBBED: currently zero-torque,
 *                       same as IDLE, but reported as a distinct mode.
 *                       Planned: hips locked at a fixed crouch, wheels
 *                       driven directly from InputIdx::YAW_STICK / VEL_TGT
 *                       (no balancing) — see the planned mode state machine
 *                       in README.md. Also planned but unimplemented:
 *                       stand-up/crouch transitions between this and
 *                       ROBOT_BALANCING — needs leg-length control
 *                       (FiveBarIK, controls_plan.md sections 2-3) that
 *                       doesn't exist yet.
 *
 * Called from ControlThread at 400 Hz.
 */

enum RobotMode { ROBOT_IDLE = 0, ROBOT_BALANCING, ROBOT_CAR };

class RobotStateMachine {
public:
    // Update the state machine.  Reads g_state[], g_input[], g_armed.
    // Writes desired motor torques into motor_torques[6].
    void update(const float state[StateIdx::N],
                const float input[InputIdx::N_INPUTS],
                bool        armed,
                float       motor_torques[6]);

    RobotMode mode() const { return _mode; }

private:
    RobotMode _mode = ROBOT_IDLE;
};
