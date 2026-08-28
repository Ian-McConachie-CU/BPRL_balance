#pragma once
#include "src/RobotState.hpp"

/*
 * Robot state machine for the wheeled biped.
 *
 * States:
 *   ROBOT_IDLE       — all motors zero-torque, waiting for arm
 *   ROBOT_BALANCING  — active balance controller running
 *   ROBOT_MANUAL     — direct velocity control from RC sticks
 *
 * Called from ControlThread at 400 Hz.
 */

enum RobotMode { ROBOT_IDLE = 0, ROBOT_BALANCING, ROBOT_MANUAL };

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
