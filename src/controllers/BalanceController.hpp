#pragma once
#include "src/RobotState.hpp"

/*
 * Balance controller stub for the 5-bar linkage wheeled biped.
 *
 * Inputs:  EKF state vector (attitude, rates, position)
 *          RC inputs (lean setpoint, yaw rate, forward speed)
 * Outputs: motor_torques[6] in Nm
 *            [0..3] = hip motors (LKMTECH MG8016E-i6)
 *            [4..5] = wheel motors (Steadywin GIM6010-6)
 *
 * Replace the stub body with the actual whole-body controller.
 * Consider: LQR / MPC for balance, separate velocity loop for wheels.
 */

class BalanceController {
public:
    // Called at 400 Hz from RobotStateMachine when in BALANCING mode.
    void update(const float state[StateIdx::N],
                const float input[InputIdx::N_INPUTS],
                float       motor_torques[6]);
};
