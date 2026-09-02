#pragma once
#include "src/RobotState.hpp"
#include "src/controllers/PidBalanceController.hpp"
#include "src/controllers/LqrBalanceController.hpp"

/*
 * BalanceController — dispatches to one of two balance controllers,
 * selected at COMPILE TIME via BALANCE_CONTROLLER below (no spare RC
 * channel is assigned to this — the transmitter's channels are all spoken
 * for, see the channel map in Radio.hpp / README.md):
 *
 *   BALANCE_CTRL_PID (default)  ->  PidBalanceController (Stage 0: SLC PID
 *                                    cascade, hips locked — see that
 *                                    class's header)
 *   BALANCE_CTRL_LQR            ->  LqrBalanceController (Stage 1:
 *                                    gain-scheduled LQR, gains stubbed at
 *                                    zero pending MATLAB tuning)
 *
 * Inputs:  EKF state vector (attitude, rates, position, velocity)
 *          RC inputs (velocity target, ...)
 * Outputs: motor_torques[6] in Nm
 *            [0..3] = hip motors (LKMTECH MG8016E-i6)
 *            [4..5] = wheel motors (Steadywin GIM6010-6)
 */
#define BALANCE_CTRL_PID  0
#define BALANCE_CTRL_LQR  1

#ifndef BALANCE_CONTROLLER
#define BALANCE_CONTROLLER  BALANCE_CTRL_PID
#endif

class BalanceController {
public:
    // Called at 400 Hz from RobotStateMachine when in BALANCING mode.
    void update(const float state[StateIdx::N],
                const float input[InputIdx::N_INPUTS],
                float       motor_torques[6]);

private:
    PidBalanceController _pid;
    LqrBalanceController _lqr;
};
