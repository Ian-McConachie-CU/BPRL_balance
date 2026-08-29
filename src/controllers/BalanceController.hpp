#pragma once
#include "src/RobotState.hpp"
#include "src/controllers/PidBalanceController.hpp"
#include "src/controllers/LqrBalanceController.hpp"

/*
 * BalanceController — dispatches to one of two balance controllers based
 * on input[InputIdx::CTRL_SEL], so both can be bench-tested / A-B compared
 * without reflashing:
 *
 *   CTRL_SEL <  0  ->  PidBalanceController  (Stage 0: SLC PID cascade,
 *                       hips locked — see that class's header)
 *   CTRL_SEL >= 0  ->  LqrBalanceController  (Stage 1: gain-scheduled LQR,
 *                       gains stubbed at zero pending MATLAB tuning)
 *
 * Inputs:  EKF state vector (attitude, rates, position, velocity)
 *          RC inputs (velocity target, controller select, ...)
 * Outputs: motor_torques[6] in Nm
 *            [0..3] = hip motors (LKMTECH MG8016E-i6)
 *            [4..5] = wheel motors (Steadywin GIM6010-6)
 */
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
