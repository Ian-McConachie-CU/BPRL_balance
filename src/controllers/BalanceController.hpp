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
 *   BALANCE_CTRL_PID             ->  PidBalanceController (Stage 0: SLC PID
 *                                     cascade, hips locked — see that
 *                                     class's header)
 *   BALANCE_CTRL_LQR (default)   ->  LqrBalanceController (Stage 1:
 *                                     gain-scheduled LQR against the
 *                                     velocity-controlled-wheel reduced
 *                                     model — real gains from
 *                                     MatLab_controls/export_wheel_balance_gains.m,
 *                                     see src/controllers/WheelBalanceGainTable.hpp)
 *
 * Inputs:  EKF state vector (attitude, rates, position, velocity, leg state)
 *          RC inputs (velocity target, height set, ...)
 * Outputs: motor_torques[6] in Nm — [0..3] hip motors; [4..5] left zero and
 *          UNUSED under BALANCE_CTRL_LQR (wheels run in ODrive velocity
 *          mode instead — see wheel_vel_L()/_R() below and
 *          RobotStateMachine.hpp), only meaningful under BALANCE_CTRL_PID.
 */
#define BALANCE_CTRL_PID  0
#define BALANCE_CTRL_LQR  1

#ifndef BALANCE_CONTROLLER
#define BALANCE_CONTROLLER  BALANCE_CTRL_LQR
#endif

class BalanceController {
public:
    // True when the compiled-in controller drives the wheels via ODrive
    // velocity mode (can_motor_set_velocity(), see wheel_vel_L()/_R()
    // below) rather than the torque path (motor_torques[4]/[5]).
    // RobotStateMachine/WheelSendThread use this instead of hardcoding a
    // mode check — see RobotStateMachine.cpp's wants_wheel_velocity().
    static constexpr bool USES_WHEEL_VELOCITY = (BALANCE_CONTROLLER == BALANCE_CTRL_LQR);

    // Called at 400 Hz from RobotStateMachine when in BALANCING mode.
    void update(const float state[StateIdx::N],
                const float input[InputIdx::N_INPUTS],
                float       motor_torques[6]);

    // Called once on the ROBOT_STANDING_UP -> ROBOT_BALANCING transition
    // (RobotStateMachine) so whichever controller is compiled in starts
    // from a sane state -- currently only meaningful for the LQR path
    // (seeds its leg-height ramp from the current measured leg length,
    // see LqrBalanceController::reset()); a no-op for PID (HipLock's
    // fixed targets need no such handoff, and PidBalanceController is
    // otherwise untouched by this session's work).
    void reset(const float state[StateIdx::N]);

    // Meaningless unless USES_WHEEL_VELOCITY -- see RobotStateMachine's
    // wheel_velocity_mode()/wheel_vel_L()/_R().
    float wheel_vel_L() const { return _lqr.wheel_vel_L(); }
    float wheel_vel_R() const { return _lqr.wheel_vel_R(); }

private:
    PidBalanceController _pid;
    LqrBalanceController _lqr;
};
