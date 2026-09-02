#include "src/controllers/BalanceController.hpp"

void BalanceController::update(const float state[StateIdx::N],
                                const float input[InputIdx::N_INPUTS],
                                float       motor_torques[6])
{
#if BALANCE_CONTROLLER == BALANCE_CTRL_LQR
    _lqr.update(state, input, motor_torques);
#else
    _pid.update(state, input, motor_torques);
#endif
}
