#include "src/controllers/BalanceController.hpp"

void BalanceController::update(const float state[StateIdx::N],
                                const float input[InputIdx::N_INPUTS],
                                float       motor_torques[6])
{
    // Switching leaves the inactive controller's PIDs un-updated; each PID's
    // own stale-input timeout (see PID.hpp) resets its integrator the next
    // time it's selected, so no explicit reset is needed here.
    if (input[InputIdx::CTRL_SEL] < 0.0f) {
        _pid.update(state, input, motor_torques);
    } else {
        _lqr.update(state, input, motor_torques);
    }
}
