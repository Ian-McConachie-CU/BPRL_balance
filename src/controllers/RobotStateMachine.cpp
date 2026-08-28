#include "src/controllers/RobotStateMachine.hpp"
#include "src/controllers/BalanceController.hpp"
#include <cstring>

static BalanceController s_balance;

void RobotStateMachine::update(const float state[StateIdx::N],
                                const float input[InputIdx::N_INPUTS],
                                bool        armed,
                                float       motor_torques[6])
{
    // Default: all motors zero torque
    memset(motor_torques, 0, 6 * sizeof(float));

    if (!armed) {
        _mode = ROBOT_IDLE;
        return;
    }

    // Mode switch: MODE_SW < 0 → IDLE, ≥ 0 → BALANCING
    _mode = (input[InputIdx::MODE_SW] >= 0.0f) ? ROBOT_BALANCING : ROBOT_IDLE;

    if (_mode == ROBOT_BALANCING) {
        s_balance.update(state, input, motor_torques);
    }
}
