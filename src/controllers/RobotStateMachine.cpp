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

    // Mode switch (ch6/AuxB): MODE_SW < 0 → CAR, ≥ 0 → BALANCING.
    _mode = (input[InputIdx::MODE_SW] >= 0.0f) ? ROBOT_BALANCING : ROBOT_CAR;

    if (_mode == ROBOT_BALANCING) {
        s_balance.update(state, input, motor_torques);
    }
    // ROBOT_CAR: STUBBED — falls through with the zero torque set above.
    // Planned (not yet implemented, see RobotStateMachine.hpp / README.md):
    // hips locked at a fixed crouch, wheels driven directly from
    // YAW_STICK/VEL_TGT, plus the stand-up/crouch transition between this
    // and ROBOT_BALANCING. All of that needs leg-length control
    // (FiveBarIK, controls_plan.md sections 2-3) that doesn't exist yet.
}
