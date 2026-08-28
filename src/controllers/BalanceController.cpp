#include "src/controllers/BalanceController.hpp"
#include <cstring>

void BalanceController::update(const float state[StateIdx::N],
                                const float input[InputIdx::N_INPUTS],
                                float       motor_torques[6])
{
    // Stub: zero torque on all motors until balance algorithm is implemented.
    // TODO: implement LQR / whole-body controller here.
    //   state[StateIdx::Q0..Q3] — body attitude quaternion
    //   state[StateIdx::P,Q,R] — body angular rates
    //   input[InputIdx::ROLL_TGT] — lean setpoint
    //   input[InputIdx::YAW_RATE] — yaw rate demand
    //   input[InputIdx::THRUST]   — forward speed demand
    (void)state;
    (void)input;
    memset(motor_torques, 0, 6 * sizeof(float));
}
