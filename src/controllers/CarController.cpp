#include "src/controllers/CarController.hpp"
#include "src/math/math.hpp"

void CarController::reset()
{
    _vel_L = 0.0f;
    _vel_R = 0.0f;
}

void CarController::update(const float input[InputIdx::N_INPUTS])
{
    const float vel_cmd = input[InputIdx::VEL_TGT]   * MAX_WHEEL_VEL_RADS;
    const float yaw_cmd = input[InputIdx::YAW_STICK] * YAW_MIX_RADS;

    // Positive yaw = right turn = left wheel faster, right wheel slower
    // (differential steering) -- see class header.
    _vel_L = constrain_float(vel_cmd + yaw_cmd, -WHEEL_VEL_LIMIT_RADS, WHEEL_VEL_LIMIT_RADS);
    _vel_R = constrain_float(vel_cmd - yaw_cmd, -WHEEL_VEL_LIMIT_RADS, WHEEL_VEL_LIMIT_RADS);
}
