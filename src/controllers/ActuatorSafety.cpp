#include "src/controllers/ActuatorSafety.hpp"
#include "src/coms/CANMotor.hpp"
#include "src/math/math.hpp"

// Out-of-class definitions required pre-C++17 for constexpr array members
// that are odr-used (indexed at runtime below) rather than just read as
// compile-time constants.
constexpr float ActuatorSafety::HIP_ANGLE_MIN_RAD[4];
constexpr float ActuatorSafety::HIP_ANGLE_MAX_RAD[4];

// CAN ids 1..6 = hip FL, FR, RL, RR, wheel L, wheel R (main.cpp's
// can_motor_register order) — matches motor_torques[]'s own ordering.
static constexpr uint8_t MOTOR_IDS[6] = {1, 2, 3, 4, 5, 6};

float ActuatorSafety::limit_scale(float value, float lo, float hi, float margin, float torque)
{
    if (torque > 0.0f) {
        // Pushing toward `hi` — room shrinks to 0 at the bound, negative past it.
        return constrain_float((hi - value) / margin, 0.0f, 1.0f);
    }
    if (torque < 0.0f) {
        return constrain_float((value - lo) / margin, 0.0f, 1.0f);
    }
    return 1.0f;
}

void ActuatorSafety::apply(float motor_torques[6]) const
{
    for (int i = 0; i < 6; ++i) {
        const bool is_hip = (i < 4);

        CanMotorState ms = {};
        if (!can_motor_get_state(MOTOR_IDS[i], &ms) || !ms.valid) {
            // No verified feedback for this motor -- can't confirm it's
            // within any limit, so command nothing rather than guess.
            motor_torques[i] = 0.0f;
            continue;
        }

        float t = motor_torques[i];

        if (is_hip) {
            const float angle_scale = limit_scale(ms.pos_rad, HIP_ANGLE_MIN_RAD[i], HIP_ANGLE_MAX_RAD[i],
                                                   HIP_ANGLE_SOFT_MARGIN_RAD, t);
            const float vel_scale   = limit_scale(ms.vel_rads, -HIP_VEL_LIMIT_RADS, HIP_VEL_LIMIT_RADS,
                                                   HIP_VEL_SOFT_MARGIN_RADS, t);
            t = constrain_float(t * angle_scale * vel_scale, -HIP_TORQUE_LIMIT_NM, HIP_TORQUE_LIMIT_NM);
        } else {
            const float vel_scale = limit_scale(ms.vel_rads, -WHEEL_VEL_LIMIT_RADS, WHEEL_VEL_LIMIT_RADS,
                                                 WHEEL_VEL_SOFT_MARGIN_RADS, t);
            t = constrain_float(t * vel_scale, -WHEEL_TORQUE_LIMIT_NM, WHEEL_TORQUE_LIMIT_NM);
        }

        motor_torques[i] = t;
    }
}
