#include "src/controllers/ActuatorSafety.hpp"
#include "src/coms/CANMotor.hpp"
#include "src/math/math.hpp"

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

        // Hips: no limiting here -- CANMotor.cpp's hip safety gate already
        // clamps whatever can_motor_set_torque() is about to be called with
        // below, at the source, regardless of caller. See ActuatorSafety.hpp.
        if (is_hip) continue;

        float t = motor_torques[i];
        const float vel_scale = limit_scale(ms.vel_rads, -WHEEL_VEL_LIMIT_RADS, WHEEL_VEL_LIMIT_RADS,
                                             WHEEL_VEL_SOFT_MARGIN_RADS, t);
        motor_torques[i] = constrain_float(t * vel_scale, -WHEEL_TORQUE_LIMIT_NM, WHEEL_TORQUE_LIMIT_NM);
    }
}
