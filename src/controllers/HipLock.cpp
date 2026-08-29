#include "src/controllers/HipLock.hpp"
#include "src/coms/CANMotor.hpp"
#include "src/math/math.hpp"

// CAN ids 1..4 = hip FL, FR, RL, RR (see main.cpp's can_motor_register calls).
static constexpr uint8_t HIP_IDS[4] = {1, 2, 3, 4};

HipLock::HipLock()
    : _pid{ PID(KP, KI, KD, IMAX), PID(KP, KI, KD, IMAX),
            PID(KP, KI, KD, IMAX), PID(KP, KI, KD, IMAX) }
{}

void HipLock::reset()
{
    for (int i = 0; i < 4; ++i) _pid[i].reset();
}

void HipLock::update(float hip_torques[4])
{
    const float targets[4] = { TARGET_FL_RAD, TARGET_FR_RAD, TARGET_RL_RAD, TARGET_RR_RAD };

    for (int i = 0; i < 4; ++i) {
        CanMotorState ms = {};
        if (!can_motor_get_state(HIP_IDS[i], &ms) || !ms.valid) {
            // No feedback yet (motor not registered / no frame received) --
            // command zero rather than run the PID open-loop on stale data.
            hip_torques[i] = 0.0f;
            continue;
        }
        const float error = targets[i] - ms.pos_rad;
        hip_torques[i] = constrain_float(_pid[i].update(error), -TORQUE_LIMIT_NM, TORQUE_LIMIT_NM);
    }
}
