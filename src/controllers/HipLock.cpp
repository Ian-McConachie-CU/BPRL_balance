#include "src/controllers/HipLock.hpp"
#include "src/coms/CANMotor.hpp"
#include "src/kinematics/LegParams.hpp"
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

void HipLock::update(float hip_torques[4], float L_target, float thL_target)
{
    float phi1_target, phi4_target;
    if (!ik(LEG_PARAMS, L_target, thL_target, phi1_target, phi4_target)) {
        for (int i = 0; i < 4; ++i) hip_torques[i] = 0.0f;
        return;
    }
    // FL/FR (front, ids 1/2) = phi4; RL/RR (rear, ids 3/4) = phi1 --
    // matches StateManager.cpp's LEG_HIP_MAP.
    const float targets[4] = { phi4_target, phi4_target, phi1_target, phi1_target };

    for (int i = 0; i < 4; ++i) {
        CanMotorState ms = {};
        if (!can_motor_get_state(HIP_IDS[i], &ms) || !ms.valid) {
            hip_torques[i] = 0.0f;
            continue;
        }
        const float error = targets[i] - ms.pos_rad;
        hip_torques[i] = constrain_float(_pid[i].update(error), -TORQUE_LIMIT_NM, TORQUE_LIMIT_NM);
    }
}
