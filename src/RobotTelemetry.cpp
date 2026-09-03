#include "src/RobotTelemetry.hpp"
#include "src/state_estimator/StateManager.hpp"
#include "src/coms/CANPower.hpp"
#include "src/math/math.hpp"

MUTEX_DECL(telemetry_mtx);
RobotTelemetry g_telemetry = {};

void telemetry_update(const float state[StateIdx::N], const StateManager& state_mgr,
                       const CanMotorState hips[4], const CanMotorState wheels[2])
{
    RobotTelemetry t = {};

    for (int leg = 0; leg < 2; ++leg) {
        LegState ls;
        state_mgr.get_leg_state(leg, ls);
        t.leg_L[leg]       = ls.L0;
        t.leg_L_dot[leg]   = ls.L0_dot;
        t.leg_thL[leg]     = ls.thL;
        t.leg_thL_dot[leg] = ls.thL_dot;
        t.leg_x_dot[leg]   = ls.x_dot;
        t.leg_z_dot[leg]   = ls.z_dot;
        t.leg_valid[leg]   = ls.valid;
    }

    for (int i = 0; i < 4; ++i) {
        t.hip_pos_rad[i]  = hips[i].pos_rad;
        t.hip_vel_rads[i] = hips[i].vel_rads;
    }
    for (int i = 0; i < 2; ++i) t.wheel_vel_rads[i] = wheels[i].vel_rads;

    // Battery voltage: literal average of the 4 hips' 0x9A voltage telemetry
    // (see CANMotor.cpp's can_motor_poll_status_round_robin()) -- refreshes
    // at that slower ~1Hz/hip rate, everything else here at the full
    // 500 Hz StateEstThread tick.
    float vsum = 0.0f;
    int   vn   = 0;
    for (int i = 0; i < 4; ++i) {
        if (hips[i].valid) { vsum += hips[i].voltage_V; vn++; }
    }
    t.battery_voltage_V = (vn > 0) ? vsum / (float)vn : 0.0f;

    chMtxLock(&power_mtx);
    PowerMonState pwr = g_power;
    chMtxUnlock(&power_mtx);
    t.power_monitor_voltage_V = pwr.voltage_V;
    t.total_current_A         = pwr.current_A;

    // World/NED-frame velocity — rotate body-frame U,V,W by the current
    // attitude quaternion. Not a new Kalman state: this is the same
    // R_b2n*v term EKF::predict() already computes internally for position
    // integration, just exposed here as a standalone output.
    Quat q = { state[StateIdx::Q0], state[StateIdx::Q1], state[StateIdx::Q2], state[StateIdx::Q3] };
    float R[3][3];
    quat_to_rot_body2ned(q, R);
    const float U = state[StateIdx::U], V = state[StateIdx::V], W = state[StateIdx::W];
    t.x_dot = R[0][0]*U + R[0][1]*V + R[0][2]*W;
    t.z_dot = R[2][0]*U + R[2][1]*V + R[2][2]*W;

    t.valid = true;

    chMtxLock(&telemetry_mtx);
    g_telemetry = t;
    chMtxUnlock(&telemetry_mtx);
}
