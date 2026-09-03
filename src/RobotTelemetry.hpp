#pragma once
#include "ch.h"
#include "src/RobotState.hpp"
#include "src/coms/CANMotor.hpp"

class StateManager;

/*
 * RobotTelemetry — everything requested for tracking/logging that ISN'T a
 * Kalman-filtered quantity (those live in StateIdx/g_state instead). See
 * telemetry_plan.md's Architecture section for why these are two separate
 * containers: per-leg (not averaged) FK breakdown, individual hip/wheel
 * telemetry, battery voltage/current, and world-frame velocity are all
 * either direct sensor readings or simple derived computations, not
 * something that benefits from Kalman fusion.
 *
 * Populated once per StateEstThread tick (500 Hz) — same "compute once,
 * publish under a mutex, everyone reads a snapshot" pattern g_state/g_input
 * already use.
 *
 * RC stick inputs (g_input[]/InputIdx) and robot state-machine mode
 * (RobotStateMachine::mode()) are deliberately NOT duplicated here — they
 * already exist and are already at full rate.
 */
struct RobotTelemetry {
    float leg_L[2], leg_L_dot[2];       // m, m/s        (0=left, 1=right) -- PER-LEG, not averaged
    float leg_thL[2], leg_thL_dot[2];   // rad, rad/s
    float leg_x_dot[2], leg_z_dot[2];   // m/s, foot-point velocity rel. to body, NED convention
    bool  leg_valid[2];

    float hip_pos_rad[4], hip_vel_rads[4];   // ids 1-4, FL/FR/RL/RR
    float wheel_vel_rads[2];                  // ids 5-6, L/R

    float battery_voltage_V;         // average of 4 hips' voltage_V (0x9A telemetry) -- literal per request
    float power_monitor_voltage_V;   // Matek CAN-L4-BM voltage -- independent cross-check
    float total_current_A;           // Matek CAN-L4-BM current

    float x_dot, z_dot;              // m/s, world/NED-frame velocity -- derived, NOT a Kalman state

    bool valid;
};

extern mutex_t        telemetry_mtx;
extern RobotTelemetry g_telemetry;

// Call once per StateEstThread tick, after state_mgr.get_state(). state[]:
// the just-published StateIdx::N-element state vector. state_mgr: same
// instance StateEstThread already owns (read-only access to get_leg_state()).
// hips/wheels: the same CanMotorState snapshots update_legs_and_wheels()
// was called with this tick (no new CAN reads needed).
void telemetry_update(const float state[StateIdx::N], const StateManager& state_mgr,
                       const CanMotorState hips[4], const CanMotorState wheels[2]);
