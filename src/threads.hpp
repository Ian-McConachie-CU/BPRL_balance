#pragma once
#include "ch.h"
#include "hal.h"
#include "src/RobotState.hpp"

/* ── Shared raw sensor data types ────────────────────────────────────────── */

struct IMURaw {
    float accel[3];  // m/s² (X fwd, Y right, Z down)
    float gyro[3];   // rad/s
    bool  valid;
};

struct CANIMURaw {
    // Quaternion NED→Body from IMX5 CID_INS_QUATN2B [W,X,Y,Z], Hamilton convention.
    float q0, q1, q2, q3;
    bool  has_new_quat;

    // Body-frame angular rates from IMX5 (100 Hz)
    float p, q, r;           // rad/s
    float ax, ay, az;        // m/s²
    bool  has_new_rates;

    bool  valid;
};

struct MocapRaw {
    float x, y, z;    // NED position (m)
    float vx, vy, vz; // NED velocity (m/s)
    bool  has_new;
    bool  valid;
};

/* ── Shared robot state — access only under respective mutex ──────────────
 * Defined in threads.cpp.                                                   */
extern mutex_t state_mtx;
extern float   g_state[StateIdx::N];    // full 19-element EKF state
extern float   g_euler[3];              // [roll, pitch, yaw] (rad)
extern float   g_input[InputIdx::N_INPUTS];  // RC inputs
extern float   g_motor_torques[6];      // motor torque commands [Nm] × 6
extern bool    g_armed;

extern mutex_t imu_mtx;
extern IMURaw  g_imu[3];    // [0]=ICM-20948 primary, [1]=ext, [2]=ICM-20602

extern mutex_t   can_imu_mtx;
extern CANIMURaw g_can_imu;

extern mutex_t  mocap_mtx;
extern MocapRaw g_mocap;

/* ── Thread rates — passed as arg by main ────────────────────────────────── */

struct LogRates {
    sysinterval_t period;   // 50 Hz → TIME_MS2I(20)
};

struct ThreadRates {
    sysinterval_t spi;        // SPIThread        1 kHz
    sysinterval_t est;        // StateEstThread   500 Hz
    sysinterval_t control;    // ControlThread    200 Hz (hips + compute)
    sysinterval_t wheel_send; // WheelSendThread  300 Hz (wheels only, see threads.cpp)
    sysinterval_t radio;      // RadioThread      100 Hz
    sysinterval_t heartbeat;  // HeartbeatThread  5 Hz
    sysinterval_t debug;      // DebugThread      10 Hz (BPRL_DEBUG only)
    LogRates       log;       // LogThread        50 Hz
};

/* ── Thread entry points ─────────────────────────────────────────────────── */
void threads_start(const ThreadRates &rates);
