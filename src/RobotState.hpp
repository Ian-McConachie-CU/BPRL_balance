#pragma once
#include <cstdint>

/*
 * Shared robot state indices — 19-state EKF output written by StateEstThread,
 * read by ControlThread, LogThread, and DebugThread.
 *
 * Units are SI throughout:
 *   Position:    metres         (NED inertial frame)
 *   Velocity:    m/s            (body frame)
 *   Accel:       m/s²           (body frame, gravity-corrected)
 *   Quaternion:  dimensionless  (NED→Body, Hamilton [W,X,Y,Z], scalar-first)
 *   Rates:       rad/s          (body frame)
 *   Ang. accel:  rad/s²         (body frame, 50 Hz lowpass filtered)
 *
 * Future: joint angles and joint rates will be appended beyond index N (19)
 * once the leg kinematics are added to the EKF.
 */
namespace StateIdx {
    // ── Position — inertial NED frame (m) ────────────────────────────────
    constexpr int X     = 0;
    constexpr int Y     = 1;
    constexpr int Z_POS = 2;

    // ── Translational velocity — body frame (m/s) ─────────────────────────
    constexpr int U     = 3;
    constexpr int V     = 4;
    constexpr int W     = 5;

    // ── Translational acceleration — body frame, gravity-corrected (m/s²) ─
    constexpr int U_DOT = 6;
    constexpr int V_DOT = 7;
    constexpr int W_DOT = 8;

    // ── Quaternion NED→Body [W,X,Y,Z] Hamilton, scalar-first ─────────────
    constexpr int Q0    = 9;
    constexpr int Q1    = 10;
    constexpr int Q2    = 11;
    constexpr int Q3    = 12;

    // ── Angular velocity — body frame (rad/s) ─────────────────────────────
    constexpr int P     = 13;
    constexpr int Q     = 14;
    constexpr int R     = 15;

    // ── Angular acceleration — body frame, 50 Hz lowpass filtered (rad/s²) ─
    constexpr int P_DOT = 16;
    constexpr int Q_DOT = 17;
    constexpr int R_DOT = 18;

    // Total current state dimension (joint angles added in future EKF extension)
    constexpr int N = 19;
}

namespace InputIdx {
    constexpr int THRUST      = 0; // forward/speed demand [0, 1] -- legacy drone-style
                                    // throttle input, unused by the balance controllers
    constexpr int ROLL_TGT    = 1; // lean / balance setpoint [-1, 1]
    constexpr int PITCH_TGT   = 2; // pitch setpoint [-1, 1]
    constexpr int YAW_RATE    = 3; // yaw rate demand [-1, 1]
    constexpr int MODE_SW     = 4; // mode switch [-1,1]; <0=IDLE, >=0=BALANCE
    constexpr int VEL_TGT     = 5; // forward velocity demand [-1, 1], scaled to m/s by
                                    // the balance controller's own tunable max-velocity
                                    // constant. RC channel/switch assignment not yet
                                    // finalized on the transmitter -- see Radio.cpp.
    constexpr int CTRL_SEL    = 6; // balance controller select: <0 = PID cascade,
                                    // >=0 = LQR. RC channel TBD, see Radio.cpp.
    constexpr int N_INPUTS    = 7;
}
