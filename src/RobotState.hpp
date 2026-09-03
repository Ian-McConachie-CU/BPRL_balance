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

    // ── Leg state — both legs averaged, directly computed + lowpass
    // filtered (NOT Kalman-fused, see EKF.hpp's header comment) via
    // FiveBarIK + StateManager::update_legs_and_wheels(). Per-leg (not
    // averaged) breakdown lives in RobotTelemetry instead — see
    // telemetry_plan.md's Architecture section for why these live in two
    // separate places. theta = phi - thL (NED sign convention, see
    // src/kinematics/FiveBarIK.hpp).
    constexpr int LEG_L         = 19;  // m,    virtual leg length
    constexpr int LEG_L_DOT     = 20;  // m/s
    constexpr int LEG_PITCH     = 21;  // rad,  theta = phi - thL
    constexpr int LEG_PITCH_DOT = 22;  // rad/s

    // Total current state dimension
    constexpr int N = 23;
}

namespace InputIdx {
    // RC channel assignments below match the physical transmitter — see the
    // channel-map table in Radio.hpp / README.md. Channels not listed here
    // (arm switch, and the reserved Aux channels) either bypass g_input[]
    // entirely (arm -> g_armed) or aren't wired to anything yet.
    constexpr int YAW_STICK   = 0; // ch0: yaw stick [-1, 1]
    constexpr int VEL_TGT     = 1; // ch1: forward velocity demand [-1, 1], scaled to
                                    // m/s by the balance controller's own tunable
                                    // max-velocity constant.
    constexpr int HEIGHT_SET  = 2; // ch2: height-set switch [-1, 1] -- PLACEHOLDER,
                                    // read into g_input[] but not yet consumed by any
                                    // controller (needs FiveBarIK leg-length control,
                                    // see controls_plan.md sections 2-3). RadioThread
                                    // forces this to 0 on arm until the stick is first
                                    // brought back to zero, reset every disarm -- see
                                    // threads.cpp.
    constexpr int LEANOVER    = 3; // ch3: leanover switch [-1, 1] -- PLACEHOLDER, ditto.
    constexpr int MODE_SW     = 4; // ch6 (AuxB): car/balance mode select [-1,1];
                                    // <0=IDLE/CAR, >=0=BALANCE (see RobotStateMachine).
    constexpr int N_INPUTS    = 5;
}
