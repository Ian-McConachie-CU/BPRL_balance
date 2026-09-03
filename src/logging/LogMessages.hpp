#pragma once
#include <cstdint>
#include <cstddef>

/*
 * Binary log record format (ArduPilot DataFlash compatible):
 *   Header:   one FMT record per message type (89 bytes each, written at file open)
 *   Data:     [0xA3][0x95][msg_id][...packed struct body...]
 *
 * FMT record layout (89 bytes total):
 *   [0xA3][0x95][0x80][type_u8][length_u8][name_4][format_16][labels_64]
 *   length = total data record size including 3-byte header = 3 + sizeof(body)
 *   format = ArduPilot type codes: Q=uint64 H=uint16 f=float i=int32 h=int16 B=uint8
 *
 * Files produced by this logger can be opened in UAV Log Viewer
 * (plot.ardupilot.org). TimeUS must be the first field for the time axis.
 *
 * To add a new log set:
 *   1. Define a packed struct and LOG_MSG_* constant below.
 *   2. Add one entry to kLogDefs[] with the format code string and labels.
 *   3. Snapshot the data and call logger.write(LOG_MSG_*, msg) in LogThread.
 */

/* ── Message IDs ─────────────────────────────────────────────────────────── */
constexpr uint8_t LOG_MSG_FMT  = 0x80U;  // schema header — written once at file open
constexpr uint8_t LOG_MSG_ATT  = 0x09U;  // angular states: attitude + rates + angular accel
constexpr uint8_t LOG_MSG_LIN  = 0x0AU;  // linear states: position + velocity + linear accel
constexpr uint8_t LOG_MSG_RCIN = 0x05U;  // RC stick inputs + arm state
constexpr uint8_t LOG_MSG_OUTP = 0x06U;  // motor torque commands [Nm] × 4 (first 4 of 6)
constexpr uint8_t LOG_MSG_HIPS = 0x07U;  // hip encoder position [rad] + bus voltage [V] × 4
constexpr uint8_t LOG_MSG_HERR = 0x08U;  // hip latched error-flag bitmask × 4
constexpr uint8_t LOG_MSG_LEGS = 0x0BU;  // per-leg length/pitch + rates, from FiveBarIK × 2
constexpr uint8_t LOG_MSG_LEGV = 0x0CU;  // per-leg foot-point Cartesian velocity [m/s] × 2
constexpr uint8_t LOG_MSG_MOTV = 0x0DU;  // hip + wheel velocity [rad/s], all 6 motors
constexpr uint8_t LOG_MSG_PWR  = 0x0EU;  // battery voltage (hip avg), power-monitor voltage, total current
constexpr uint8_t LOG_MSG_VNED = 0x0FU;  // world/NED-frame velocity (derived, not a Kalman state)

/* ── Packed message bodies ───────────────────────────────────────────────── */

struct __attribute__((packed)) LogMsgATT {
    uint64_t time_us;    // microseconds since boot (TimeUS — required by UAV Log Viewer)
    uint16_t rate_hz;    // logging rate (50)
    float    roll;       // rad  (from g_euler[0])
    float    pitch;      // rad  (from g_euler[1])
    float    yaw;        // rad  (from g_euler[2])
    float    p;          // rad/s  body-frame roll rate
    float    q;          // rad/s  body-frame pitch rate
    float    r;          // rad/s  body-frame yaw rate
    float    p_dot;      // rad/s²  body-frame roll angular acceleration
    float    q_dot;      // rad/s²  body-frame pitch angular acceleration
    float    r_dot;      // rad/s²  body-frame yaw angular acceleration
};
// Format: "QHfffffffff"   Body size: 8+2+9×4 = 46 B   Record: 49 B

struct __attribute__((packed)) LogMsgLIN {
    uint64_t time_us;
    uint16_t rate_hz;
    float    x;       // m  NED inertial position
    float    y;       // m
    float    z;       // m  (down positive)
    float    u;       // m/s  body-frame translational velocity
    float    v;       // m/s
    float    w;       // m/s
    float    u_dot;   // m/s²  body-frame translational acceleration (gravity-corrected)
    float    v_dot;   // m/s²
    float    w_dot;   // m/s²
};
// Format: "QHfffffffff"   Body: 46 B   Record: 49 B

struct __attribute__((packed)) LogMsgRCIN {
    uint64_t time_us;
    uint16_t rate_hz;
    float    yaw_stk;     // [-1, 1]  yaw stick (ch0)
    float    vel_stk;     // [-1, 1]  forward velocity target (ch1)
    float    height_stk;  // [-1, 1]  height-set switch (ch2) -- live in ROBOT_BALANCING
                            // under BALANCE_CTRL_LQR (see LqrBalanceController.hpp),
                            // unconsumed in every other mode
    float    lean_stk;    // [-1, 1]  leanover switch (ch3) -- placeholder, unconsumed
    uint8_t  armed;       // 0=disarmed, 1=armed
    uint8_t  mode;        // RobotMode: 0=IDLE, 1=STANDING_UP, 2=BALANCING, 3=CAR
};
// Format: "QHffffBB"   Body: 8+2+4×4+1+1 = 28 B   Record: 31 B

struct __attribute__((packed)) LogMsgOUTP {
    uint64_t time_us;
    uint16_t rate_hz;
    float    tq0;    // Nm  hip FL torque command
    float    tq1;    // Nm  hip FR torque command
    float    tq2;    // Nm  hip RL torque command
    float    tq3;    // Nm  hip RR torque command
};
// Format: "QHffff"   Body: 8+2+4×4 = 26 B   Record: 29 B

struct __attribute__((packed)) LogMsgHIPS {
    uint64_t time_us;
    uint16_t rate_hz;
    float    pos0;    // rad  hip FL encoder position (motor-side, from 0xA1/0xA2 reply)
    float    pos1;    // rad  hip FR
    float    pos2;    // rad  hip RL
    float    pos3;    // rad  hip RR
    float    volt0;   // V    hip FL bus voltage (0x9A RMD status1 request, see CANMotor.cpp)
    float    volt1;   // V    hip FR
    float    volt2;   // V    hip RL
    float    volt3;   // V    hip RR
};
// Format: "QHffffffff"   Body: 8+2+8×4 = 42 B   Record: 45 B

struct __attribute__((packed)) LogMsgHERR {
    uint64_t time_us;
    uint16_t rate_hz;
    uint8_t  err0;    // raw RMD errorState bitmask, hip FL (0 = no fault)
    uint8_t  err1;    // hip FR
    uint8_t  err2;    // hip RL
    uint8_t  err3;    // hip RR
};
// Format: "QHBBBB"   Body: 8+2+1×4 = 14 B   Record: 17 B

struct __attribute__((packed)) LogMsgLEGS {
    uint64_t time_us;
    uint16_t rate_hz;
    float    L0;       // m    leg 0 (left) virtual leg length
    float    L0dot;    // m/s
    float    ThL0;     // rad  leg 0 hip-relative leg angle
    float    ThL0dot;  // rad/s
    float    L1;       // m    leg 1 (right)
    float    L1dot;    // m/s
    float    ThL1;     // rad
    float    ThL1dot;  // rad/s
};
// Format: "QHffffffff"   Body: 8+2+8×4 = 42 B   Record: 45 B

struct __attribute__((packed)) LogMsgLEGV {
    uint64_t time_us;
    uint16_t rate_hz;
    float    leg0_xdot;   // m/s  leg 0 (left) foot-point velocity rel. to body, NED convention
    float    leg0_zdot;   // m/s
    float    leg1_xdot;   // m/s  leg 1 (right)
    float    leg1_zdot;   // m/s
};
// Format: "QHffff"   Body: 8+2+4×4 = 26 B   Record: 29 B

struct __attribute__((packed)) LogMsgMOTV {
    uint64_t time_us;
    uint16_t rate_hz;
    float    hip_vel0;    // rad/s  hip FL
    float    hip_vel1;    // rad/s  hip FR
    float    hip_vel2;    // rad/s  hip RL
    float    hip_vel3;    // rad/s  hip RR
    float    wheel_vel0;  // rad/s  wheel L
    float    wheel_vel1;  // rad/s  wheel R
};
// Format: "QHffffff"   Body: 8+2+6×4 = 34 B   Record: 37 B

struct __attribute__((packed)) LogMsgPWR {
    uint64_t time_us;
    uint16_t rate_hz;
    float    batt_v;    // V  battery voltage -- average of 4 hips' 0x9A telemetry
    float    pmon_v;    // V  Matek CAN-L4-BM voltage (independent cross-check)
    float    total_i;   // A  Matek CAN-L4-BM current
};
// Format: "QHfff"   Body: 8+2+3×4 = 22 B   Record: 25 B

struct __attribute__((packed)) LogMsgVNED {
    uint64_t time_us;
    uint16_t rate_hz;
    float    xdot_world;   // m/s  world/NED-frame forward velocity (derived, not a Kalman state)
    float    zdot_world;   // m/s  world/NED-frame vertical velocity
};
// Format: "QHff"   Body: 8+2+2×4 = 18 B   Record: 21 B

/* ── Log descriptor table ────────────────────────────────────────────────── */

struct LogDef {
    uint8_t     msg_id;
    const char *name;    // ≤4 chars, null-padded to 4 by strncpy (e.g. "ATT" → stored as "ATT\0")
    const char *fmt;     // ArduPilot format codes (≤16 chars): Q H f i h B …
    const char *labels;  // comma-separated field names (≤64 chars); TimeUS must be first
    size_t      body_size;
};

constexpr LogDef kLogDefs[] = {
    { LOG_MSG_ATT,
      "ATT",
      "QHfffffffff",
      "TimeUS,Rate,Roll,Pitch,Yaw,P,Q,R,Pdot,Qdot,Rdot",
      sizeof(LogMsgATT) },

    { LOG_MSG_LIN,
      "LIN",
      "QHfffffffff",
      "TimeUS,Rate,X,Y,Z,U,V,W,Udot,Vdot,Wdot",
      sizeof(LogMsgLIN) },

    { LOG_MSG_RCIN,
      "RCIN",
      "QHffffBB",
      "TimeUS,Rate,YawStk,VelStk,HeightStk,LeanStk,Armed,Mode",
      sizeof(LogMsgRCIN) },

    { LOG_MSG_OUTP,
      "OUTP",
      "QHffff",
      "TimeUS,Rate,Tq0,Tq1,Tq2,Tq3",
      sizeof(LogMsgOUTP) },

    { LOG_MSG_HIPS,
      "HIPS",
      "QHffffffff",
      "TimeUS,Rate,Pos0,Pos1,Pos2,Pos3,Volt0,Volt1,Volt2,Volt3",
      sizeof(LogMsgHIPS) },

    { LOG_MSG_HERR,
      "HERR",
      "QHBBBB",
      "TimeUS,Rate,Err0,Err1,Err2,Err3",
      sizeof(LogMsgHERR) },

    { LOG_MSG_LEGS,
      "LEGS",
      "QHffffffff",
      "TimeUS,Rate,L0,L0dot,ThL0,ThL0dot,L1,L1dot,ThL1,ThL1dot",
      sizeof(LogMsgLEGS) },

    { LOG_MSG_LEGV,
      "LEGV",
      "QHffff",
      "TimeUS,Rate,Leg0Xdot,Leg0Zdot,Leg1Xdot,Leg1Zdot",
      sizeof(LogMsgLEGV) },

    { LOG_MSG_MOTV,
      "MOTV",
      "QHffffff",
      "TimeUS,Rate,HipVel0,HipVel1,HipVel2,HipVel3,WheelVel0,WheelVel1",
      sizeof(LogMsgMOTV) },

    { LOG_MSG_PWR,
      "PWR",
      "QHfff",
      "TimeUS,Rate,BattV,PmonV,TotalI",
      sizeof(LogMsgPWR) },

    { LOG_MSG_VNED,
      "VNED",
      "QHff",
      "TimeUS,Rate,XdotWorld,ZdotWorld",
      sizeof(LogMsgVNED) },
};

constexpr size_t kNumLogDefs = sizeof(kLogDefs) / sizeof(kLogDefs[0]);
