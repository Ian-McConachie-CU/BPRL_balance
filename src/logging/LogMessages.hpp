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
    float    height_stk;  // [-1, 1]  height-set switch (ch2) -- placeholder, unconsumed
    float    lean_stk;    // [-1, 1]  leanover switch (ch3) -- placeholder, unconsumed
    uint8_t  armed;       // 0=disarmed, 1=armed
};
// Format: "QHffffB"   Body: 8+2+4×4+1 = 27 B   Record: 30 B

struct __attribute__((packed)) LogMsgOUTP {
    uint64_t time_us;
    uint16_t rate_hz;
    float    tq0;    // Nm  hip FL torque command
    float    tq1;    // Nm  hip FR torque command
    float    tq2;    // Nm  hip RL torque command
    float    tq3;    // Nm  hip RR torque command
};
// Format: "QHffff"   Body: 8+2+4×4 = 26 B   Record: 29 B

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
      "QHffffB",
      "TimeUS,Rate,YawStk,VelStk,HeightStk,LeanStk,Armed",
      sizeof(LogMsgRCIN) },

    { LOG_MSG_OUTP,
      "OUTP",
      "QHffff",
      "TimeUS,Rate,Tq0,Tq1,Tq2,Tq3",
      sizeof(LogMsgOUTP) },
};

constexpr size_t kNumLogDefs = sizeof(kLogDefs) / sizeof(kLogDefs[0]);
