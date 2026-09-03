#include "src/coms/CANMotor.hpp"
#include "src/coms/CAN.hpp"
#include <cstring>
#include <cmath>

/* ── Global state table ─────────────────────────────────────────────────── */

MUTEX_DECL(motor_state_mtx);
CanMotorState g_motors[CAN_MOTOR_MAX] = {};

struct MotorEntry {
    uint8_t       id;         // firmware slot — what every other function looks motors up by
    CanMotorProto proto;
    bool          registered;
    uint8_t       node_id;    // ODRIVE only: actual wire-level ODrive node id, may differ from
                               // `id` (this project's own 1-6 slot numbering) — see
                               // can_motor_register()'s node_id param.
    float         sign;       // ODRIVE only: +1.0 or -1.0, see can_motor_register()'s sign param.
};
static MotorEntry s_entries[CAN_MOTOR_MAX] = {};
static int        s_count = 0;

static int find_entry(uint8_t id)
{
    for (int i = 0; i < s_count; i++)
        if (s_entries[i].id == id) return i;
    return -1;
}

/* ════════════════════════════════════════════════════════════════════════════
 * LK-TECH MG8016E-i6 / DG80R-C7 drive — CONFIRMED against the vendor's own
 * "CAN PROTOCOL V2.35" document (not the MyActuator reference this used to
 * be guessed from):
 *   Command:  SID = 0x140 + motor_id,  8 bytes
 *   Response: SID = 0x140 + motor_id,  8 bytes — SAME identifier as the
 *             command, within 0.25ms. This driver previously listened on
 *             0x240+id (the MyActuator/RMD-X convention) and would never
 *             have received a real reply regardless of wiring.
 *
 * Torque command (0xA1) — CONFIRMED:
 *   data[0] = 0xA1, data[1..3] = 0x00
 *   data[4..5] = int16 iqControl (LSB first), range -2048..2048,
 *                corresponding to -33..33 A for the MG series specifically.
 *   data[6..7] = 0x00
 *   Response: byte1=temp(int8,1C/LSB), byte2-3=iq(int16,same units as
 *   command), byte4-5=speed(int16,1 dps/LSB), byte6-7=encoder position
 *   (uint16 — top 16 bits of the MG series' 18-bit encoder, 0..65535 over
 *   a full 0..360 deg revolution).
 *
 * Velocity command (0xA2) — CONFIRMED:
 *   data[0] = 0xA2, data[1..3] = 0x00
 *   data[4..7] = int32 speed (LSB first), 0.01 dps/LSB
 * ════════════════════════════════════════════════════════════════════════════ */

// Nm<->ratio scale for can_motor_set_torque()'s Nm convenience API. CAN
// PROTOCOL V2.35 confirms the raw command is iqControl, int16_t range
// -2048..2048 corresponding to -33..33 A for the MG series (33/2048 A per
// LSB), but gives no Nm mapping (the motor's Kt isn't documented). This
// default (~83.3 ratio/Nm) is a ROUGH placeholder derived only from the
// ~12 Nm continuous rating used for sizing in wheeled_biped_project_notes.md
// — it is NOT a measured calibration. Prefer can_motor_set_torque_raw()
// until this has been calibrated against a real load or torque wrench.
static constexpr float   RMD_TORQUE_SCALE_DEFAULT = 83.3f;   // ratio/Nm, placeholder
static constexpr int16_t RMD_TORQUE_RATIO_MAX     = 2048;    // confirmed by CAN PROTOCOL V2.35
static constexpr float   RMD_SPEED_DPS_MAX        = 24000.0f; // confirmed by the MGv2 manual
static float s_rmd_torque_scale = RMD_TORQUE_SCALE_DEFAULT;

void  can_motor_set_rmd_torque_scale(float ratio_per_Nm) { s_rmd_torque_scale = ratio_per_Nm; }
float can_motor_get_rmd_torque_scale(void)                { return s_rmd_torque_scale; }

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

/* ════════════════════════════════════════════════════════════════════════════
 * Hip zero-offset + sign + safety bounds — the FINAL gate every hip (RMD)
 * torque/velocity/position command passes through, regardless of caller (the
 * normal ControlThread path, a debug/test tool over USB, anything future).
 * Wired in at can_motor_set_torque()/set_torque_raw()/set_velocity()/
 * set_position() below, and at rmd_rx_cb()'s decode (so g_motors[].pos_rad
 * is ALWAYS already robot-frame, offset+sign baked in, for every reader —
 * StateManager's leg FK, HipLock, USB MOTOR,status, everything).
 *
 * HARDCODED BY DESIGN (not runtime-settable, not flash-persisted): edit,
 * recompile, reflash. Calibration workflow: mount the hip in whatever
 * orientation, read its raw (offset=0) angle via MOTOR,status, physically
 * move it to your desired zero pose, read the raw angle there, set
 * HIP_OFFSET_RAD[id-1] to that value, recompile+reflash — MOTOR,status
 * will then read ~0 at that pose.
 *
 * robot_frame_angle = HIP_SIGN[id-1] * (raw_encoder_rad - HIP_OFFSET_RAD[id-1])
 *
 * HIP_SIGN: +1 or -1 per hip. Left/right hip pairs are mechanically mirror
 * images of each other (same as the wheel L/R sign in can_motor_register()),
 * so a raw encoder increasing in "the same physical direction" reads as
 * INCREASING robot-frame angle on one side but DECREASING on the other
 * unless corrected here. Set so that positive robot-frame angle means the
 * same physical motion (e.g. "hip flexes forward") on every hip regardless
 * of which side it's mounted on — this is what lets ONE HIP_ANGLE_MIN/MAX_RAD
 * pair below apply uniformly to all 4 instead of needing a mirrored pair per
 * side. Applied at decode (rmd_rx_cb, both pos_rad AND vel_rads/torque_Nm --
 * a mirrored hip's reported velocity/torque sign must flip too, same as
 * position) and at every command send (torque/velocity/position) — same
 * "gate once at the source" pattern as the offset itself.
 *
 * NOTE: raw_encoder_rad (see rmd_rx_cb's 0xA1/0xA2 case) is a wrapping
 * 0..2*pi single-turn value, never negative before the offset is applied.
 * HIP_ANGLE_MIN/MAX_RAD below assume the offset is chosen so the hip's
 * actual mechanical range of motion (expected well under 360 deg) doesn't
 * straddle the 0/2*pi wrap point after subtraction — true for any offset
 * placed near the middle of the real range, which is the natural choice
 * anyway (see the "move to your desired zero pose" step above). This holds
 * regardless of HIP_SIGN, since negating (raw-offset) doesn't change WHERE
 * the wrap point falls relative to the reachable window, only which end of
 * the window it would be nearest to.
 * ════════════════════════════════════════════════════════════════════════════ */

// hip1=FL, hip2=FR, hip3=RL, hip4=RR (see main.cpp's registration comments).
// FL/RL (left side) inverted relative to FR/RR (right side) -- mirrored
// mounting, same concept as wheel L's sign in can_motor_register().
static constexpr float HIP_SIGN[4] = { -1.0f, 1.0f, -1.0f, 1.0f };

// Calibrated 2026-09-02 -- per-hip, NOT the earlier uniform pi-radians bench
// placeholder (each hip's real raw-at-zero-pose reading differs, hence 4
// distinct values here rather than 1 repeated). Per the calibration workflow
// above: raw angle read via MOTOR,status (offset=0) at each hip's desired
// zero pose. Order is [hip1, hip2, hip3, hip4] = [FL, FR, RL, RR].
static constexpr float HIP_OFFSET_RAD[4] = { 3.33f, 3.22f, 3.18f, 3.74f };

// Robot-frame (post-offset, post-sign) hard stops + soft margin -- same
// window for all 4 hips since HIP_SIGN normalizes every hip to one shared
// convention (see that comment above). -1.45 rad =~ -83.1 deg,
// 0.44 rad =~ +25.2 deg (~108 deg total span). MEASURE against the real
// linkage before trusting this on hardware, same as every other placeholder
// constant in this codebase.
static constexpr float HIP_ANGLE_MIN_RAD[4]      = { -1.45f, -1.45f, -1.45f, -1.45f };  // =~ -83.1 deg
static constexpr float HIP_ANGLE_MAX_RAD[4]      = {  0.44f,  0.44f,  0.44f,  0.44f };  // =~ +25.2 deg
static constexpr float HIP_ANGLE_SOFT_MARGIN_RAD = 0.175f;   // ~10 deg before the stop
static constexpr float HIP_VEL_LIMIT_RADS        = 20.0f;    // ~190 rpm
static constexpr float HIP_VEL_SOFT_MARGIN_RADS  = 4.0f;
static constexpr float HIP_TORQUE_LIMIT_NM       = 12.0f;    // MG8016E-i6 continuous

static inline float hip_to_robot_frame(uint8_t id, float raw_rad)
{
    return HIP_SIGN[id - 1] * (raw_rad - HIP_OFFSET_RAD[id - 1]);
}
static inline float hip_to_raw(uint8_t id, float robot_frame_rad)
{
    return HIP_OFFSET_RAD[id - 1] + HIP_SIGN[id - 1] * robot_frame_rad;
}

// Direction-aware soft ramp: scale in [0,1] applied to whatever `signed_qty`
// is about to be sent. 1.0 = unrestricted; ramps linearly to 0.0 exactly at
// whichever bound `signed_qty`'s sign is pushing toward; the opposite
// direction is never restricted. Same formula as the codebase's other
// soft-limit ramps (was ActuatorSafety::limit_scale() — hip angle/velocity
// limiting now lives here instead, see ActuatorSafety.hpp's updated header).
static float hip_limit_scale(float value, float lo, float hi, float margin, float signed_qty)
{
    if (signed_qty > 0.0f) return clampf((hi - value) / margin, 0.0f, 1.0f);
    if (signed_qty < 0.0f) return clampf((value - lo) / margin, 0.0f, 1.0f);
    return 1.0f;
}

// Clamps a torque command (Nm or raw ratio — caller's units, caller hard-
// clamps to its own range afterward) given the hip's current robot-frame
// position and velocity. Used by both can_motor_set_torque() (Nm) and
// can_motor_set_torque_raw() (raw ratio) — the soft-ramp math is unitless
// (a [0,1] scale), so it applies to either.
static float hip_soft_scale(uint8_t id, float pos_rad, float vel_rads, float signed_qty)
{
    const int i = id - 1;
    const float angle_scale = hip_limit_scale(pos_rad, HIP_ANGLE_MIN_RAD[i], HIP_ANGLE_MAX_RAD[i],
                                               HIP_ANGLE_SOFT_MARGIN_RAD, signed_qty);
    const float vel_scale   = hip_limit_scale(vel_rads, -HIP_VEL_LIMIT_RADS, HIP_VEL_LIMIT_RADS,
                                               HIP_VEL_SOFT_MARGIN_RADS, signed_qty);
    return angle_scale * vel_scale;
}

// Velocity-mode commands: ramp the commanded velocity itself toward zero as
// the joint nears its angle bound (direction-aware — only the direction
// pushing toward the bound is restricted), then hard-clamp to the rating.
static float hip_clamp_velocity(uint8_t id, float vel_rads, float pos_rad)
{
    const int i = id - 1;
    const float scale = hip_limit_scale(pos_rad, HIP_ANGLE_MIN_RAD[i], HIP_ANGLE_MAX_RAD[i],
                                         HIP_ANGLE_SOFT_MARGIN_RAD, vel_rads);
    return clampf(vel_rads * scale, -HIP_VEL_LIMIT_RADS, HIP_VEL_LIMIT_RADS);
}

// Position-mode targets: hard-clamp the robot-frame target into bounds
// before it's ever converted to a raw setpoint and sent.
static float hip_clamp_position_target(uint8_t id, float target_rad)
{
    const int i = id - 1;
    return clampf(target_rad, HIP_ANGLE_MIN_RAD[i], HIP_ANGLE_MAX_RAD[i]);
}

// g_motors[]'s current state for hip id -- helper for the can_motor_set_*()
// gates below, all of which need "where is this hip right now" to compute
// the soft-ramp scale. Returns false if feedback isn't valid yet; callers
// must refuse to send anything in that case rather than guess a scale from
// stale/zero data -- same fail-safe philosophy as ActuatorSafety's existing
// "no verified feedback -> command nothing" rule.
static bool hip_current_state(int entry_idx, float &pos_rad, float &vel_rads)
{
    chMtxLock(&motor_state_mtx);
    bool valid = g_motors[entry_idx].valid;
    pos_rad    = g_motors[entry_idx].pos_rad;
    vel_rads   = g_motors[entry_idx].vel_rads;
    chMtxUnlock(&motor_state_mtx);
    return valid;
}

// Records the outcome of a can_send()/can_send_rtr() this entry's command
// just made -- see CanMotorState's tx_ok/tx_fail/rx_count comment. Despite
// the counters living on every CanMotorState (not just hips), this was
// only ever wired up for the RMD path until 2026-09-02 -- ODrive (wheel)
// commands went through odrive_send_torque()/odrive_send_velocity()
// without ever calling this, so tx_ok/tx_fail always read 0/0 for wheels
// regardless of what was actually happening on the bus. Renamed from
// hip_record_tx to motor_record_tx accordingly; behavior unchanged.
static void motor_record_tx(int entry_idx, bool ok)
{
    chMtxLock(&motor_state_mtx);
    if (ok) g_motors[entry_idx].tx_ok++; else g_motors[entry_idx].tx_fail++;
    chMtxUnlock(&motor_state_mtx);
}

static bool rmd_send_torque_raw(uint8_t id, int16_t ratio)
{
    if (ratio > RMD_TORQUE_RATIO_MAX) ratio = RMD_TORQUE_RATIO_MAX;
    if (ratio < -RMD_TORQUE_RATIO_MAX) ratio = -RMD_TORQUE_RATIO_MAX;
    uint8_t data[8] = {};
    data[0] = 0xA1;
    data[4] = (uint8_t)((uint16_t)ratio & 0xFFU);
    data[5] = (uint8_t)(((uint16_t)ratio >> 8) & 0xFFU);
    return can_send(CAN_BUS_1, 0x140U + id, data, 8);
}

static bool rmd_send_torque(uint8_t id, float torque_Nm)
{
    int16_t ratio = (int16_t)clampf(torque_Nm * s_rmd_torque_scale,
                                     -RMD_TORQUE_RATIO_MAX, RMD_TORQUE_RATIO_MAX);
    return rmd_send_torque_raw(id, ratio);
}

// Multi-motor broadcast torque command (0x280) -- CAN PROTOCOL V2.35,
// documented (previously unimplemented) in Motor_tool/CAN_config.md sec
// 1.4. ONE frame carries torque for hips 1-4 simultaneously --
// DATA[0:1]/[2:3]/[4:5]/[6:7] = int16 iqControl for motors 1-4
// respectively (same raw range as 0xA1, -2000..2000) -- replacing 4
// separate 0xA1 commands with 1. Replies still come back individually on
// each hip's own 0x140+id address (unchanged, rmd_rx_cb already handles
// this) -- broadcast only changes the OUTGOING command, not how replies
// are read. Requires all 4 drives configured for broadcast mode (their own
// GUI tool) and a matching baud rate (1 Mbit/s or 500 kbit/s only, per the
// vendor doc) -- switched to 500 kbit/s alongside this, see CAN.cpp.
static bool rmd_send_torque_broadcast(const int16_t ratio[4])
{
    uint8_t data[8];
    for (int i = 0; i < 4; i++) {
        data[i * 2]     = (uint8_t)((uint16_t)ratio[i] & 0xFFU);
        data[i * 2 + 1] = (uint8_t)(((uint16_t)ratio[i] >> 8) & 0xFFU);
    }
    return can_send(CAN_BUS_1, 0x280U, data, 8);
}

static bool rmd_send_velocity(uint8_t id, float vel_rads)
{
    // Scale: vel (rad/s) → dps → LSB (0.01 dps/LSB), clamped to the
    // confirmed +/-24000 dps range.
    float dps = clampf(vel_rads * (180.0f / 3.14159265f), -RMD_SPEED_DPS_MAX, RMD_SPEED_DPS_MAX);
    int32_t spd = (int32_t)(dps * 100.0f);
    uint8_t data[8] = {};
    data[0] = 0xA2;
    data[4] = (uint8_t)((uint32_t)spd & 0xFFU);
    data[5] = (uint8_t)(((uint32_t)spd >> 8)  & 0xFFU);
    data[6] = (uint8_t)(((uint32_t)spd >> 16) & 0xFFU);
    data[7] = (uint8_t)(((uint32_t)spd >> 24) & 0xFFU);
    return can_send(CAN_BUS_1, 0x140U + id, data, 8);
}

// Single Loop Angle Control 2 (0xA6) -- ported from Motor_tool/src/RmdMotor.cpp's
// confirmed implementation. data[1]=spinDirection (0x00=CW,0x01=CCW -- which
// way it turns to REACH the target, not derived automatically); data[2..3]=
// max speed (uint16, 1 dps/LSB); data[4..7]=target angle (uint32, 0.01 deg/LSB),
// ABSOLUTE but SINGLE-TURN (0..359.99 deg, wraps every revolution) -- the
// same reference frame rmd_rx_cb's 0xA1/0xA2 raw_pos decode already uses,
// unlike the 0xA4 multi-turn variant (not used here for exactly that reason:
// pos_rad is a wrapping single-turn value, so only 0xA6 stays in the same
// frame). angle_rad here is the RAW (pre-offset) target -- callers convert
// through hip_to_raw() first, see can_motor_set_position() below.
static void rmd_send_position_single_turn(uint8_t id, float angle_rad, float maxspeed_rads, bool cw)
{
    angle_rad = fmodf(angle_rad, 2.0f * 3.14159265f);
    if (angle_rad < 0.0f) angle_rad += 2.0f * 3.14159265f;
    uint16_t maxspeed_dps = (uint16_t)(maxspeed_rads * (180.0f / 3.14159265f));
    uint32_t angle_lsb    = (uint32_t)(angle_rad * (180.0f / 3.14159265f) * 100.0f);
    uint8_t data[8] = {};
    data[0] = 0xA6;
    data[1] = cw ? 0x00U : 0x01U;
    data[2] = (uint8_t)(maxspeed_dps & 0xFFU);
    data[3] = (uint8_t)((maxspeed_dps >> 8) & 0xFFU);
    data[4] = (uint8_t)(angle_lsb & 0xFFU);
    data[5] = (uint8_t)((angle_lsb >> 8)  & 0xFFU);
    data[6] = (uint8_t)((angle_lsb >> 16) & 0xFFU);
    data[7] = (uint8_t)((angle_lsb >> 24) & 0xFFU);
    can_send(CAN_BUS_1, 0x140U + id, data, 8);
}

// 0x9A ReadState1 — the only way to get voltage/error off an RMD hip;
// unlike temp/pos/vel/torque, nothing else requests this automatically (see
// can_motor_poll_status_round_robin(), called from ControlThread).
static void rmd_send_status_request(uint8_t id)
{
    uint8_t data[8] = {};
    data[0] = 0x9A;
    can_send(CAN_BUS_1, 0x140U + id, data, 8);
}

static void rmd_rx_cb(const CANRxFrame &f, void *ctx)
{
    int idx = (int)(uintptr_t)ctx;
    if (f.DLC < 8) return;

    // Counts every reply on this SID regardless of command byte -- unlike
    // `valid`, which only latches on a decodable one -- so it's a direct
    // measure of "how often does this specific hip actually reply", for
    // comparing against tx_ok/tx_fail (see CanMotorState's comment).
    chMtxLock(&motor_state_mtx);
    g_motors[idx].rx_count++;
    chMtxUnlock(&motor_state_mtx);

    // Every RMD reply lands on the same SID (0x140+id) regardless of which
    // command it's answering, so data[0] (the command echo) has to gate the
    // decode — 0x9A's layout is unrelated to 0xA1/0xA2's and decoding it
    // with the wrong formula would silently corrupt torque/vel/pos with
    // voltage/reserved bytes reinterpreted as iq/speed/encoder.
    switch (f.data8[0]) {
    case 0xA1: case 0xA2: {
        // Confirmed by CAN PROTOCOL V2.35: byte1=temp, byte2-3=iq (int16,
        // same +/-2048 units as the command), byte4-5=speed (int16, 1
        // dps/LSB), byte6-7=encoder position (uint16, top 16 bits of the MG
        // series' 18-bit encoder -> 0..65535 over 0..360 deg).
        int16_t  raw_iq    = (int16_t)((uint16_t)f.data8[2] | ((uint16_t)f.data8[3] << 8));
        int16_t  raw_speed = (int16_t)((uint16_t)f.data8[4] | ((uint16_t)f.data8[5] << 8));
        uint16_t raw_enc   = (uint16_t)f.data8[6] | ((uint16_t)f.data8[7] << 8);
        uint8_t  temp      = f.data8[1];
        float    raw_pos   = (float)raw_enc * (3.14159265f / 32768.0f);
        uint8_t  hip_id    = s_entries[idx].id;   // == the CAN id, since RMD's SID is 0x140+id

        // HIP_SIGN flips torque/vel/pos together -- a mirrored hip's
        // reported velocity and torque direction must match its now-mirrored
        // position convention, same as the offset applies to position alone.
        const float sign = HIP_SIGN[hip_id - 1];

        chMtxLock(&motor_state_mtx);
        g_motors[idx].torque_Nm = s_rmd_torque_scale > 0.0f
                                      ? sign * (float)raw_iq / s_rmd_torque_scale : 0.0f;
        g_motors[idx].vel_rads  = sign * (float)raw_speed * (3.14159265f / 180.0f);
        // Offset+sign-corrected to robot frame -- see the hip safety section
        // above. Every reader of pos_rad (StateManager's leg FK, HipLock,
        // MOTOR,status, ...) sees this already-corrected value, never raw.
        g_motors[idx].pos_rad   = hip_to_robot_frame(hip_id, raw_pos);
        g_motors[idx].temp_C    = (float)temp;
        g_motors[idx].valid     = true;
        chMtxUnlock(&motor_state_mtx);
        break;
    }
    case 0x9A: {
        // byte1=temp (int8, 1C/LSB), byte7=errorState — match CAN PROTOCOL
        // V2.35 and real hardware. Voltage does NOT match the vendor doc:
        // it claims byte3-4 at 0.1V/LSB, but real MG8016E-i6 replies put it
        // at byte2-3, 0.01V/LSB (confirmed against a known 41V bench supply
        // during Motor_tool bring-up — see Motor_tool/CAN_config.md §1.3 and
        // Motor_tool/src/RmdMotor.cpp's matching case). Trust the hardware.
        int8_t   temp = (int8_t)f.data8[1];
        uint16_t volt = (uint16_t)f.data8[2] | ((uint16_t)f.data8[3] << 8);

        chMtxLock(&motor_state_mtx);
        g_motors[idx].temp_C      = (float)temp;
        g_motors[idx].voltage_V   = (float)volt * 0.01f;
        g_motors[idx].error_flags = f.data8[7];
        g_motors[idx].valid       = true;
        chMtxUnlock(&motor_state_mtx);
        break;
    }
    default:
        break;
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * ODrive CAN Simple protocol — for a GIM6010-8 running ODrive firmware on a
 * GDS68 driver. Arbitration ID = (node_id << 5) | cmd_id (standard 11-bit).
 *
 * Set_Axis_State (0x07), Set_Controller_Mode (0x0B), Set_Input_Vel (0x0D) —
 * CONFIRMED against real hardware in final-project-Ian-McConachie-CU (an
 * ECEN5813 balancing-robot project using this same GIM6010-8 + ODrive
 * combination, just a different CAN transceiver). Get_Encoder_Estimates
 * (0x09), Set_Input_Torque (0x0E) and Heartbeat (0x01) are the standard
 * ODrive CAN Simple numbering for the same firmware family, but that project
 * never exercised them (open-loop velocity commands only, no feedback read)
 * — treat as unconfirmed until checked against real traffic.
 * ════════════════════════════════════════════════════════════════════════════ */

static constexpr uint8_t ODRIVE_CMD_HEARTBEAT           = 0x01;
static constexpr uint8_t ODRIVE_CMD_SET_AXIS_STATE      = 0x07;
static constexpr uint8_t ODRIVE_CMD_GET_ENCODER_EST     = 0x09;
static constexpr uint8_t ODRIVE_CMD_SET_CONTROLLER_MODE = 0x0B;
static constexpr uint8_t ODRIVE_CMD_SET_INPUT_VEL       = 0x0D;
static constexpr uint8_t ODRIVE_CMD_SET_INPUT_TORQUE    = 0x0E;

static constexpr uint32_t ODRIVE_AXIS_STATE_IDLE        = 1;
static constexpr uint32_t ODRIVE_AXIS_STATE_CLOSED_LOOP = 8;
static constexpr uint32_t ODRIVE_CONTROL_MODE_TORQUE    = 1;
static constexpr uint32_t ODRIVE_CONTROL_MODE_VELOCITY  = 2;
static constexpr uint32_t ODRIVE_INPUT_MODE_PASSTHROUGH = 1;

// Same rating ActuatorSafety::WHEEL_TORQUE_LIMIT_NM already clamps to before
// this is ever called — this is a second, independent floor, same pattern as
// RMD_TORQUE_RATIO_MAX / gim_set_torque_limit() for the other two drivers.
static constexpr float ODRIVE_TORQUE_LIMIT_NM = 8.0f;   // GIM6010-6 stall

// GIM6010-8's internal reduction ratio — matches GEAR_RATIO in
// final-project-Ian-McConachie-CU's motor_control.h (the "-8" in the model
// name). Implies the ODrive's encoder/command reference frame there was the
// MOTOR (rotor) side, not the wheel's output shaft — needed to convert
// between this codebase's wheel-referenced rad/s and Nm (ActuatorSafety,
// BalanceController) and whatever the ODrive axis actually expects.
static constexpr float ODRIVE_GEAR_RATIO = 8.0f;

static inline uint32_t odrive_arb_id(uint8_t node_id, uint8_t cmd_id)
{
    return ((uint32_t)node_id << 5) | cmd_id;
}

static void odrive_set_axis_state(uint8_t id, uint32_t state)
{
    uint8_t data[8] = {};
    memcpy(&data[0], &state, 4);
    can_send(CAN_BUS_1, odrive_arb_id(id, ODRIVE_CMD_SET_AXIS_STATE), data, 8);
}

static void odrive_set_controller_mode(uint8_t id, uint32_t control_mode, uint32_t input_mode)
{
    uint8_t data[8];
    memcpy(&data[0], &control_mode, 4);
    memcpy(&data[4], &input_mode, 4);
    can_send(CAN_BUS_1, odrive_arb_id(id, ODRIVE_CMD_SET_CONTROLLER_MODE), data, 8);
}

// Puts the axis into torque control (this codebase drives every registered
// motor uniformly via can_motor_set_torque() from ControlThread — see
// threads.cpp — unlike final-project-Ian-McConachie-CU, which only ever used
// velocity control from its own PID loop). Called once per motor from
// can_motor_register() below, mirroring that project's Motor_velo_ctr_init()
// timing (short delays between commands so the drive has time to process
// each one) but for CONTROL_MODE_TORQUE instead of CONTROL_MODE_VELOCITY.
static void odrive_init_axis(uint8_t id)
{
    odrive_set_controller_mode(id, ODRIVE_CONTROL_MODE_TORQUE, ODRIVE_INPUT_MODE_PASSTHROUGH);
    chThdSleepMilliseconds(50);
    odrive_set_axis_state(id, ODRIVE_AXIS_STATE_CLOSED_LOOP);
    chThdSleepMilliseconds(50);
}

// NOT from the old project (it never sent torque commands — see section
// header). torque_Nm is wheel/output-referenced, matching ActuatorSafety's
// WHEEL_TORQUE_LIMIT_NM convention; dividing by the gear ratio converts to
// the smaller motor-side torque a reducer needs for the same output torque
// (the inverse of the velocity relationship below). If your ODrive axis
// config already reports/expects output-shaft-referenced units (e.g. it has
// its own gear ratio baked into config), remove this division.
static bool odrive_send_torque(uint8_t id, float torque_Nm)
{
    torque_Nm = clampf(torque_Nm, -ODRIVE_TORQUE_LIMIT_NM, ODRIVE_TORQUE_LIMIT_NM);
    float motor_torque_Nm = torque_Nm / ODRIVE_GEAR_RATIO;
    uint8_t data[8] = {};
    memcpy(&data[0], &motor_torque_Nm, 4);   // IEEE float, LSB byte order (matches STM32 native)
    return can_send(CAN_BUS_1, odrive_arb_id(id, ODRIVE_CMD_SET_INPUT_TORQUE), data, 8);
}

// Reproduces final-project-Ian-McConachie-CU's Motor_set_velocity() exactly:
// multiplies by GEAR_RATIO and sends that value directly as ODrive's
// Input_Vel float. NOTE this is NOT converted rad/s -> turns/s (no /2pi
// anywhere in the original either, despite ODrive's field being named
// "turns/s") — reproduced verbatim rather than "corrected", since the
// verbatim version is what was actually validated on this motor. Only takes
// effect if the axis's controller mode is actually VELOCITY —
// odrive_init_axis() sets TORQUE, so this is here for API completeness
// (can_motor_set_velocity is part of the public interface) rather than
// something this project's control loop currently calls for wheel motors.
static bool odrive_send_velocity(uint8_t id, float vel_rads)
{
    float motor_cmd = vel_rads * ODRIVE_GEAR_RATIO;
    float torque_ff = 0.0f;
    uint8_t data[8];
    memcpy(&data[0], &motor_cmd, 4);
    memcpy(&data[4], &torque_ff, 4);
    return can_send(CAN_BUS_1, odrive_arb_id(id, ODRIVE_CMD_SET_INPUT_VEL), data, 8);
}

static void odrive_rx_cb(const CANRxFrame &f, void *ctx)
{
    int idx = (int)(uintptr_t)ctx;
    if (f.DLC < 8) return;

    uint8_t cmd = (uint8_t)(f.std.SID & 0x1FU);

    // Counts every frame on this wheel's arbitration IDs regardless of
    // which command it is (Heartbeat or Get_Encoder_Estimates) -- same
    // "how often does this motor actually reply" measure rmd_rx_cb's
    // rx_count is, now wired up for ODrive too (previously only RMD did
    // this, so wheels always showed rx=0 in MOTOR,status even when
    // healthy).
    chMtxLock(&motor_state_mtx);
    g_motors[idx].rx_count++;
    if (cmd == ODRIVE_CMD_GET_ENCODER_EST) {
        // Get_Encoder_Estimates: bytes0-3 = float Pos_Estimate (turns),
        // bytes4-7 = float Vel_Estimate (turns/s). UNCONFIRMED cmd id — see
        // section header. Also NOT from the old project (no precedent —
        // it never read feedback): converts motor-side turns to
        // wheel/output-referenced rad, dividing by the gear ratio, unlike
        // odrive_send_velocity()'s verbatim-copied (ungeared) TX path above
        // — so a command sent via can_motor_set_velocity() and the resulting
        // pos_rad/vel_rads read back here are NOT in matching units. Only
        // relevant if the axis is ever put into VELOCITY mode; currently
        // dormant since odrive_init_axis() uses TORQUE.
        float pos_turns, vel_turns_s;
        memcpy(&pos_turns, &f.data8[0], 4);
        memcpy(&vel_turns_s, &f.data8[4], 4);
        // sign flips a mechanically-mirrored wheel's feedback to match this
        // project's convention -- see can_motor_register()'s sign param.
        const float sign = s_entries[idx].sign;
        g_motors[idx].pos_rad  = sign * pos_turns * (2.0f * 3.14159265f) / ODRIVE_GEAR_RATIO;
        g_motors[idx].vel_rads = sign * vel_turns_s * (2.0f * 3.14159265f) / ODRIVE_GEAR_RATIO;
        g_motors[idx].valid    = true;
    } else if (cmd == ODRIVE_CMD_HEARTBEAT) {
        // Heartbeat carries axis_error/axis_state, not any of CanMotorState's
        // fields — used here purely as a liveness signal (this motor is
        // actually on the bus and talking).
        g_motors[idx].valid = true;
    }
    chMtxUnlock(&motor_state_mtx);
}

/* ════════════════════════════════════════════════════════════════════════════
 * SDC102 (Steadywin GDS6) protocol — stub
 *
 * Fill actual frame formats from the GIM6010-8 instruction manual.
 * CAN IDs and command bytes are placeholders pending datasheet review.
 * ════════════════════════════════════════════════════════════════════════════ */

static void sdc102_send_torque(uint8_t id, float torque_Nm)
{
    // TODO: replace with actual SDC102 torque command format
    (void)id; (void)torque_Nm;
}

static void sdc102_send_velocity(uint8_t id, float vel_rads)
{
    // TODO: replace with actual SDC102 velocity command format
    (void)id; (void)vel_rads;
}

static void sdc102_rx_cb(const CANRxFrame &f, void *ctx)
{
    // TODO: parse SDC102 feedback frame
    (void)f; (void)ctx;
}

/* ── Registration ───────────────────────────────────────────────────────── */

void can_motor_register(uint8_t id, CanMotorProto proto, uint8_t node_id, float sign)
{
    if (s_count >= CAN_MOTOR_MAX) return;

    // node_id=0 (default) means "same as id" — preserves old behavior for
    // RMD/SDC102, and for ODRIVE whenever the physical drive's node id
    // happens to match this project's slot numbering.
    uint8_t wire_id = (node_id != 0) ? node_id : id;

    int idx = s_count++;
    s_entries[idx] = {id, proto, true, wire_id, sign};

    switch (proto) {
    case CAN_MOTOR_RMD:
        // Reply arrives on the SAME identifier as the command (confirmed by
        // CAN PROTOCOL V2.35) — was previously 0x240+id (MyActuator/RMD-X
        // convention), which this drive never actually replies on.
        bprl_can_register(CAN_BUS_1, 0x140U + id, rmd_rx_cb, (void *)(uintptr_t)idx);
        break;
    case CAN_MOTOR_SDC102:
        // TODO: register correct SDC102 response ID when protocol is confirmed
        bprl_can_register(CAN_BUS_1, 0x300U + id, sdc102_rx_cb, (void *)(uintptr_t)idx);
        break;
    case CAN_MOTOR_ODRIVE:
        bprl_can_register(CAN_BUS_1, odrive_arb_id(wire_id, ODRIVE_CMD_HEARTBEAT),
                           odrive_rx_cb, (void *)(uintptr_t)idx);
        bprl_can_register(CAN_BUS_1, odrive_arb_id(wire_id, ODRIVE_CMD_GET_ENCODER_EST),
                           odrive_rx_cb, (void *)(uintptr_t)idx);
        odrive_init_axis(wire_id);
        break;
    }
}

/* ── Command API ────────────────────────────────────────────────────────── */

bool can_motor_set_torque(uint8_t id, float torque_Nm)
{
    int i = find_entry(id);
    if (i < 0) return false;
    switch (s_entries[i].proto) {
    case CAN_MOTOR_RMD: {
        // No verified feedback yet -> send zero rather than refuse to send
        // at all. Refusing outright creates a bootstrap dependency on the
        // separate, much slower (~1 Hz/hip) 0x9A status round-robin to ever
        // establish `valid` -- sending zero instead means THIS command's
        // own 0xA1 reply is what bootstraps it, on the very next tick,
        // regardless of 0x9A. (Found 2026-09-02: hips 2-4 sat at "no data"
        // indefinitely with the refuse-to-send version, even though bus
        // scans showed them replying fine -- this is that bug's fix.)
        float pos_rad, vel_rads;
        float t = torque_Nm;
        if (hip_current_state(i, pos_rad, vel_rads)) {
            t = clampf(torque_Nm * hip_soft_scale(id, pos_rad, vel_rads, torque_Nm),
                       -HIP_TORQUE_LIMIT_NM, HIP_TORQUE_LIMIT_NM);
        } else {
            t = 0.0f;
        }
        // HIP_SIGN applied last, after all safety-gate math (which operates
        // entirely in robot-frame convention) -- only the final wire value
        // needs mirroring, same pattern as ODRIVE's sign below.
        motor_record_tx(i, rmd_send_torque(id, t * HIP_SIGN[id - 1]));
        break;
    }
    case CAN_MOTOR_SDC102: sdc102_send_torque(id, torque_Nm); break;
    case CAN_MOTOR_ODRIVE:
        motor_record_tx(i, odrive_send_torque(s_entries[i].node_id, torque_Nm * s_entries[i].sign));
        break;
    }
    return true;
}

// Sends torque for all 4 hips (ids 1-4) in ONE combined 0x280 broadcast
// frame instead of 4 separate 0xA1 commands -- see rmd_send_torque_broadcast()
// above. torques[i] corresponds to id i+1 (torques[0]=id1 ... torques[3]=id4),
// matching motor_torques[6]'s existing [0..3]=hip convention elsewhere
// (ActuatorSafety, ControlThread). Each hip's torque is independently
// safety-clamped exactly as can_motor_set_torque() would -- broadcast only
// changes how the already-clamped values reach the bus, not the gate
// itself. Requires ids 1-4 all registered as CAN_MOTOR_RMD; returns false
// (sends nothing) otherwise, same fail-clean convention as elsewhere here.
bool can_motor_set_hip_torques_broadcast(const float torques[4])
{
    int     entry_idx[4];
    int16_t ratio[4];

    for (int k = 0; k < 4; k++) {
        uint8_t id = (uint8_t)(k + 1);
        entry_idx[k] = find_entry(id);
        if (entry_idx[k] < 0 || s_entries[entry_idx[k]].proto != CAN_MOTOR_RMD) return false;

        float pos_rad, vel_rads;
        float t = torques[k];
        if (hip_current_state(entry_idx[k], pos_rad, vel_rads)) {
            t = clampf(t * hip_soft_scale(id, pos_rad, vel_rads, t),
                       -HIP_TORQUE_LIMIT_NM, HIP_TORQUE_LIMIT_NM);
        } else {
            t = 0.0f;   // no verified feedback yet -- same bootstrap-safe zero as can_motor_set_torque()
        }
        // HIP_SIGN applied last, same as can_motor_set_torque()'s comment.
        ratio[k] = (int16_t)clampf(t * s_rmd_torque_scale * HIP_SIGN[id - 1],
                                    -RMD_TORQUE_RATIO_MAX, RMD_TORQUE_RATIO_MAX);
    }

    bool ok = rmd_send_torque_broadcast(ratio);
    for (int k = 0; k < 4; k++) motor_record_tx(entry_idx[k], ok);   // one frame -- succeeds/fails as a unit
    return ok;
}

bool can_motor_set_torque_raw(uint8_t id, int16_t ratio)
{
    int i = find_entry(id);
    if (i < 0 || s_entries[i].proto != CAN_MOTOR_RMD) return false;
    float pos_rad, vel_rads;
    int16_t ratio_out = 0;   // no verified feedback yet -> zero, see can_motor_set_torque()'s comment
    if (hip_current_state(i, pos_rad, vel_rads)) {
        float ratio_f = clampf((float)ratio * hip_soft_scale(id, pos_rad, vel_rads, (float)ratio),
                                -(float)RMD_TORQUE_RATIO_MAX, (float)RMD_TORQUE_RATIO_MAX);
        ratio_out = (int16_t)ratio_f;
    }
    // HIP_SIGN applied last, same as can_motor_set_torque()'s comment.
    motor_record_tx(i, rmd_send_torque_raw(id, (int16_t)((float)ratio_out * HIP_SIGN[id - 1])));
    return true;
}

bool can_motor_set_velocity(uint8_t id, float vel_rads)
{
    int i = find_entry(id);
    if (i < 0) return false;
    switch (s_entries[i].proto) {
    case CAN_MOTOR_RMD: {
        // See can_motor_set_torque()'s comment -- zero, not refuse, when
        // feedback isn't verified yet.
        float pos_rad, cur_vel_rads;
        vel_rads = hip_current_state(i, pos_rad, cur_vel_rads)
                       ? hip_clamp_velocity(id, vel_rads, pos_rad) : 0.0f;
        // HIP_SIGN applied last, same as can_motor_set_torque()'s comment.
        motor_record_tx(i, rmd_send_velocity(id, vel_rads * HIP_SIGN[id - 1]));
        break;
    }
    case CAN_MOTOR_SDC102: sdc102_send_velocity(id, vel_rads); break;
    case CAN_MOTOR_ODRIVE:
        motor_record_tx(i, odrive_send_velocity(s_entries[i].node_id, vel_rads * s_entries[i].sign));
        break;
    }
    return true;
}

// ODRIVE only. Switches the axis's own closed-loop controller mode between
// velocity and torque at runtime -- odrive_init_axis() sets TORQUE once at
// boot (matching how every controller in this codebase drives wheels), but
// MotorTest's wheel sweep switches to VELOCITY for the duration of the
// test (confirmed on real hardware to track a commanded velocity much
// better than the torque-PID approach, which stalled under the wheel's own
// static friction) and back to TORQUE when the test stops. Always uses
// PASSTHROUGH input mode either way. Returns false if id not registered or
// not CAN_MOTOR_ODRIVE.
//
// IMPORTANT (found 2026-09-02, real hardware): a bare Set_Controller_Mode
// sent while the axis is ALREADY in CLOSED_LOOP_CONTROL can fault the axis
// on some ODrive firmware versions, silently dropping it back to IDLE --
// which then ignores torque commands entirely ("wheel doesn't turn" after
// a mode switch, with no obvious error visible from this side). Fixed by
// bracketing the mode change with the exact same IDLE -> mode -> settle ->
// CLOSED_LOOP -> settle sequence odrive_init_axis() already uses at boot,
// instead of changing mode in place while still closed-loop.
//
// settle: true blocks for ~150ms (three chThdSleepMilliseconds(50) calls)
// between each step, matching odrive_init_axis()'s proven timing -- use
// this from non-timing-critical callers (USBCmdThread) where reliability
// matters more than latency. false skips all three sleeps, sending the
// same three commands back-to-back -- use this from a timing-critical
// caller (e.g. ControlThread's arm-triggered abort path) where blocking
// for 150ms would itself be a worse problem (stalling every motor's torque
// command, not just this wheel's) than a best-effort revert that might
// occasionally need the ODrive's own retry/settle behavior to fully land.
bool can_motor_set_odrive_mode(uint8_t id, bool velocity_mode, bool settle)
{
    int i = find_entry(id);
    if (i < 0 || s_entries[i].proto != CAN_MOTOR_ODRIVE) return false;
    uint8_t wire_id = s_entries[i].node_id;

    odrive_set_axis_state(wire_id, ODRIVE_AXIS_STATE_IDLE);
    if (settle) chThdSleepMilliseconds(50);
    odrive_set_controller_mode(wire_id,
                                velocity_mode ? ODRIVE_CONTROL_MODE_VELOCITY : ODRIVE_CONTROL_MODE_TORQUE,
                                ODRIVE_INPUT_MODE_PASSTHROUGH);
    if (settle) chThdSleepMilliseconds(50);
    odrive_set_axis_state(wire_id, ODRIVE_AXIS_STATE_CLOSED_LOOP);
    if (settle) chThdSleepMilliseconds(50);
    return true;
}

// RMD (hip) only -- no position-mode wired up for SDC102/ODRIVE. target_rad
// is the ROBOT-FRAME angle (offset+sign applied automatically); maxspeed_rads
// is clamped to the same hip velocity rating everything else uses; cw
// selects which way the motor turns to reach the target in ROBOT-FRAME terms
// (defaults to CW, matching Motor_tool's RmdMotor.cpp convention -- there's
// no "shortest path" auto-selection here, same as that reference
// implementation) -- inverted to the opposite WIRE-level direction for a
// HIP_SIGN<0 hip, same reasoning as hip_to_raw() mirroring the angle itself.
bool can_motor_set_position(uint8_t id, float target_rad, float maxspeed_rads, bool cw)
{
    int i = find_entry(id);
    if (i < 0 || s_entries[i].proto != CAN_MOTOR_RMD) return false;
    target_rad    = hip_clamp_position_target(id, target_rad);
    maxspeed_rads = clampf(maxspeed_rads, 0.0f, HIP_VEL_LIMIT_RADS);
    bool wire_cw  = (HIP_SIGN[id - 1] > 0.0f) ? cw : !cw;
    rmd_send_position_single_turn(id, hip_to_raw(id, target_rad), maxspeed_rads, wire_cw);
    return true;
}

bool can_motor_request_encoder(uint8_t id)
{
    int i = find_entry(id);
    if (i < 0 || s_entries[i].proto != CAN_MOTOR_ODRIVE) return false;
    // Get_Encoder_Estimates has no dedicated "request" command byte in the
    // ODrive CAN Simple protocol -- an RTR frame at its own arbitration ID
    // is what triggers an immediate one-shot reply, instead of waiting for
    // whatever periodic broadcast rate the drive happens to be configured
    // for (see can_send_rtr()'s header comment in CAN.hpp).
    bool ok = can_send_rtr(CAN_BUS_1,
                           odrive_arb_id(s_entries[i].node_id, ODRIVE_CMD_GET_ENCODER_EST),
                           8);
    motor_record_tx(i, ok);
    return ok;
}

bool can_motor_get_state(uint8_t id, CanMotorState *out)
{
    int i = find_entry(id);
    if (i < 0 || !out) return false;
    chMtxLock(&motor_state_mtx);
    *out = g_motors[i];
    chMtxUnlock(&motor_state_mtx);
    return true;
}

bool can_motor_request_status(uint8_t id)
{
    int i = find_entry(id);
    if (i < 0 || s_entries[i].proto != CAN_MOTOR_RMD) return false;
    rmd_send_status_request(id);
    return true;
}

float can_motor_hip_angle_min(uint8_t id)
{
    if (id < 1 || id > 4) return 0.0f;
    return HIP_ANGLE_MIN_RAD[id - 1];
}

float can_motor_hip_angle_max(uint8_t id)
{
    if (id < 1 || id > 4) return 0.0f;
    return HIP_ANGLE_MAX_RAD[id - 1];
}

void can_motor_poll_status_round_robin(void)
{
    static int s_next = 0;   // index into s_entries, persists across calls
    if (s_count == 0) return;
    for (int n = 0; n < s_count; n++) {
        int i = (s_next + n) % s_count;
        if (s_entries[i].proto == CAN_MOTOR_RMD) {
            rmd_send_status_request(s_entries[i].id);
            s_next = (i + 1) % s_count;
            return;
        }
    }
}
