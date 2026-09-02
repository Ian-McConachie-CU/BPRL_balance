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

static void rmd_send_torque_raw(uint8_t id, int16_t ratio)
{
    if (ratio > RMD_TORQUE_RATIO_MAX) ratio = RMD_TORQUE_RATIO_MAX;
    if (ratio < -RMD_TORQUE_RATIO_MAX) ratio = -RMD_TORQUE_RATIO_MAX;
    uint8_t data[8] = {};
    data[0] = 0xA1;
    data[4] = (uint8_t)((uint16_t)ratio & 0xFFU);
    data[5] = (uint8_t)(((uint16_t)ratio >> 8) & 0xFFU);
    can_send(CAN_BUS_1, 0x140U + id, data, 8);
}

static void rmd_send_torque(uint8_t id, float torque_Nm)
{
    int16_t ratio = (int16_t)clampf(torque_Nm * s_rmd_torque_scale,
                                     -RMD_TORQUE_RATIO_MAX, RMD_TORQUE_RATIO_MAX);
    rmd_send_torque_raw(id, ratio);
}

static void rmd_send_velocity(uint8_t id, float vel_rads)
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
    can_send(CAN_BUS_1, 0x140U + id, data, 8);
}

static void rmd_rx_cb(const CANRxFrame &f, void *ctx)
{
    int idx = (int)(uintptr_t)ctx;
    if (f.DLC < 8) return;

    // Confirmed by CAN PROTOCOL V2.35: byte0=command echo, byte1=temp,
    // byte2-3=iq (int16, same +/-2048 units as the command), byte4-5=speed
    // (int16, 1 dps/LSB), byte6-7=encoder position (uint16, top 16 bits of
    // the MG series' 18-bit encoder -> 0..65535 over 0..360 deg).
    int16_t  raw_iq    = (int16_t)((uint16_t)f.data8[2] | ((uint16_t)f.data8[3] << 8));
    int16_t  raw_speed = (int16_t)((uint16_t)f.data8[4] | ((uint16_t)f.data8[5] << 8));
    uint16_t raw_enc   = (uint16_t)f.data8[6] | ((uint16_t)f.data8[7] << 8);
    uint8_t  temp      = f.data8[1];

    chMtxLock(&motor_state_mtx);
    g_motors[idx].torque_Nm = s_rmd_torque_scale > 0.0f
                                  ? (float)raw_iq / s_rmd_torque_scale : 0.0f;
    g_motors[idx].vel_rads  = (float)raw_speed * (3.14159265f / 180.0f);
    g_motors[idx].pos_rad   = (float)raw_enc * (3.14159265f / 32768.0f);
    g_motors[idx].temp_C    = (float)temp;
    g_motors[idx].valid     = true;
    chMtxUnlock(&motor_state_mtx);
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
static void odrive_send_torque(uint8_t id, float torque_Nm)
{
    torque_Nm = clampf(torque_Nm, -ODRIVE_TORQUE_LIMIT_NM, ODRIVE_TORQUE_LIMIT_NM);
    float motor_torque_Nm = torque_Nm / ODRIVE_GEAR_RATIO;
    uint8_t data[8] = {};
    memcpy(&data[0], &motor_torque_Nm, 4);   // IEEE float, LSB byte order (matches STM32 native)
    can_send(CAN_BUS_1, odrive_arb_id(id, ODRIVE_CMD_SET_INPUT_TORQUE), data, 8);
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
static void odrive_send_velocity(uint8_t id, float vel_rads)
{
    float motor_cmd = vel_rads * ODRIVE_GEAR_RATIO;
    float torque_ff = 0.0f;
    uint8_t data[8];
    memcpy(&data[0], &motor_cmd, 4);
    memcpy(&data[4], &torque_ff, 4);
    can_send(CAN_BUS_1, odrive_arb_id(id, ODRIVE_CMD_SET_INPUT_VEL), data, 8);
}

static void odrive_rx_cb(const CANRxFrame &f, void *ctx)
{
    int idx = (int)(uintptr_t)ctx;
    if (f.DLC < 8) return;

    uint8_t cmd = (uint8_t)(f.std.SID & 0x1FU);

    chMtxLock(&motor_state_mtx);
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
        g_motors[idx].pos_rad  = pos_turns * (2.0f * 3.14159265f) / ODRIVE_GEAR_RATIO;
        g_motors[idx].vel_rads = vel_turns_s * (2.0f * 3.14159265f) / ODRIVE_GEAR_RATIO;
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

void can_motor_register(uint8_t id, CanMotorProto proto, uint8_t node_id)
{
    if (s_count >= CAN_MOTOR_MAX) return;

    // node_id=0 (default) means "same as id" — preserves old behavior for
    // RMD/SDC102, and for ODRIVE whenever the physical drive's node id
    // happens to match this project's slot numbering.
    uint8_t wire_id = (node_id != 0) ? node_id : id;

    int idx = s_count++;
    s_entries[idx] = {id, proto, true, wire_id};

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
    case CAN_MOTOR_RMD:    rmd_send_torque(id, torque_Nm);    break;
    case CAN_MOTOR_SDC102: sdc102_send_torque(id, torque_Nm); break;
    case CAN_MOTOR_ODRIVE: odrive_send_torque(s_entries[i].node_id, torque_Nm); break;
    }
    return true;
}

bool can_motor_set_torque_raw(uint8_t id, int16_t ratio)
{
    int i = find_entry(id);
    if (i < 0 || s_entries[i].proto != CAN_MOTOR_RMD) return false;
    rmd_send_torque_raw(id, ratio);
    return true;
}

bool can_motor_set_velocity(uint8_t id, float vel_rads)
{
    int i = find_entry(id);
    if (i < 0) return false;
    switch (s_entries[i].proto) {
    case CAN_MOTOR_RMD:    rmd_send_velocity(id, vel_rads);    break;
    case CAN_MOTOR_SDC102: sdc102_send_velocity(id, vel_rads); break;
    case CAN_MOTOR_ODRIVE: odrive_send_velocity(s_entries[i].node_id, vel_rads); break;
    }
    return true;
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
