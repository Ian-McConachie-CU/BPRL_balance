#include "src/coms/CANMotor.hpp"
#include "src/coms/CAN.hpp"
#include <cstring>
#include <cmath>

/* ── Global state table ─────────────────────────────────────────────────── */

MUTEX_DECL(motor_state_mtx);
CanMotorState g_motors[CAN_MOTOR_MAX] = {};

struct MotorEntry {
    uint8_t       id;
    CanMotorProto proto;
    bool          registered;
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

void can_motor_register(uint8_t id, CanMotorProto proto)
{
    if (s_count >= CAN_MOTOR_MAX) return;

    int idx = s_count++;
    s_entries[idx] = {id, proto, true};

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
