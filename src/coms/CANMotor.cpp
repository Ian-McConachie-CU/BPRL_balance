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
 * RMD / MyActuator protocol
 *   Command:  SID = 0x140 + motor_id,  8 bytes
 *   Response: SID = 0x240 + motor_id,  8 bytes
 *
 * Torque command (0xA1):
 *   data[0] = 0xA1
 *   data[1..3] = 0x00
 *   data[4..5] = int16 torque (LSB first), scale: 0.01 A/LSB (current loop)
 *   data[6..7] = 0x00
 *   Response echoes position, speed, torque, temp.
 *
 * Velocity command (0xA2):
 *   data[0] = 0xA2
 *   data[1..3] = 0x00
 *   data[4..7] = int32 speed (LSB first), 0.01 dps/LSB
 * ════════════════════════════════════════════════════════════════════════════ */

// Torque constant approximation: 1 Nm ≈ 6 A for the MG8016E-i6.
// Tune this per actual motor spec sheet.
static constexpr float RMD_TORQUE_SCALE = 6.0f;  // A/Nm (current per torque)
static constexpr float RMD_I_TO_LSB     = 100.0f; // 0.01 A/LSB → 100 LSB/A

static void rmd_send_torque(uint8_t id, float torque_Nm)
{
    int16_t iq = (int16_t)(torque_Nm * RMD_TORQUE_SCALE * RMD_I_TO_LSB);
    uint8_t data[8] = {};
    data[0] = 0xA1;
    data[4] = (uint8_t)((uint16_t)iq & 0xFFU);
    data[5] = (uint8_t)(((uint16_t)iq >> 8) & 0xFFU);
    can_send(CAN_BUS_1, 0x140U + id, data, 8);
}

static void rmd_send_velocity(uint8_t id, float vel_rads)
{
    // Scale: vel (rad/s) → dps → LSB  (0.01 dps/LSB)
    int32_t spd = (int32_t)(vel_rads * (180.0f / 3.14159265f) * 100.0f);
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

    // Response byte 0 = command echo; bytes 1=temp, 2-3=iq, 4-5=speed, 6-7=angle
    int16_t raw_iq    = (int16_t)((uint16_t)f.data8[2] | ((uint16_t)f.data8[3] << 8));
    int16_t raw_speed = (int16_t)((uint16_t)f.data8[4] | ((uint16_t)f.data8[5] << 8));
    int16_t raw_angle = (int16_t)((uint16_t)f.data8[6] | ((uint16_t)f.data8[7] << 8));
    uint8_t temp      = f.data8[1];

    chMtxLock(&motor_state_mtx);
    g_motors[idx].torque_Nm = (float)raw_iq / (RMD_TORQUE_SCALE * RMD_I_TO_LSB);
    g_motors[idx].vel_rads  = (float)raw_speed * (3.14159265f / 18000.0f); // dps×0.1 → rad/s
    g_motors[idx].pos_rad   = (float)raw_angle * (3.14159265f / 18000.0f);
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
        // Register response callback for this motor's reply ID
        bprl_can_register(CAN_BUS_1, 0x240U + id, rmd_rx_cb, (void *)(uintptr_t)idx);
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
