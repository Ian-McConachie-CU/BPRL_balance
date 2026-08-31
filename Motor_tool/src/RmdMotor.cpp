#include "src/RmdMotor.hpp"
#include "ch.h"
#include <cmath>
#include <cstring>

// Torque-ratio scale for the rmd_torque(Nm) convenience wrapper. CAN
// PROTOCOL V2.35 confirms the raw command is iqControl, int16_t range
// -2048..2048 corresponding to -33..33 A for the MG series (33/2048 A per
// LSB) — but gives no Nm mapping (the motor's Kt isn't documented). This
// default (~83.3 ratio/Nm) is a ROUGH placeholder derived only from
// wheeled_biped_project_notes.md's ~12 Nm continuous rating — it is NOT a
// measured calibration. Prefer rmd_torque_raw() until you've calibrated this
// against a real load or torque wrench.
static constexpr float RMD_TORQUE_SCALE_DEFAULT = 83.3f;   // ratio/Nm, placeholder
static constexpr int16_t RMD_TORQUE_RATIO_MAX   = 2048;    // confirmed by CAN PROTOCOL V2.35
static constexpr float RMD_SPEED_DPS_MAX        = 24000.0f; // confirmed by the MGv2 manual

static RmdState s_state[RMD_ID_MAX + 1] = {};   // index 0 unused
static CanBus   s_bus = CAN_BUS_1;
static float    s_torque_scale = RMD_TORQUE_SCALE_DEFAULT;

void rmd_set_bus(CanBus bus) { s_bus = bus; }
CanBus rmd_get_bus(void)     { return s_bus; }

void  rmd_set_torque_scale(float ratio_per_Nm) { s_torque_scale = ratio_per_Nm; }
float rmd_get_torque_scale(void)               { return s_torque_scale; }

static bool id_ok(uint8_t id) { return id >= 1 && id <= RMD_ID_MAX; }

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static bool send_cmd(uint8_t id, const uint8_t data[8])
{
    if (!id_ok(id)) return false;
    return can_send(s_bus, 0x140U + id, data, 8);
}

bool rmd_torque_raw(uint8_t id, int16_t ratio)
{
    if (ratio > RMD_TORQUE_RATIO_MAX) ratio = RMD_TORQUE_RATIO_MAX;
    if (ratio < -RMD_TORQUE_RATIO_MAX) ratio = -RMD_TORQUE_RATIO_MAX;
    uint8_t data[8] = {};
    data[0] = 0xA1;
    data[4] = (uint8_t)((uint16_t)ratio & 0xFFU);
    data[5] = (uint8_t)(((uint16_t)ratio >> 8) & 0xFFU);
    return send_cmd(id, data);
}

bool rmd_torque(uint8_t id, float torque_Nm)
{
    int16_t ratio = (int16_t)clampf(torque_Nm * s_torque_scale,
                                     -RMD_TORQUE_RATIO_MAX, RMD_TORQUE_RATIO_MAX);
    return rmd_torque_raw(id, ratio);
}

bool rmd_velocity(uint8_t id, float vel_rads)
{
    // rad/s -> dps -> LSB (0.01 dps/LSB), clamped to the confirmed +/-24000 dps range
    float dps = clampf(vel_rads * (180.0f / 3.14159265f), -RMD_SPEED_DPS_MAX, RMD_SPEED_DPS_MAX);
    int32_t spd = (int32_t)(dps * 100.0f);
    uint8_t data[8] = {};
    data[0] = 0xA2;
    data[4] = (uint8_t)((uint32_t)spd & 0xFFU);
    data[5] = (uint8_t)(((uint32_t)spd >> 8)  & 0xFFU);
    data[6] = (uint8_t)(((uint32_t)spd >> 16) & 0xFFU);
    data[7] = (uint8_t)(((uint32_t)spd >> 24) & 0xFFU);
    return send_cmd(id, data);
}

bool rmd_position(uint8_t id, float angle_rad, float maxspeed_rads)
{
    // Multi Loop Angle Control 2 (0xA4, confirmed by CAN PROTOCOL V2.35):
    // data[2..3] = max speed, uint16_t, 1 dps/LSB; data[4..7] = target
    // angle, int32_t, 0.01 deg/LSB. (0xA3 is the angle-only variant with no
    // speed limit field — not used here since this function always takes one.)
    uint16_t maxspeed_dps = (uint16_t)(maxspeed_rads * (180.0f / 3.14159265f));
    int32_t  angle_lsb    = (int32_t)(angle_rad * (180.0f / 3.14159265f) * 100.0f);
    uint8_t data[8] = {};
    data[0] = 0xA4;
    data[2] = (uint8_t)(maxspeed_dps & 0xFFU);
    data[3] = (uint8_t)((maxspeed_dps >> 8) & 0xFFU);
    data[4] = (uint8_t)((uint32_t)angle_lsb & 0xFFU);
    data[5] = (uint8_t)(((uint32_t)angle_lsb >> 8)  & 0xFFU);
    data[6] = (uint8_t)(((uint32_t)angle_lsb >> 16) & 0xFFU);
    data[7] = (uint8_t)(((uint32_t)angle_lsb >> 24) & 0xFFU);
    return send_cmd(id, data);
}

bool rmd_position_single_turn(uint8_t id, float angle_rad, float maxspeed_rads, bool cw)
{
    // Single Loop Angle Control 2 (0xA6): data[1] = spinDirection
    // (0x00=CW, 0x01=CCW — the direction it turns to REACH the target, not
    // derived from target-vs-current like 0xA4); data[2..3] = max speed,
    // uint16_t, 1 dps/LSB; data[4..7] = target angle, uint32_t, 0.01 deg/LSB,
    // ABSOLUTE but SINGLE-TURN (0..359.99 deg, wraps every revolution) — the
    // same reference frame as rmd_read_encoder()'s 0x90 reply, unlike
    // rmd_position()'s 0xA4 which targets absolute multi-turn position.
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
    return send_cmd(id, data);
}

bool rmd_increment_position(uint8_t id, float delta_rad)
{
    // Increment Angle Control 1 (0xA7): data[4..7] = angleIncrement,
    // int32_t, 0.01 deg/LSB, signed — relative to current position, no
    // speed-limit field (data[1..3] NULL). See header note for why this
    // exists alongside rmd_position()'s absolute 0xA4.
    int32_t angle_lsb = (int32_t)(delta_rad * (180.0f / 3.14159265f) * 100.0f);
    uint8_t data[8] = {};
    data[0] = 0xA7;
    data[4] = (uint8_t)((uint32_t)angle_lsb & 0xFFU);
    data[5] = (uint8_t)(((uint32_t)angle_lsb >> 8)  & 0xFFU);
    data[6] = (uint8_t)(((uint32_t)angle_lsb >> 16) & 0xFFU);
    data[7] = (uint8_t)(((uint32_t)angle_lsb >> 24) & 0xFFU);
    return send_cmd(id, data);
}

bool rmd_stop(uint8_t id)
{
    uint8_t data[8] = {};
    data[0] = 0x81;
    return send_cmd(id, data);
}

bool rmd_off(uint8_t id)
{
    uint8_t data[8] = {};
    data[0] = 0x80;
    return send_cmd(id, data);
}

bool rmd_resume(uint8_t id)
{
    uint8_t data[8] = {};
    data[0] = 0x88;
    return send_cmd(id, data);
}

bool rmd_request_status(uint8_t id)
{
    uint8_t data[8] = {};
    data[0] = 0x9A;
    return send_cmd(id, data);
}

bool rmd_clear_error(uint8_t id)
{
    uint8_t data[8] = {};
    data[0] = 0x9B;
    return send_cmd(id, data);
}

bool rmd_read_encoder(uint8_t id)
{
    uint8_t data[8] = {};
    data[0] = 0x90;
    return send_cmd(id, data);
}

bool rmd_get_state(uint8_t id, RmdState &out)
{
    if (!id_ok(id)) return false;
    // s_state[id] is written field-by-field by rmd_rx_cb() on the CAN RX
    // thread; without this, a STATUS read from the command thread can race
    // it and see a torn mix of old/new fields (e.g. a stale pos paired with
    // a fresh torque). Both sides are short, non-blocking memory copies, so
    // a S-locked critical section is cheap and safe here.
    chSysLock();
    out = s_state[id];
    chSysUnlock();
    return true;
}

void rmd_stop_all(void)
{
    for (uint8_t id = 1; id <= RMD_ID_MAX; id++)
        if (s_state[id].valid) rmd_stop(id);
}

static void rmd_rx_cb(CanBus bus, const CANRxFrame &f, void *ctx)
{
    (void)ctx;
    if (bus != s_bus || f.common.XTD || f.DLC < 8) return;

    // Confirmed by CAN PROTOCOL V2.35: the reply frame uses the SAME
    // identifier as the command (0x140+id), not 0x240+id.
    uint32_t sid = f.std.SID;
    if (sid <= 0x140U || sid > 0x140U + RMD_ID_MAX) return;
    uint8_t id = (uint8_t)(sid - 0x140U);

    RmdState &st = s_state[id];
    uint8_t cmd = f.data8[0];

    switch (cmd) {
    case 0xA1: case 0xA2: case 0xA3: case 0xA4: case 0xA6: case 0xA7: {
        // Confirmed by CAN PROTOCOL V2.35 (torque/speed/angle command
        // response tables — all share this layout, only the command echo
        // byte differs): byte1=temp (int8, 1C/LSB), byte2-3=torque current
        // iq (int16, raw +/-2048 units = -33..33A for MG series), byte4-5=
        // speed (int16, 1 dps/LSB), byte6-7=encoder position (uint16 — the
        // MGv2 manual's driver spec confirms an 18-bit encoder for this
        // series, and this field holds its top 16 bits per the protocol
        // doc, so 0..65535 maps to a full 0..360 deg revolution).
        int16_t  raw_iq    = (int16_t)((uint16_t)f.data8[2] | ((uint16_t)f.data8[3] << 8));
        int16_t  raw_speed = (int16_t)((uint16_t)f.data8[4] | ((uint16_t)f.data8[5] << 8));
        uint16_t raw_enc   = (uint16_t)f.data8[6] | ((uint16_t)f.data8[7] << 8);
        float torque_Nm = s_torque_scale > 0.0f ? (float)raw_iq / s_torque_scale : 0.0f;
        float vel_rads  = (float)raw_speed * (3.14159265f / 180.0f);
        float pos_rad   = (float)raw_enc * (3.14159265f / 32768.0f);
        float temp_C    = (float)f.data8[1];
        uint32_t now    = (uint32_t)TIME_I2MS(chVTGetSystemTime());
        // Struct is also read (as a whole) from the command thread via
        // rmd_get_state() — lock so that read never sees a torn mix of
        // fields from two different updates.
        chSysLock();
        st.torque_Nm = torque_Nm;
        st.vel_rads  = vel_rads;
        st.pos_rad   = pos_rad;
        st.temp_C    = temp_C;
        st.valid     = true;
        st.last_update_ms = now;
        chSysUnlock();
        break;
    }
    case 0x9A: {
        // byte1=temp (int8, 1C/LSB), byte7=errorState — these match CAN
        // PROTOCOL V2.35 and real hardware. The voltage field does NOT:
        // the vendor doc says byte3-4 at 0.1V/LSB, but on real MG8016E-i6
        // replies (poll hips, 2026-08-31) that lands 4x too low and one
        // byte late — byte3 sat at a constant 0x10 across every unit while
        // byte2 (documented as NULL) tracked real sensor noise. Empirically
        // confirmed: byte2-3 at 0.01V/LSB puts all 4 hips at 41.8-42.1V
        // against a known 41V bench supply. Trust the hardware over the doc
        // here.
        int8_t   temp = (int8_t)f.data8[1];
        uint16_t volt = (uint16_t)f.data8[2] | ((uint16_t)f.data8[3] << 8);
        uint32_t now  = (uint32_t)TIME_I2MS(chVTGetSystemTime());
        chSysLock();
        st.temp_C      = (float)temp;
        st.voltage_V   = (float)volt * 0.01f;
        st.error_flags = f.data8[7];
        st.valid       = true;
        st.last_update_ms = now;
        chSysUnlock();
        break;
    }
    case 0x90: {
        // Read encoder reply. byte2-3 = encoder, uint16_t, LSB order,
        // already relative to the drive's configured zero point — that part
        // matches the vendor doc and is now confirmed (poll hips,
        // 2026-08-31: byte2-3 = encoder, byte4-5 = encoderRaw, byte6-7 =
        // encoderOffset=0, so encoder == encoderRaw - offset checks out
        // internally). What doesn't match: the doc's generic example calls
        // this a 14-bit field (0..16383), but this MG8016E-i6/DG80R7E's own
        // GUI reports "Encoder Type: 16Bit Encoder" (Ktech_manual.pdf) —
        // real raw values here go well past 16383 (e.g. 54735), and scaling
        // as 16-bit puts every reading back in a sane 0-360deg single-turn
        // range. Confirmed against hardware: trust 16-bit for this unit.
        uint16_t enc = (uint16_t)f.data8[2] | ((uint16_t)f.data8[3] << 8);
        float pos_rad = (float)enc * (2.0f * 3.14159265f / 65536.0f);
        uint32_t now  = (uint32_t)TIME_I2MS(chVTGetSystemTime());
        chSysLock();
        st.pos_rad = pos_rad;
        st.valid   = true;
        st.last_update_ms = now;
        chSysUnlock();
        break;
    }
    default:
        break;
    }
}

void rmd_init(void)
{
    memset(s_state, 0, sizeof(s_state));
    can_subscribe(rmd_rx_cb, nullptr);
}
