#include "src/GimMotor.hpp"
#include "ch.h"
#include <cstring>

static GimState s_state[GIM_ID_MAX + 1] = {};   // index 0 unused
static CanBus   s_bus = CAN_BUS_1;

// Placeholders — see the header comment on gim_set_torque_constant().
static float s_torque_constant = 1.0f;   // Nm/A, UNKNOWN default
static float s_gear_ratio      = 1.0f;   // UNKNOWN default

// Continuous rating from wheeled_biped_project_notes.md.
static constexpr float GIM_DEFAULT_TORQUE_LIMIT_NM = 3.3f;
static float s_torque_limit = GIM_DEFAULT_TORQUE_LIMIT_NM;

void gim_set_bus(CanBus bus) { s_bus = bus; }
CanBus gim_get_bus(void)     { return s_bus; }

void  gim_set_torque_constant(float Kt_Nm_per_A) { s_torque_constant = Kt_Nm_per_A; }
float gim_get_torque_constant(void)              { return s_torque_constant; }
void  gim_set_gear_ratio(float ratio)            { s_gear_ratio = ratio; }
float gim_get_gear_ratio(void)                   { return s_gear_ratio; }

void  gim_set_torque_limit(float limit_Nm) { s_torque_limit = limit_Nm < 0 ? 0 : limit_Nm; }
float gim_get_torque_limit(void)           { return s_torque_limit; }

static bool id_ok(uint8_t id) { return id >= 1 && id <= GIM_ID_MAX; }

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static bool send_simple(uint8_t id, uint8_t cmd)
{
    if (!id_ok(id)) return false;
    uint8_t data[8] = {cmd, 0, 0, 0, 0, 0, 0, 0};
    return can_send(s_bus, id, data, 8);
}

bool gim_start(uint8_t id)
{
    bool ok = send_simple(id, 0x91);
    if (ok) s_state[id].enabled = true;   // optimistic; corrected by the ack (see rx cb)
    return ok;
}

bool gim_stop(uint8_t id)
{
    bool ok = send_simple(id, 0x92);
    if (ok) s_state[id].enabled = false;
    return ok;
}

bool gim_pause(uint8_t id) { return send_simple(id, 0x97); }

bool gim_torque(uint8_t id, float torque_Nm, uint32_t duration_ms)
{
    if (!id_ok(id) || !s_state[id].enabled) return false;
    torque_Nm = clampf(torque_Nm, -s_torque_limit, s_torque_limit);

    uint8_t data[8];
    data[0] = 0x93;
    memcpy(&data[1], &torque_Nm, 4);   // IEEE float, LSB byte order (matches STM32 native)
    data[5] = (uint8_t)(duration_ms & 0xFFU);
    data[6] = (uint8_t)((duration_ms >> 8) & 0xFFU);
    data[7] = (uint8_t)((duration_ms >> 16) & 0xFFU);
    return can_send(s_bus, id, data, 8);
}

bool gim_velocity(uint8_t id, float vel_rads, uint32_t duration_ms)
{
    if (!id_ok(id) || !s_state[id].enabled) return false;
    // rad/s -> RPM (spec's Speed Control unit)
    float rpm = vel_rads * (60.0f / (2.0f * 3.14159265f));

    uint8_t data[8];
    data[0] = 0x94;
    memcpy(&data[1], &rpm, 4);
    data[5] = (uint8_t)(duration_ms & 0xFFU);
    data[6] = (uint8_t)((duration_ms >> 8) & 0xFFU);
    data[7] = (uint8_t)((duration_ms >> 16) & 0xFFU);
    return can_send(s_bus, id, data, 8);
}

bool gim_position(uint8_t id, float pos_rad, uint32_t duration_ms)
{
    if (!id_ok(id) || !s_state[id].enabled) return false;

    uint8_t data[8];
    data[0] = 0x95;
    memcpy(&data[1], &pos_rad, 4);   // spec's Position Control unit is RAD directly
    data[5] = (uint8_t)(duration_ms & 0xFFU);
    data[6] = (uint8_t)((duration_ms >> 8) & 0xFFU);
    data[7] = (uint8_t)((duration_ms >> 16) & 0xFFU);
    return can_send(s_bus, id, data, 8);
}

bool gim_get_fault(uint8_t id) { return send_simple(id, 0xB2); }
bool gim_ack_fault(uint8_t id) { return send_simple(id, 0xB3); }

bool gim_get_indicator(uint8_t id, uint8_t ind_id)
{
    if (!id_ok(id)) return false;
    uint8_t data[8] = {0xB4, ind_id, 0, 0, 0, 0, 0, 0};
    return can_send(s_bus, id, data, 8);
}

bool gim_get_state(uint8_t id, GimState &out)
{
    if (!id_ok(id)) return false;
    // See the matching comment in RmdMotor.cpp's rmd_get_state() — s_state[id]
    // is written field-by-field by gim_rx_cb() on the CAN RX thread, so this
    // whole-struct read needs to be locked against it to avoid a torn mix of
    // old/new fields.
    chSysLock();
    out = s_state[id];
    chSysUnlock();
    return true;
}

void gim_stop_all(void)
{
    for (uint8_t id = 1; id <= GIM_ID_MAX; id++)
        if (s_state[id].enabled || s_state[id].valid) gim_stop(id);
}

// Torque/Speed/Position responses share this layout (spec 3.2.7-3.2.9):
// byte1=RES, byte2=Temp, byte3-4=packed pos (16-bit), byte5-7=ST0-2 (packed
// 12-bit speed + 12-bit torque).
static void decode_control_response(uint8_t id, const uint8_t *d)
{
    GimState &st = s_state[id];

    uint16_t pos_raw = (uint16_t)d[3] | ((uint16_t)d[4] << 8);
    // 12-bit speed: ST0 (d[5]) is the high 8 bits, ST1[7:4] (d[6]) the low 4.
    uint16_t speed_raw = ((uint16_t)d[5] << 4) | (uint16_t)(d[6] >> 4);
    // 12-bit torque: ST1[3:0] (d[6]) is the high 4 bits, ST2 (d[7]) the low 8.
    uint16_t torque_raw = ((uint16_t)(d[6] & 0x0FU) << 8) | (uint16_t)d[7];

    st.pos_rad  = (float)pos_raw   * (25.0f / 65535.0f) - 12.5f;
    st.vel_rads = (float)speed_raw * (130.0f / 4095.0f) - 65.0f;

    float kt_gear = s_torque_constant * s_gear_ratio;
    st.torque_Nm = (float)torque_raw * (450.0f * kt_gear / 4095.0f) - 225.0f * kt_gear;

    st.temp_C = (float)d[2];
    st.valid  = true;
    st.last_update_ms = (uint32_t)TIME_I2MS(chVTGetSystemTime());
}

static void gim_rx_cb(CanBus bus, const CANRxFrame &f, void *ctx)
{
    (void)ctx;
    if (bus != s_bus || f.common.XTD || f.DLC < 8) return;

    // Check the full SID before truncating to uint8_t for id_ok() — SID is
    // 11 bits, and truncating first would alias e.g. 0x10A (266) down to 10
    // and misattribute an unrelated frame to GIM id 10.
    uint32_t sid = f.std.SID;
    if (sid < 1U || sid > (uint32_t)GIM_ID_MAX) return;   // response ID == node ID, see header comment
    uint8_t id = (uint8_t)sid;

    GimState &st = s_state[id];
    uint8_t  cmd = f.data8[0];
    uint32_t now = (uint32_t)TIME_I2MS(chVTGetSystemTime());

    // All cases below only touch s_state[id] with plain field writes (no
    // blocking calls), and gim_get_state() reads the whole struct at once
    // from the command thread — lock so that read never sees a torn mix of
    // fields from two different updates.
    chSysLock();
    switch (cmd) {
    case 0x91:   // Start Motor ack
        st.enabled = (f.data8[1] == 0x00);   // RES==0x00 (Success)
        st.valid = true; st.last_update_ms = now;
        break;
    case 0x92:   // Stop Motor ack
        st.enabled = false;
        st.valid = true; st.last_update_ms = now;
        break;
    case 0x93: case 0x94: case 0x95:   // Torque/Speed/Position response
        decode_control_response(id, f.data8);
        break;
    case 0xB2:   // Get Fault
        st.fault = f.data8[2];
        st.valid = true; st.last_update_ms = now;
        break;
    case 0xB3:   // Acknowledge Fault ack
        st.valid = true; st.last_update_ms = now;
        break;
    case 0xB4: {   // Retrieve Indicator
        float val;
        memcpy(&val, &f.data8[4], 4);
        st.last_indicator    = val;
        st.last_indicator_id = f.data8[1];
        st.valid = true; st.last_update_ms = now;
        break;
    }
    default:
        break;
    }
    chSysUnlock();
}

void gim_init(void)
{
    memset(s_state, 0, sizeof(s_state));
    can_subscribe(gim_rx_cb, nullptr);
}
