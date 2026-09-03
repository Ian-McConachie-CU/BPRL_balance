#include "src/OdriveMotor.hpp"
#include "ch.h"
#include <cstring>

static OdriveState s_state[ODRIVE_ID_MAX + 1] = {};   // indexed by node_id, 0 included (legal but unused here)
static CanBus       s_bus = CAN_BUS_1;

static constexpr uint8_t ODRIVE_CMD_HEARTBEAT        = 0x01;
static constexpr uint8_t ODRIVE_CMD_SET_AXIS_STATE   = 0x07;
static constexpr uint8_t ODRIVE_CMD_GET_ENCODER_EST  = 0x09;
static constexpr uint8_t ODRIVE_CMD_SET_CTRL_MODE    = 0x0B;
static constexpr uint8_t ODRIVE_CMD_SET_INPUT_VEL    = 0x0D;
static constexpr uint8_t ODRIVE_CMD_SET_INPUT_TORQUE = 0x0E;

static float s_gear_ratio = 8.0f;   // matches ODRIVE_GEAR_RATIO in src/coms/CANMotor.cpp (GIM6010-8)

// Same placeholder rating the parent firmware clamps to (ODRIVE_TORQUE_LIMIT_NM
// in src/coms/CANMotor.cpp) — a second, independent floor on top of whatever
// the ODrive axis's own config limits.
static constexpr float ODRIVE_DEFAULT_TORQUE_LIMIT_NM = 8.0f;
static float s_torque_limit = ODRIVE_DEFAULT_TORQUE_LIMIT_NM;

void odrive_set_bus(CanBus bus) { s_bus = bus; }
CanBus odrive_get_bus(void)     { return s_bus; }

void  odrive_set_gear_ratio(float ratio) { s_gear_ratio = ratio; }
float odrive_get_gear_ratio(void)        { return s_gear_ratio; }

void  odrive_set_torque_limit(float limit_Nm) { s_torque_limit = limit_Nm < 0 ? 0 : limit_Nm; }
float odrive_get_torque_limit(void)           { return s_torque_limit; }

static bool id_ok(uint8_t id) { return id <= ODRIVE_ID_MAX; }

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline uint32_t arb_id(uint8_t id, uint8_t cmd) { return ((uint32_t)id << 5) | cmd; }

bool odrive_set_mode(uint8_t id, bool velocity_mode)
{
    if (!id_ok(id)) return false;
    uint32_t control_mode = velocity_mode ? ODRIVE_CONTROL_MODE_VELOCITY : ODRIVE_CONTROL_MODE_TORQUE;
    uint32_t input_mode    = ODRIVE_INPUT_MODE_PASSTHROUGH;
    uint8_t data[8];
    memcpy(&data[0], &control_mode, 4);
    memcpy(&data[4], &input_mode, 4);
    return can_send(s_bus, arb_id(id, ODRIVE_CMD_SET_CTRL_MODE), data, 8);
}

static bool odrive_set_axis_state(uint8_t id, uint32_t state)
{
    if (!id_ok(id)) return false;
    uint8_t data[8] = {};
    memcpy(&data[0], &state, 4);
    return can_send(s_bus, arb_id(id, ODRIVE_CMD_SET_AXIS_STATE), data, 8);
}

bool odrive_start(uint8_t id)
{
    // Mirrors odrive_init_axis() in the parent firmware: set the controller
    // mode before requesting closed-loop, same short-settle-time ordering.
    bool mode_ok = odrive_set_mode(id, false /*torque*/);
    chThdSleepMilliseconds(50);
    bool state_ok = odrive_set_axis_state(id, ODRIVE_AXIS_STATE_CLOSED_LOOP);
    return mode_ok && state_ok;
}

bool odrive_idle(uint8_t id) { return odrive_set_axis_state(id, ODRIVE_AXIS_STATE_IDLE); }

bool odrive_torque(uint8_t id, float torque_Nm)
{
    if (!id_ok(id)) return false;
    torque_Nm = clampf(torque_Nm, -s_torque_limit, s_torque_limit);
    float motor_torque_Nm = torque_Nm / s_gear_ratio;
    uint8_t data[8] = {};
    memcpy(&data[0], &motor_torque_Nm, 4);   // IEEE float, LSB byte order (matches STM32 native)
    return can_send(s_bus, arb_id(id, ODRIVE_CMD_SET_INPUT_TORQUE), data, 8);
}

bool odrive_velocity(uint8_t id, float vel_rads)
{
    if (!id_ok(id)) return false;
    // Reproduces the parent firmware's odrive_send_velocity() verbatim,
    // including its NOTE: this multiplies by gear ratio only, not a
    // rad/s -> turns/s conversion (no /2pi), because that's what was
    // actually validated on this motor in final-project-Ian-McConachie-CU.
    float motor_cmd = vel_rads * s_gear_ratio;
    float torque_ff = 0.0f;
    uint8_t data[8];
    memcpy(&data[0], &motor_cmd, 4);
    memcpy(&data[4], &torque_ff, 4);
    return can_send(s_bus, arb_id(id, ODRIVE_CMD_SET_INPUT_VEL), data, 8);
}

bool odrive_get_state(uint8_t id, OdriveState &out)
{
    if (!id_ok(id)) return false;
    // s_state[id] is written field-by-field by odrive_rx_cb() on the CAN RX
    // thread — same torn-read concern as RmdMotor/GimMotor's get_state(),
    // same fix (lock around the whole-struct copy).
    chSysLock();
    out = s_state[id];
    chSysUnlock();
    return true;
}

int odrive_list_seen(uint8_t *out, int max, uint32_t max_age_ms)
{
    uint32_t now = (uint32_t)TIME_I2MS(chVTGetSystemTime());
    int n = 0;
    for (int id = 0; id <= ODRIVE_ID_MAX && n < max; id++) {
        chSysLock();
        bool     valid = s_state[id].valid;
        uint32_t age   = now - s_state[id].last_heartbeat_ms;
        chSysUnlock();
        if (valid && age <= max_age_ms) out[n++] = (uint8_t)id;
    }
    return n;
}

void odrive_stop_all(void)
{
    for (int id = 0; id <= ODRIVE_ID_MAX; id++)
        if (s_state[id].valid) odrive_idle((uint8_t)id);
}

static void odrive_rx_cb(CanBus bus, const CANRxFrame &f, void *ctx)
{
    (void)ctx;
    if (bus != s_bus || f.common.XTD || f.DLC < 8) return;

    uint32_t sid = f.std.SID;
    uint8_t  cmd = (uint8_t)(sid & 0x1FU);
    uint8_t  id  = (uint8_t)(sid >> 5);   // exact for an 11-bit SID: max 0x7FF >> 5 == ODRIVE_ID_MAX

    uint32_t now = (uint32_t)TIME_I2MS(chVTGetSystemTime());
    OdriveState &st = s_state[id];

    chSysLock();
    if (cmd == ODRIVE_CMD_HEARTBEAT) {
        memcpy(&st.axis_error, &f.data8[0], 4);
        st.axis_state        = f.data8[4];
        st.valid              = true;
        st.last_heartbeat_ms = now;
    } else if (cmd == ODRIVE_CMD_GET_ENCODER_EST) {
        float pos_turns, vel_turns_s;
        memcpy(&pos_turns, &f.data8[0], 4);
        memcpy(&vel_turns_s, &f.data8[4], 4);
        st.pos_rad  = pos_turns * (2.0f * 3.14159265f) / s_gear_ratio;
        st.vel_rads = vel_turns_s * (2.0f * 3.14159265f) / s_gear_ratio;
    }
    chSysUnlock();
}

void odrive_init(void)
{
    memset(s_state, 0, sizeof(s_state));
    can_subscribe(odrive_rx_cb, nullptr);
}
