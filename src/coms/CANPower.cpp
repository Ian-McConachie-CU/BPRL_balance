#include "src/coms/CANPower.hpp"
#include "src/coms/CAN.hpp"
#include <cmath>

mutex_t       power_mtx;
PowerMonState g_power = { NAN, NAN, 0, false };

// IEEE 754 float16 → float32 conversion.
static float f16_to_f32(uint16_t h)
{
    const uint32_t s = (uint32_t)(h >> 15) & 1U;
    const uint32_t e = (uint32_t)(h >> 10) & 0x1FU;
    const uint32_t m = (uint32_t)(h)       & 0x3FFU;

    if (e == 31U) {
        // Infinity or NaN — propagate as NaN (quiet).
        const uint32_t nan_bits = 0x7FC00000U;
        float v; __builtin_memcpy(&v, &nan_bits, 4); return v;
    }
    if (e == 0U) {
        // Denormal or zero — treat as zero for power monitoring purposes.
        return s ? -0.0f : 0.0f;
    }
    // Normal: f16 bias = 15, f32 bias = 127 → add 112 to exponent.
    const uint32_t bits = (s << 31) | ((e + 112U) << 23) | (m << 13);
    float v; __builtin_memcpy(&v, &bits, 4); return v;
}

/*
 * DroneCAN BatteryInfo (DTID 1092) receive callback.
 *
 * BatteryInfo is always a multi-frame transfer (payload > 7 bytes).
 * We only read the first CAN frame, identified by start_of_transfer (tail bit 7 = 1).
 * The first 6 payload bytes always contain temperature, voltage, and current.
 */
static void battery_info_cb(const CANRxFrame &f, void *ctx)
{
    (void)ctx;
    if (f.DLC < 8) return;

    const uint8_t tail = f.data8[7];
    if (!(tail & 0x80U)) return;  // not start-of-transfer; skip continuation frames

    const uint16_t v_raw = (uint16_t)f.data8[2] | ((uint16_t)f.data8[3] << 8);
    const uint16_t i_raw = (uint16_t)f.data8[4] | ((uint16_t)f.data8[5] << 8);

    const float voltage = f16_to_f32(v_raw);
    const float current = f16_to_f32(i_raw);

    chMtxLock(&power_mtx);
    g_power.voltage_V = voltage;
    g_power.current_A = current;
    g_power.node_id   = (uint8_t)(f.ext.EID & 0x7FU);
    g_power.valid     = true;
    chMtxUnlock(&power_mtx);
}

void power_mon_init(void)
{
    chMtxObjectInit(&power_mtx);
    // BatteryInfo DTID 1092 = 0x0444; packed into EID bits [23:8].
    // Mask 0x00FFFF80 matches DTID bits [23:8] and broadcast indicator bit [7] = 0,
    // ignoring priority [28:24] and source node ID [6:0].
    bprl_can_register_ext(CAN_BUS_2, 0x00044400U, 0x00FFFF80U, battery_info_cb, nullptr);
}
