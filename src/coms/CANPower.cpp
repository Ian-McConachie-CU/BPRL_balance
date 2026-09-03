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
 * Reassembly state for the in-flight BatteryInfo transfer. can_dispatch()
 * (and every RX callback it calls, including this one) runs exclusively on
 * CANThread -- there is no concurrent access to these, so no lock is needed
 * around them (only around the final g_power publish, which other threads
 * do read).
 */
static bool     s_pending_valid    = false;
static uint8_t  s_pending_node_id  = 0;
static uint8_t  s_pending_tid      = 0;
static bool     s_pending_toggle   = false;   // expected toggle bit on frame 2
static uint16_t s_pending_volt_raw = 0;
static uint8_t  s_pending_curr_lo  = 0;

/*
 * DroneCAN BatteryInfo (DTID 1092) receive callback — reassembles frames 1-2
 * to get temperature/voltage/current (see the header comment for the exact
 * wire layout and why this isn't just "read frame 1 directly").
 */
static void battery_info_cb(const CANRxFrame &f, void *ctx)
{
    (void)ctx;
    if (f.DLC < 8) return;

    const uint8_t  tail    = f.data8[7];
    const bool     sot     = (tail & 0x80U) != 0U;
    const bool     toggle  = (tail & 0x20U) != 0U;
    const uint8_t  tid     = tail & 0x1FU;
    const uint8_t  node_id = (uint8_t)(f.ext.EID & 0x7FU);

    if (sot) {
        // Frame 1: [CRC_lo, CRC_hi, temp_lo, temp_hi, volt_lo, volt_hi,
        // curr_lo, tail] -- bytes 0-1 are the transfer CRC, not payload.
        // Stash what we have and wait for frame 2 to complete current_A.
        // A fresh start_of_transfer always wins over whatever was pending
        // (self-heals from a dropped frame 2 on the very next publish,
        // ~100ms later at this module's rate -- no timeout needed).
        s_pending_valid    = true;
        s_pending_node_id  = node_id;
        s_pending_tid      = tid;
        s_pending_toggle   = !toggle;   // frame 2 must show the flipped toggle
        s_pending_volt_raw = (uint16_t)f.data8[4] | ((uint16_t)f.data8[5] << 8);
        s_pending_curr_lo  = f.data8[6];
        return;
    }

    // Continuation frame -- only useful if it's frame 2 of the transfer
    // we're mid-reassembling (same node, same transfer ID, alternated
    // toggle); anything else is either an unrelated transfer's frame or a
    // stale/out-of-order one, drop it and keep waiting for the next SOT.
    if (!s_pending_valid || node_id != s_pending_node_id ||
        tid != s_pending_tid || toggle != s_pending_toggle) {
        return;
    }
    s_pending_valid = false;   // consume -- one-shot regardless of outcome below

    const uint16_t curr_raw = (uint16_t)s_pending_curr_lo | ((uint16_t)f.data8[0] << 8);

    chMtxLock(&power_mtx);
    g_power.voltage_V = f16_to_f32(s_pending_volt_raw);
    g_power.current_A = f16_to_f32(curr_raw);
    g_power.node_id   = node_id;
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
