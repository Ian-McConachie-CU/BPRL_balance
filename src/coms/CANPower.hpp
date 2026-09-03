#pragma once
#include "hal.h"

/*
 * Matek AP_Periph CAN-L4-BM power monitor — DroneCAN (UAVCAN v0).
 *
 * Device broadcasts uavcan.equipment.power.BatteryInfo (DTID 1092) on bus 2.
 * BatteryInfo is always a multi-frame transfer (payload > 7 bytes) — we
 * reassemble frames 1-2 to get temperature/voltage/current, ignoring the
 * rest (capacity, state-of-charge, model name, ...).
 *
 * Wire layout, CORRECTED 2026-09-02: an earlier version of this decoder
 * treated frame 1's data bytes as if they were payload bytes 0-6 directly,
 * which is wrong -- DroneCAN prepends a 2-byte little-endian transfer CRC
 * to frame 1, BEFORE the actual payload. That off-by-2 meant the old
 * "voltage_V" was actually decoding temperature, "current_A" was actually
 * decoding voltage, and real current (which spans the frame1/frame2
 * boundary) was never read at all. Verified against
 * ~/Documents/ardupilot's DroneCAN source (this module ships stock
 * AP_Periph firmware) and the public DroneCAN spec.
 *
 * Frame 1 (start_of_transfer=1): [CRC_lo, CRC_hi, temp_lo, temp_hi,
 *   volt_lo, volt_hi, curr_lo, tail]
 * Frame 2 (start_of_transfer=0, toggle bit flipped from frame 1, same
 *   transfer ID): [curr_hi, ...rest not needed..., tail]
 *
 * Tail byte (last byte of every frame): bit7=start_of_transfer,
 * bit6=end_of_transfer, bit5=toggle (0 on frame 1, alternates after),
 * bits4-0=transfer ID (constant across one transfer's frames).
 *
 * Registration: masked extended-frame match on DTID bits [23:8] = 0x0444.
 *   eid = 0x00044400, mask = 0x00FFFF80 → matches any node ID and any priority.
 *
 * Prerequisite: the module defaults to CAN_NODE=0 (DroneCAN dynamic node
 * allocation) and this firmware implements no DNA server -- it will not
 * transmit BatteryInfo at all until given a static, nonzero CAN_NODE via
 * the DroneCAN GUI Tool / Mission Planner (one-time device config, no
 * firmware change).
 */

struct PowerMonState {
    float   voltage_V;   // pack voltage [V]
    float   current_A;   // discharge current [A] (positive = discharge)
    uint8_t node_id;     // DroneCAN source node ID of last frame
    bool    valid;       // true once at least one valid frame received
};

extern mutex_t       power_mtx;
extern PowerMonState g_power;

// Call once after can_drv_init() in main.cpp.
void power_mon_init(void);
