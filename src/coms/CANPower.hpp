#pragma once
#include "hal.h"

/*
 * Matek AP_Periph CAN-L4-BM power monitor — DroneCAN (UAVCAN v0).
 *
 * Device broadcasts uavcan.equipment.power.BatteryInfo (DTID 1092) on bus 2.
 * We decode the first CAN frame of each transfer to get voltage and current:
 *
 *   payload[0..1]: temperature (float16, K)  — not stored
 *   payload[2..3]: voltage     (float16, V)  → g_power.voltage_V
 *   payload[4..5]: current     (float16, A)  → g_power.current_A
 *   payload[7]:    UAVCAN tail byte (start_of_transfer | end_of_transfer | toggle | tid)
 *
 * Registration: masked extended-frame match on DTID bits [23:8] = 0x0444.
 *   eid = 0x00044400, mask = 0x00FFFF80 → matches any node ID and any priority.
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
