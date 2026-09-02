#pragma once
#include "hal.h"

/*
 * CAN bus driver — FDCAN1 (bus 1) and FDCAN2 (bus 2), both at 1 Mbit/s.
 *
 * Bus assignment:
 *   CAN_BUS_1 (FDCAN1) — 6 CAN motors: RMD hip × 4 (IDs 1–4), ODrive wheel × 2 (IDs 5–6)
 *   CAN_BUS_2 (FDCAN2) — Inertial Sense IMX5 INS (std IDs 0x01–0x04)
 *                       + Matek CAN-L4-BM power monitor (DroneCAN BatteryInfo, DTID 1092)
 *
 * Adding a standard-frame device (11-bit SID, exact match):
 *   bprl_can_register(bus, sid, my_cb, ctx);
 *
 * Adding a DroneCAN/extended-frame device (29-bit EID, masked match):
 *   bprl_can_register_ext(bus, eid_pattern, mask, my_cb, ctx);
 *   // e.g. DTID 1092 on any node/priority: eid=0x00044400, mask=0x00FFFF80
 */

typedef enum { CAN_BUS_1 = 0, CAN_BUS_2 = 1 } CanBus;

#define MAX_CAN_DEVICES 16   // per bus
typedef void (*CANCallback)(const CANRxFrame &frame, void *ctx);

struct CANDiag {
    uint32_t total_rx;     // total frames received (any ID)
    uint32_t dispatched;   // frames that matched a registered callback
    uint32_t last_sid;
    uint32_t last_eid;
    uint8_t  last_eff;
    uint8_t  last_dlc;
    uint8_t  last_data[8];
};

// Register a handler for an 11-bit standard CAN ID (exact SID match).
void bprl_can_register(CanBus bus, uint32_t sid, CANCallback cb, void *ctx);

// Register a handler for a 29-bit extended CAN ID with masking.
// A frame matches when (frame.ext.EID & mask) == (eid & mask).
// Use for DroneCAN: eid = DTID<<8, mask = 0x00FFFF80 matches any node/priority.
void bprl_can_register_ext(CanBus bus, uint32_t eid, uint32_t mask,
                            CANCallback cb, void *ctx);

// Dispatch one received frame to its registered handler (called by CANThread).
void can_dispatch(CanBus bus, const CANRxFrame &frame);

// Detect a latched bus-off condition (PSR.BO) and clear it by re-triggering
// the M_CAN bus-off recovery sequence (drops CCCR.INIT; the peripheral then
// auto-monitors for 128x11 consecutive recessive bits before rejoining, per
// ISO 11898-1 -- no other register writes needed). ChibiOS's FDCAN LLD never
// enables/handles the bus-off interrupt itself, so without this a bus-off
// event (e.g. from a long run of unacked frames) leaves the peripheral
// silent forever. No-op (one register read) when not in bus-off -- cheap
// enough to poll every iteration of whichever thread owns this bus's RX.
void can_check_busoff(CanBus bus);

// Send a frame on the given bus (blocks up to timeout_ms milliseconds).
bool can_send(CanBus bus, uint32_t sid, const uint8_t *data, uint8_t dlc,
              uint32_t timeout_ms = 5);

// Copy out current diagnostic counters for a bus (safe to call from any thread).
void can_get_diag(CanBus bus, CANDiag &out);

// Start both FDCAN1 and FDCAN2 and register IMX5 callbacks on bus 2.
void can_drv_init(void);

// Read key FDCAN hardware registers into out[].  Returns number of entries.
struct CANRegEntry { const char *name; uint32_t value; };
int can_read_regs(CANRegEntry *out, int max);

// ── ID scanner (diagnostic, bus 1 only) ──────────────────────────────────
#define CAN_SCAN_MAX 48

struct CANScanEntry {
    uint32_t id;
    uint32_t count;
    uint8_t  is_ext;
};

void can_scan_start(void);
void can_scan_stop(void);
int  can_scan_get(CANScanEntry *out, int max);
