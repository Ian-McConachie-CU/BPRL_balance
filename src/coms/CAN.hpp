#pragma once
#include "hal.h"

/*
 * CAN bus driver — FDCAN1 (bus 1, motors) and FDCAN2 (bus 2, IMX5+Matek).
 * Bus 1 runs at 500 kbit/s (dropped from 1 Mbit/s 2026-09-02, see CAN.cpp's
 * can_cfg_bus1 comment); bus 2 is unchanged at 1 Mbit/s. Every node on bus 1
 * (each RMD hip drive, each ODrive) must be reconfigured to match 500 kbit/s
 * too, or it goes silent rather than degraded.
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
    uint32_t tx_ok;        // can_send()/can_send_rtr() calls that returned true
    uint32_t tx_fail;      // ...that returned false (canTransmitTimeout() != MSG_OK,
                            // e.g. no free mailbox within timeout_ms) -- nonzero here
                            // means the STM32 itself isn't getting frames onto the
                            // wire, as distinct from a frame going out cleanly but
                            // getting no reply.
    // Hardware error-counter diagnostics -- quantify bus-level noise/errors
    // directly (bit/stuff/form errors, missing ACKs: anything that
    // increments the M_CAN core's own TEC/REC), independent of and
    // complementary to tx_ok/tx_fail/rx_count above and in CANMotor.cpp --
    // those can only tell you a given motor's commands/replies aren't
    // getting through; these tell you whether the BUS ITSELF is seeing
    // errors, which is what actually answers "is this bus noisy" rather
    // than "is this one motor responding." Sampled every can_check_busoff()
    // call (CANThread, every loop iteration) -- see CAN.cpp.
    uint8_t  tec;          // current Transmit Error Counter (ECR.TEC, 0-255; 128+ = error-passive)
    uint8_t  rec;          // current Receive Error Counter (ECR.REC, 0-127)
    uint8_t  tec_peak;     // highest TEC observed since boot
    uint8_t  rec_peak;     // highest REC observed since boot
    uint32_t busoff_count; // number of times PSR.BO was observed set and recovery re-triggered
    uint32_t rx_fifo_lost; // number of times RXF0S.RF0L (FIFO0 message lost) was newly observed set
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

// Send a remote-transmission-request (RTR) frame -- no data payload, just
// asks whatever device owns `sid` to reply with its data frame immediately.
// Used to actively poll ODrive's Get_Encoder_Estimates instead of waiting
// on its own periodic broadcast -- see CANMotor.cpp. `dlc` communicates the
// expected reply length per convention; an RTR frame carries no data bytes
// on the wire regardless of its value.
bool can_send_rtr(CanBus bus, uint32_t sid, uint8_t dlc, uint32_t timeout_ms = 5);

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
