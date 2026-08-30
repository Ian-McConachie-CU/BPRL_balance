#pragma once
#include "ch.h"
#include "hal.h"

/*
 * Motor_tool CAN driver — FDCAN1 + FDCAN2, both 1 Mbit/s, runtime-selectable.
 *
 * This is a standalone, trimmed copy of BPRL_balance's src/coms/CAN.* — it
 * does not know about IMUs, DroneCAN, or the fixed 6-motor layout. It exists
 * purely to move raw frames on/off the bus for the RMD and GIM drivers and
 * the sniffer/scanner below.
 */

typedef enum { CAN_BUS_1 = 0, CAN_BUS_2 = 1 } CanBus;

struct CANDiag {
    uint32_t total_rx;
    uint32_t last_sid;
    uint32_t last_eid;
    uint8_t  last_eff;
    uint8_t  last_dlc;
    uint8_t  last_data[8];
};

// Start both FDCAN1 and FDCAN2 at 1 Mbit/s. Call once at boot.
void can_drv_init(void);

// Send a standard-frame (11-bit SID) message on the given bus.
bool can_send(CanBus bus, uint32_t sid, const uint8_t *data, uint8_t dlc,
              uint32_t timeout_ms = 5);

// Per-bus diagnostic counters (last frame seen, total count).
void can_get_diag(CanBus bus, CANDiag &out);

/* ── Raw-frame subscribers ────────────────────────────────────────────────
 * The RX thread calls every registered subscriber for every frame received
 * on either bus (the subscriber itself filters by bus/ID). Used by the
 * RMD and GIM drivers to decode feedback without a fixed device table. */
typedef void (*CanRawCallback)(CanBus bus, const CANRxFrame &frame, void *ctx);
void can_subscribe(CanRawCallback cb, void *ctx);

/* ── ID scanner (diagnostic) ──────────────────────────────────────────────
 * Counts distinct arbitration IDs seen on a bus over a window — the first
 * thing to run against an unknown motor to find out what it actually
 * transmits, independent of any protocol assumption. */
#define CAN_SCAN_MAX 64
struct CANScanEntry {
    uint32_t id;
    uint32_t count;
    uint8_t  is_ext;
};
void can_scan_start(CanBus bus);
void can_scan_stop(void);
int  can_scan_get(CANScanEntry *out, int max);

/* ── Live monitor (candump-style) ─────────────────────────────────────────
 * While active, every received frame (both buses) is pushed into a ring
 * buffer for the command shell to drain and print over USB. Overwrites
 * oldest on overflow rather than blocking the RX thread. */
struct CanMonFrame {
    uint32_t t_ms;
    uint8_t  bus;
    uint32_t id;
    uint8_t  is_ext;
    uint8_t  dlc;
    uint8_t  data[8];
};
void can_monitor_start(void);
void can_monitor_stop(void);
bool can_monitor_active(void);
bool can_monitor_pop(CanMonFrame &out);   // non-blocking; false if empty

/* ── Internal loopback self-test ──────────────────────────────────────────
 * Reconfigures the given bus into internal loopback + bus-monitoring mode
 * (FDCAN_CCCR_TEST|MON, FDCAN_TEST_LBCK) — a transmitted frame is routed
 * back to the RX side entirely inside the STM32 FDCAN core, with no
 * dependency on external wiring, a transceiver, termination, or any other
 * live node. Sends one test frame and waits up to ~100ms to see it echo
 * back through the normal RX/dispatch path, then restores normal operation
 * on that bus. A pass here proves the MCU peripheral and this tool's driver
 * code are both working; a fail points at something downstream of the pins
 * (transceiver, wiring, power) or at a peripheral/clock config problem. */
struct CanSelftestResult {
    bool     tx_accepted;      // canTransmitTimeout() returned success
    bool     rx_matched;       // loopback frame decoded back correctly
    uint32_t rx_total_before;  // this bus's total_rx counter before the test
    uint32_t rx_total_after;   // ...and after — moving without rx_matched means
                                // *something* came back but didn't decode as expected
};
bool can_selftest(CanBus bus, CanSelftestResult *detail = nullptr);
