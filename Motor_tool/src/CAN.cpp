#include "src/CAN.hpp"
#include <cstring>

/*
 * FDCAN1/2 bit timing — 1 Mbit/s, PLL2Q = 80 MHz.
 * BRP=5, TSEG1=13, TSEG2=2 -> 16 Tq/bit, 87.5% sample point.
 * Identical to BPRL_balance's src/coms/CAN.cpp — same MCU, same clock tree.
 *
 * CCCR_DAR (Disable Automatic Retransmission): without this, an unacked
 * frame (e.g. probing a CAN ID with nothing listening) gets retried by the
 * FDCAN hardware immediately and indefinitely instead of just failing once —
 * discovered 2026-09-01 bringing up the wheel motors, where a single
 * `poll wheels`/`poll gim` against an unacked ID monopolized the bus and
 * ran TEC up to bus-off. This tool's commands are one-shot requests, not a
 * periodically-reissued control loop, so there's no self-healing reason to
 * let the hardware retry-storm a stale one.
 */
static const CANConfig can_cfg = {
    0x00040C01,           // NBTP
    0x00000000,           // DBTP
    FDCAN_CCCR_DAR,        // CCCR
    0x00000000,           // TEST
    0x00000000,           // RXGFC: accept all in RxFIFO0
};

/* Internal loopback + bus-monitoring config for can_selftest() — same bit
 * timing as normal operation, but TEST.LBCK routes TX back to RX inside the
 * FDCAN core itself, and MON prevents it from trying to actually drive the
 * pins/expect a real bus ACK. No external hardware required. */
static const CANConfig can_cfg_loopback = {
    0x00040C01,                          // NBTP — same bit timing
    0x00000000,                          // DBTP
    FDCAN_CCCR_TEST | FDCAN_CCCR_MON,    // CCCR
    FDCAN_TEST_LBCK,                     // TEST
    0x00000000,                          // RXGFC: accept all in RxFIFO0
};

/* ── Direct RxFIFO0 polling ──────────────────────────────────────────────
 * Ported from BPRL_flight's src/coms/CAN.cpp (bprl_can_poll()) — hardware-
 * confirmed working on this exact board (CubeOrangePlus, FDCAN1) by
 * reading real IMX5 traffic. Reads RXF0S/message-RAM directly instead of
 * going through ChibiOS's canReceiveTimeout(), which depends on that
 * driver's own internal wait/wake bookkeeping — bus 2 proved that
 * bookkeeping works at least once, but rather than keep guessing at why
 * bus 1 might differ, this sidesteps it entirely for both buses.
 *
 * Element layout matches this project's hal_can_lld.c patch: RxFIFO0 for a
 * given instance starts at word offset 0 within canp->ram_base (no
 * acceptance filters ahead of it — see the SRAMCAN_* override comment
 * there), and each element is 18 words (2 header + up to 16 data,
 * RXESC=0x777). Every frame in this project is classical 8-byte CAN, so
 * only the first two data words are ever meaningful. */
static bool poll_fifo0(CANDriver *drv, CANRxFrame &out)
{
    uint32_t rxf0s = drv->fdcan->RXF0S;
    if ((rxf0s & FDCAN_RXF0S_F0FL_Msk) == 0) return false;   // empty

    uint32_t gi = (rxf0s & FDCAN_RXF0S_F0GI_Msk) >> FDCAN_RXF0S_F0GI_Pos;
    volatile uint32_t *elem = reinterpret_cast<volatile uint32_t *>(drv->ram_base) + gi * 18U;

    out.header32[0] = elem[0];
    out.header32[1] = elem[1];
    out.data32[0]   = elem[2];
    out.data32[1]   = elem[3];

    drv->fdcan->RXF0A = gi;   // acknowledge / free this slot
    return true;
}

// While a selftest is running on a bus, the RX thread steps aside from that
// bus entirely (see CANRxThread below) so the test can do its own dedicated
// canReceiveTimeout() on the calling thread without racing the RX thread for
// the same driver's mailbox — canStop()/canStart() while another thread has
// an outstanding receive call on that driver is not something to rely on.
static volatile bool      s_selftest_active = false;
static volatile CanBus    s_selftest_bus    = CAN_BUS_1;
static constexpr uint32_t SELFTEST_SID      = 0x555;

#define MAX_SUBSCRIBERS 8
struct Subscriber { CanRawCallback cb; void *ctx; };
static Subscriber s_subs[MAX_SUBSCRIBERS];
static int        s_num_subs = 0;

static volatile CANDiag s_diag[2] = {};

void can_subscribe(CanRawCallback cb, void *ctx)
{
    if (s_num_subs < MAX_SUBSCRIBERS)
        s_subs[s_num_subs++] = {cb, ctx};
}

/* ── ID scanner ──────────────────────────────────────────────────────────── */

static volatile bool s_scan_active = false;
static CanBus        s_scan_bus    = CAN_BUS_1;
static CANScanEntry  s_scan[CAN_SCAN_MAX];
static int           s_scan_n = 0;

void can_scan_start(CanBus bus)
{
    s_scan_active = false;
    s_scan_bus    = bus;
    s_scan_n      = 0;
    memset(s_scan, 0, sizeof(s_scan));
    s_scan_active = true;
}

void can_scan_stop(void) { s_scan_active = false; }

int can_scan_get(CANScanEntry *out, int max)
{
    int n = s_scan_n < max ? s_scan_n : max;
    memcpy(out, s_scan, (size_t)n * sizeof(CANScanEntry));
    return n;
}

/* ── Live monitor ────────────────────────────────────────────────────────── */

#define MON_RING_SIZE 64
static CanMonFrame       s_mon_ring[MON_RING_SIZE];
static volatile uint32_t s_mon_head = 0;   // next write slot
static volatile uint32_t s_mon_tail = 0;   // next read slot
static volatile bool     s_mon_active = false;

void can_monitor_start(void) { s_mon_head = s_mon_tail = 0; s_mon_active = true; }
void can_monitor_stop(void)  { s_mon_active = false; }
bool can_monitor_active(void) { return s_mon_active; }

bool can_monitor_pop(CanMonFrame &out)
{
    if (s_mon_tail == s_mon_head) return false;
    out = s_mon_ring[s_mon_tail % MON_RING_SIZE];
    s_mon_tail++;
    return true;
}

static void mon_push(CanBus bus, const CANRxFrame &f)
{
    CanMonFrame &m = s_mon_ring[s_mon_head % MON_RING_SIZE];
    m.t_ms  = (uint32_t)TIME_I2MS(chVTGetSystemTime());
    m.bus   = (uint8_t)bus;
    m.is_ext = f.common.XTD;
    m.id    = m.is_ext ? f.ext.EID : f.std.SID;
    m.dlc   = f.DLC;
    for (int i = 0; i < 8; i++) m.data[i] = f.data8[i];
    s_mon_head++;
    // Ring overwrite: if head laps tail, drop the oldest by advancing tail.
    if (s_mon_head - s_mon_tail > MON_RING_SIZE) s_mon_tail = s_mon_head - MON_RING_SIZE;
}

/* ── Frame dispatch ──────────────────────────────────────────────────────── */

static void can_dispatch(CanBus bus, const CANRxFrame &frame)
{
    int b = (int)bus;
    const bool     is_ext = frame.common.XTD;
    const uint32_t match  = is_ext ? frame.ext.EID : frame.std.SID;

    s_diag[b].total_rx++;
    s_diag[b].last_sid = frame.std.SID;
    s_diag[b].last_eff = is_ext ? 1U : 0U;
    s_diag[b].last_eid = is_ext ? frame.ext.EID : 0U;
    s_diag[b].last_dlc = frame.DLC;
    for (int i = 0; i < 8; i++) s_diag[b].last_data[i] = frame.data8[i];

    if (s_scan_active && bus == s_scan_bus) {
        bool found = false;
        for (int i = 0; i < s_scan_n && !found; i++) {
            if (s_scan[i].id == match && s_scan[i].is_ext == (uint8_t)is_ext) {
                s_scan[i].count++;
                found = true;
            }
        }
        if (!found && s_scan_n < CAN_SCAN_MAX)
            s_scan[s_scan_n++] = {match, 1, (uint8_t)is_ext};
    }

    if (s_mon_active) mon_push(bus, frame);

    for (int i = 0; i < s_num_subs; i++)
        s_subs[i].cb(bus, frame, s_subs[i].ctx);
}

/* ── Frame transmit ──────────────────────────────────────────────────────── */

bool can_send(CanBus bus, uint32_t sid, const uint8_t *data, uint8_t dlc,
              uint32_t timeout_ms)
{
    CANDriver *drv = (bus == CAN_BUS_1) ? &CAND1 : &CAND2;
    CANTxFrame txf = {};
    txf.common.XTD = 0;
    txf.common.RTR = 0;
    txf.std.SID    = sid;
    txf.DLC        = dlc;
    if (dlc > 8) dlc = 8;
    for (int i = 0; i < dlc; i++) txf.data8[i] = data[i];
    return canTransmitTimeout(drv, CAN_ANY_MAILBOX, &txf,
                              TIME_MS2I(timeout_ms)) == MSG_OK;
}

void can_get_diag(CanBus bus, CANDiag &out)
{
    int b = (int)bus;
    out.total_rx   = s_diag[b].total_rx;
    out.last_sid   = s_diag[b].last_sid;
    out.last_eid   = s_diag[b].last_eid;
    out.last_eff   = s_diag[b].last_eff;
    out.last_dlc   = s_diag[b].last_dlc;
    for (int i = 0; i < 8; i++) out.last_data[i] = s_diag[b].last_data[i];
}

/* ── Internal loopback self-test ─────────────────────────────────────────── */

bool can_selftest(CanBus bus, CanSelftestResult *detail)
{
    CANDriver *drv = (bus == CAN_BUS_1) ? &CAND1 : &CAND2;

    CANDiag before = {};
    can_get_diag(bus, before);

    // Tell the RX thread to stop polling this bus, and give it a moment to
    // actually notice (it re-checks the flag every loop iteration, which
    // runs at least every ~500us, so a few ms is generous).
    s_selftest_bus    = bus;
    s_selftest_active = true;
    chThdSleepMilliseconds(5);

    canStop(drv);
    canStart(drv, &can_cfg_loopback);

    uint8_t data[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x00};
    bool tx_ok = can_send(bus, SELFTEST_SID, data, 8, 20);

    bool rx_matched = false;
    for (int i = 0; i < 100 && !rx_matched; i++) {
        CANRxFrame rxf;
        if (poll_fifo0(drv, rxf)) {
            can_dispatch(bus, rxf);   // keeps diag/scan/monitor/subscribers consistent
            if (!rxf.common.XTD && rxf.std.SID == SELFTEST_SID) {
                rx_matched = true;
            }
        } else {
            chThdSleepMilliseconds(1);
        }
    }

    CANDiag after = {};
    can_get_diag(bus, after);

    canStop(drv);
    canStart(drv, &can_cfg);   // restore normal operation on this bus

    s_selftest_active = false;   // RX thread resumes polling this bus

    if (detail) {
        detail->tx_accepted     = tx_ok;
        detail->rx_matched      = rx_matched;
        detail->rx_total_before = before.total_rx;
        detail->rx_total_after  = after.total_rx;
    }
    return rx_matched;
}

/* ── Bus-off detection/recovery ─────────────────────────────────────────
 * ChibiOS's FDCAN LLD never enables/handles the bus-off interrupt (IR_BO) —
 * see hal_can_lld.c's can_lld_serve_interrupt(), which only wires up
 * RF0N/RF1N/RF0L/RF1L/TC. Once TEC latches past 255 (bus-off), the M_CAN
 * core auto-sets CCCR.INIT and goes silent forever unless something clears
 * INIT again to restart the standard ISO 11898-1 recovery sequence (128 x
 * 11 consecutive recessive bits, monitored entirely in hardware from there —
 * no other register writes needed). Polled cheaply (one register read when
 * healthy) from CANRxThread below rather than adding a new ISR.
 */
static void can_check_busoff(CanBus bus)
{
    CANDriver *drv = (bus == CAN_BUS_1) ? &CAND1 : &CAND2;
    if ((drv->fdcan->PSR & FDCAN_PSR_BO) != 0U) {
        drv->fdcan->CCCR &= ~FDCAN_CCCR_INIT;
    }
}

/* ── RX thread ───────────────────────────────────────────────────────────── */

static THD_WORKING_AREA(waCANRx, 2048);
static THD_FUNCTION(CANRxThread, arg)
{
    (void)arg;
    chRegSetThreadName("can_rx");
    while (true) {
        CANRxFrame rxf;
        bool any = false;
        // Step aside from whichever bus can_selftest() is currently
        // reconfiguring/reading itself — see the comment by s_selftest_active.
        if (!(s_selftest_active && s_selftest_bus == CAN_BUS_1)) {
            if (poll_fifo0(&CAND1, rxf)) { can_dispatch(CAN_BUS_1, rxf); any = true; }
            can_check_busoff(CAN_BUS_1);
        }
        if (!(s_selftest_active && s_selftest_bus == CAN_BUS_2)) {
            if (poll_fifo0(&CAND2, rxf)) { can_dispatch(CAN_BUS_2, rxf); any = true; }
            can_check_busoff(CAN_BUS_2);
        }
        // Pure polling now (see poll_fifo0 comment) — yield briefly when
        // idle so this doesn't spin at 100%; drain back-to-back without
        // sleeping when frames are actually arriving.
        if (!any) chThdSleepMicroseconds(100);
    }
}

void can_drv_init(void)
{
    canStart(&CAND1, &can_cfg);
    canStart(&CAND2, &can_cfg);
    chThdCreateStatic(waCANRx, sizeof(waCANRx), NORMALPRIO + 8, CANRxThread, nullptr);
}
