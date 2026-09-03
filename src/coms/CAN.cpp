#include "src/coms/CAN.hpp"
#include "src/threads.hpp"
#include <cstring>

/*
 * FDCAN1/2 bit timing. Bus 1 (motors) and bus 2 (IMX5 IMU + Matek power
 * monitor) get SEPARATE configs as of 2026-09-02 -- see bus 1's comment
 * below for why. Both derive from PLL2Q = 80 MHz.
 *
 * CCCR_DAR (Disable Automatic Retransmission): without this, an unacked
 * frame (e.g. a motor that never answers) gets retried by the FDCAN
 * hardware immediately and indefinitely — not just slow, but monopolizing
 * the bus and racking up TEC until bus-off, discovered 2026-09-01 bringing
 * up the wheel motors. Every motor command here is already reissued every
 * control cycle, so a dropped frame is self-healing on the next tick; there
 * is no reason to let the hardware retry-storm a stale one instead. Applies
 * to both buses -- bus 2's devices are read-only from this firmware's
 * perspective (no commands sent), so DAR only matters here in that it keeps
 * the same NBTP-independent behavior consistent across both.
 */

/*
 * Bus 1 (motors) — DROPPED to 500 kbit/s from 1 Mbit/s 2026-09-02 while
 * chasing hip3/4's degraded CAN reply rate on real hardware (signal-
 * integrity issue, still being isolated) -- a slower bit rate gives the
 * receiver more timing margin against reflections/ringing from imperfect
 * termination. BRP doubled (5->10), TSEG1/TSEG2 left exactly as-is, so the
 * Tq/bit count (16) and sample point (87.5%) are UNCHANGED -- only the bit
 * period itself doubles. Previous 1 Mbit/s value was 0x00040C01, if
 * reverting.
 *
 * SJW (resync jump width) also DOUBLED (1->2 Tq) same day, purely a noise-
 * tolerance move: SJW bounds how far the receiver can shift its bit-sample
 * timing per edge to resync with the transmitter, so a larger SJW means
 * more tolerance for jitter/phase-shift induced by reflections/noise on
 * imperfect wiring, at the cost of needing a larger jump to still be wrong
 * (a real edge-timing consideration, not the same axis as sample-point
 * position). This is a LOCAL, receive-side-only parameter -- unlike bit
 * rate, it does NOT need to match other nodes on the bus, so this change
 * carries none of the "every node must agree" risk the 500 kbit/s drop
 * did. Capped at min(NTSEG1, NTSEG2) = NTSEG2 = 2 here (already at that
 * cap) -- can't be raised further without also raising NTSEG2.
 *
 * BRP=10, TSEG1=13, TSEG2=2, SJW=2 → 16 Tq/bit, 87.5% sample point.
 * NBTP: NSJW=1, NBRP=9, NTSEG1=12, NTSEG2=1.
 *
 * IMPORTANT: every node on bus 1 must agree on the BIT RATE -- each RMD
 * hip drive's own CAN baud setting (its GUI tool) and each ODrive's
 * config.can.baud_rate must ALSO be changed to 500 kbit/s, or that node
 * goes completely silent (not just degraded) instead of matching. SJW
 * does not need to match (see above).
 */
static const CANConfig can_cfg_bus1 = {
    0x02090C01,          // NBTP -- 500 kbit/s, SJW=2 Tq
    0x00000000,          // DBTP
    FDCAN_CCCR_DAR,       // CCCR
    0x00000000,          // TEST
    0x00000000,          // RXGFC: accept all in RxFIFO0
};

/*
 * Bus 2 (IMX5 INS IMU + Matek CAN-L4-BM power monitor) — bit rate left at
 * 1 Mbit/s, UNCHANGED. Neither device is part of the bus-1 hip/wheel
 * wiring problem this session is chasing, and dropping their rate without
 * also reconfiguring both devices independently (IMX5's own config tool,
 * the Matek module's DroneCAN GUI tool) would silence bus 2 entirely
 * rather than just degrade it -- not worth the risk for an unrelated bus.
 *
 * SJW doubled (1->2 Tq) same as bus 1, same day, for the same reason (more
 * tolerance for noise/reflection-induced timing jitter) -- this one IS
 * risk-free to apply here too since it's a local, receive-side-only
 * parameter that doesn't need to match other nodes (see bus 1's comment).
 *
 * BRP=5, TSEG1=13, TSEG2=2, SJW=2 → 16 Tq/bit, 87.5% sample point.
 */
static const CANConfig can_cfg_bus2 = {
    0x02040C01,          // NBTP -- 1 Mbit/s, SJW=2 Tq
    0x00000000,          // DBTP
    FDCAN_CCCR_DAR,       // CCCR
    0x00000000,          // TEST
    0x00000000,          // RXGFC: accept all in RxFIFO0
};

/* ── Per-bus device tables ───────────────────────────────────────────────── */

struct CANDevice {
    uint32_t    id;       // SID (mask==0) or EID pattern (mask!=0)
    uint32_t    mask;     // 0 = exact standard-frame SID match; else masked extended-frame match
    CANCallback callback;
    void       *ctx;
};

static CANDevice can_table[2][MAX_CAN_DEVICES];
static int       num_can_devices[2] = {0, 0};

static volatile CANDiag s_diag[2] = {};

void bprl_can_register(CanBus bus, uint32_t sid, CANCallback cb, void *ctx)
{
    int b = (int)bus;
    if (num_can_devices[b] < MAX_CAN_DEVICES)
        can_table[b][num_can_devices[b]++] = {sid, 0U, cb, ctx};
}

void bprl_can_register_ext(CanBus bus, uint32_t eid, uint32_t mask,
                            CANCallback cb, void *ctx)
{
    int b = (int)bus;
    if (num_can_devices[b] < MAX_CAN_DEVICES)
        can_table[b][num_can_devices[b]++] = {eid, mask, cb, ctx};
}

/* ── ID scanner (bus 1 only) ─────────────────────────────────────────────── */

static volatile bool s_scan_active = false;
static CANScanEntry  s_scan[CAN_SCAN_MAX];
static int           s_scan_n = 0;

void can_scan_start(void)
{
    s_scan_active = false;
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

/* ── Frame dispatch ──────────────────────────────────────────────────────── */

void can_dispatch(CanBus bus, const CANRxFrame &frame)
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

    if (b == 0 && s_scan_active) {
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

    bool matched = false;
    for (int i = 0; i < num_can_devices[b]; i++) {
        const CANDevice &dev = can_table[b][i];
        bool hit;
        if (dev.mask == 0U) {
            // Exact standard-frame SID match (ignores extended frames)
            hit = (!is_ext) && (dev.id == match);
        } else {
            // Masked extended-frame EID match
            hit = is_ext && ((match & dev.mask) == (dev.id & dev.mask));
        }
        if (hit) {
            dev.callback(frame, dev.ctx);
            matched = true;
        }
    }
    if (matched) s_diag[b].dispatched++;
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
    bool ok = canTransmitTimeout(drv, CAN_ANY_MAILBOX, &txf,
                                 TIME_MS2I(timeout_ms)) == MSG_OK;
    if (ok) s_diag[(int)bus].tx_ok++; else s_diag[(int)bus].tx_fail++;
    return ok;
}

bool can_send_rtr(CanBus bus, uint32_t sid, uint8_t dlc, uint32_t timeout_ms)
{
    CANDriver *drv = (bus == CAN_BUS_1) ? &CAND1 : &CAND2;
    CANTxFrame txf = {};
    txf.common.XTD = 0;
    txf.common.RTR = 1;
    txf.std.SID    = sid;
    txf.DLC        = dlc;
    bool ok = canTransmitTimeout(drv, CAN_ANY_MAILBOX, &txf,
                                 TIME_MS2I(timeout_ms)) == MSG_OK;
    if (ok) s_diag[(int)bus].tx_ok++; else s_diag[(int)bus].tx_fail++;
    return ok;
}

/* ── Bus-off detection/recovery + error-counter diagnostics ─────────────── */

// Rising-edge trackers for the two "was this already true last time we
// looked" conditions below -- PSR.BO clears on its own once recovery
// completes, and RXF0S.RF0L is the M_CAN core's own "a message was lost"
// flag (cleared once a subsequent message is stored successfully); without
// edge-detecting both, a condition that stays set across several
// back-to-back polls (this runs every CANThread loop iteration) would
// inflate the count far past "how many distinct events happened."
static bool s_was_busoff[2]   = { false, false };
static bool s_was_fifo_lost[2] = { false, false };

void can_check_busoff(CanBus bus)
{
    int b = (int)bus;
    CANDriver *drv = (bus == CAN_BUS_1) ? &CAND1 : &CAND2;
    const uint32_t psr  = drv->fdcan->PSR;
    const uint32_t ecr  = drv->fdcan->ECR;
    const uint32_t rxf0 = drv->fdcan->RXF0S;

    const bool is_busoff   = (psr & FDCAN_PSR_BO)      != 0U;
    const bool fifo_lost   = (rxf0 & FDCAN_RXF0S_RF0L) != 0U;
    const uint8_t tec = (uint8_t)((ecr & FDCAN_ECR_TEC_Msk) >> FDCAN_ECR_TEC_Pos);
    const uint8_t rec = (uint8_t)((ecr & FDCAN_ECR_REC_Msk) >> FDCAN_ECR_REC_Pos);

    s_diag[b].tec = tec;
    s_diag[b].rec = rec;
    if (tec > s_diag[b].tec_peak) s_diag[b].tec_peak = tec;
    if (rec > s_diag[b].rec_peak) s_diag[b].rec_peak = rec;

    if (is_busoff && !s_was_busoff[b]) s_diag[b].busoff_count++;
    s_was_busoff[b] = is_busoff;

    if (fifo_lost && !s_was_fifo_lost[b]) s_diag[b].rx_fifo_lost++;
    s_was_fifo_lost[b] = fifo_lost;

    if (is_busoff) {
        drv->fdcan->CCCR &= ~FDCAN_CCCR_INIT;
    }
}

void can_get_diag(CanBus bus, CANDiag &out)
{
    int b = (int)bus;
    out.total_rx   = s_diag[b].total_rx;
    out.dispatched = s_diag[b].dispatched;
    out.last_sid   = s_diag[b].last_sid;
    out.last_eid   = s_diag[b].last_eid;
    out.last_eff   = s_diag[b].last_eff;
    out.last_dlc   = s_diag[b].last_dlc;
    for (int i = 0; i < 8; i++) out.last_data[i] = s_diag[b].last_data[i];
    out.tx_ok      = s_diag[b].tx_ok;
    out.tx_fail    = s_diag[b].tx_fail;
    out.tec          = s_diag[b].tec;
    out.rec          = s_diag[b].rec;
    out.tec_peak     = s_diag[b].tec_peak;
    out.rec_peak     = s_diag[b].rec_peak;
    out.busoff_count = s_diag[b].busoff_count;
    out.rx_fifo_lost = s_diag[b].rx_fifo_lost;
}

/* ── IMX5 INS IMU callbacks (bus 2) ─────────────────────────────────────── */

static inline int16_t le16s(const uint8_t *p)
{
    int16_t v;
    __builtin_memcpy(&v, p, sizeof(v));
    return v;
}

static void imx5_can_cb(const CANRxFrame &f, void *ctx)
{
    (void)ctx;
    chMtxLock(&can_imu_mtx);
    switch (f.std.SID) {
    case 0x01:
        g_can_imu.q0 = le16s(&f.data8[0]) * (1.0f / 10000.0f);
        g_can_imu.q1 = le16s(&f.data8[2]) * (1.0f / 10000.0f);
        g_can_imu.q2 = le16s(&f.data8[4]) * (1.0f / 10000.0f);
        g_can_imu.q3 = le16s(&f.data8[6]) * (1.0f / 10000.0f);
        g_can_imu.has_new_quat = true;
        g_can_imu.valid        = true;
        break;
    case 0x02:
        g_can_imu.p  = le16s(&f.data8[0]) * (1.0f / 1000.0f);
        g_can_imu.ax = le16s(&f.data8[2]) * (1.0f / 100.0f);
        g_can_imu.has_new_rates = true;
        g_can_imu.valid         = true;
        break;
    case 0x03:
        g_can_imu.q  = le16s(&f.data8[0]) * (1.0f / 1000.0f);
        g_can_imu.ay = le16s(&f.data8[2]) * (1.0f / 100.0f);
        break;
    case 0x04:
        g_can_imu.r  = le16s(&f.data8[0]) * (1.0f / 1000.0f);
        g_can_imu.az = le16s(&f.data8[2]) * (1.0f / 100.0f);
        break;
    default: break;
    }
    chMtxUnlock(&can_imu_mtx);
}

/* ── Register dump (FDCAN1) ─────────────────────────────────────────────── */

int can_read_regs(CANRegEntry *out, int max)
{
    if (!out || max < 1) return 0;
    auto *f = CAND1.fdcan;
    static const struct { const char *n; uint32_t off; } regs[] = {
        { "CCCR",  0x018 },
        { "NBTP",  0x01C },
        { "RXGFC", 0x080 },
        { "RXF0C", 0x0A0 },
        { "RXF0S", 0x0A4 },
        { "RXESC", 0x1BC },
        { "PSR",   0x044 },
        { "ECR",   0x040 },
    };
    int n = 0;
    const uint8_t *base = reinterpret_cast<const uint8_t *>(f);
    for (auto &r : regs) {
        if (n >= max) break;
        uint32_t v;
        __builtin_memcpy(&v, base + r.off, 4);
        out[n++] = { r.n, v };
    }
    return n;
}

/* ── Initialisation ──────────────────────────────────────────────────────── */

void can_drv_init(void)
{
    canStart(&CAND1, &can_cfg_bus1);
    canStart(&CAND2, &can_cfg_bus2);

    // IMX5 INS IMU on bus 2 (standard 11-bit IDs 0x01–0x04)
    bprl_can_register(CAN_BUS_2, 0x01, imx5_can_cb, nullptr);
    bprl_can_register(CAN_BUS_2, 0x02, imx5_can_cb, nullptr);
    bprl_can_register(CAN_BUS_2, 0x03, imx5_can_cb, nullptr);
    bprl_can_register(CAN_BUS_2, 0x04, imx5_can_cb, nullptr);
}
