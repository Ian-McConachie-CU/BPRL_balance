#include "src/coms/CAN.hpp"
#include "src/threads.hpp"
#include <cstring>

/*
 * FDCAN1/2 bit timing — 1 Mbit/s, PLL2Q = 80 MHz.
 * BRP=5, TSEG1=13, TSEG2=2 → 16 Tq/bit, 87.5% sample point.
 * NBTP: NSJW=0, NBRP=4, NTSEG1=12, NTSEG2=1.
 *
 * CCCR_DAR (Disable Automatic Retransmission): without this, an unacked
 * frame (e.g. a motor that never answers) gets retried by the FDCAN
 * hardware immediately and indefinitely — not just slow, but monopolizing
 * the bus and racking up TEC until bus-off, discovered 2026-09-01 bringing
 * up the wheel motors. Every motor command here is already reissued every
 * control cycle, so a dropped frame is self-healing on the next tick; there
 * is no reason to let the hardware retry-storm a stale one instead.
 */
static const CANConfig can_cfg = {
    0x00040C01,          // NBTP
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
    return canTransmitTimeout(drv, CAN_ANY_MAILBOX, &txf,
                              TIME_MS2I(timeout_ms)) == MSG_OK;
}

/* ── Bus-off detection/recovery ─────────────────────────────────────────── */

void can_check_busoff(CanBus bus)
{
    CANDriver *drv = (bus == CAN_BUS_1) ? &CAND1 : &CAND2;
    if ((drv->fdcan->PSR & FDCAN_PSR_BO) != 0U) {
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
    canStart(&CAND1, &can_cfg);
    canStart(&CAND2, &can_cfg);

    // IMX5 INS IMU on bus 2 (standard 11-bit IDs 0x01–0x04)
    bprl_can_register(CAN_BUS_2, 0x01, imx5_can_cb, nullptr);
    bprl_can_register(CAN_BUS_2, 0x02, imx5_can_cb, nullptr);
    bprl_can_register(CAN_BUS_2, 0x03, imx5_can_cb, nullptr);
    bprl_can_register(CAN_BUS_2, 0x04, imx5_can_cb, nullptr);
}
