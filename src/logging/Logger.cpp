/*
 * Logger.cpp — ring-buffered binary SD card logger implementation.
 *
 * DMA coherency:  The STM32H7 SDMMC IDMA bypasses the CPU D-cache.
 * Any buffer passed to disk_write() must reside in non-cacheable memory.
 * s_fs, s_file, and s_flush_buf are placed in the .nocache section
 * (SRAM3, 0x30040000) which the MPU marks non-cacheable when
 * STM32_NOCACHE_ENABLE is TRUE in mcuconf.h.
 *
 * The ring buffer _buf[] stays in normal cached AXI SRAM — it is copied
 * into s_flush_buf (non-cached) by ring_read() before each f_write call.
 */

#include "src/logging/Logger.hpp"
#include "src/logging/LogMessages.hpp"
#include "ch.h"
#include "hal.h"
#include "ff.h"
#include <cstring>
#include <cstdio>

/* ── DMA-safe (non-cacheable) FatFS structures ──────────────────────────── */

static constexpr size_t FLUSH_CHUNK = 4096U;

/* Placed in .nocache (SRAM3) so SDMMC IDMA sees coherent data. */
static FATFS   __attribute__((section(".nocache"))) s_fs;
static FIL     __attribute__((section(".nocache"))) s_file;
static uint8_t __attribute__((section(".nocache"))) s_flush_buf[FLUSH_CHUNK];

/* ── Logger global instance ──────────────────────────────────────────────── */
Logger logger;

/* ── Board-level SDC hooks ───────────────────────────────────────────────
 * The Cube FMU microSD socket has no CD/WP signals routed to the MCU.
 * Report "always inserted / not write-protected"; sdcConnect() handles
 * actual card presence via the command-response handshake.            */
extern "C" {
bool sdc_lld_is_card_inserted(SDCDriver *sdcp) { (void)sdcp; return true; }
bool sdc_lld_is_write_protected(SDCDriver *sdcp) { (void)sdcp; return false; }

}

/* ── SDC configuration: 4-bit bus ───────────────────────────────────────── */
static const SDCConfig sdc_cfg = {
    SDC_MODE_4BIT,
    0U
};

/* ═══════════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

bool Logger::init()
{
    _head       = 0;
    _tail       = 0;
    _open       = false;
    _sync_count = 0;

    if (sdcStart(&SDCD1, &sdc_cfg) != MSG_OK) {
        _last_init_err = 1;
        sdcStop(&SDCD1);
        return false;
    }
    if (sdcConnect(&SDCD1) != HAL_SUCCESS) {
        _last_init_err = 2;
        sdcStop(&SDCD1);
        return false;
    }
    {
        FRESULT fr = f_mount(&s_fs, "/", 1);
        if (fr != FR_OK) {
            _last_init_err = 3;
            _last_ff_err   = (uint8_t)fr;
            sdcDisconnect(&SDCD1);
            sdcStop(&SDCD1);
            return false;
        }
    }

    f_mkdir("/LOGS");  // ignore error if directory already exists

    char path[32];
    find_next_log_index(path, sizeof(path));

    if (f_open(&s_file, path, FA_WRITE | FA_CREATE_NEW) != FR_OK) {
        _last_init_err = 4;
        f_mount(nullptr, "/", 0);
        sdcDisconnect(&SDCD1);
        sdcStop(&SDCD1);
        return false;
    }

    _last_init_err = 5;
    _open = true;
    strncpy(_current_path, path, sizeof(_current_path) - 1);
    write_schema_header();
    return true;
}

bool Logger::is_ready() const
{
    return _open;
}

void Logger::flush()
{
    if (!_open) return;

    const size_t n = ring_read(s_flush_buf, FLUSH_CHUNK);
    if (n == 0) return;

    UINT written;
    if (f_write(&s_file, s_flush_buf, (UINT)n, &written) != FR_OK) {
        _open = false;  // SD error — stop logging
        return;
    }

    if (++_sync_count >= SYNC_INTERVAL) {
        f_sync(&s_file);
        _sync_count = 0;
    }
}

void Logger::close()
{
    if (!_open) return;
    flush();
    f_sync(&s_file);
    f_close(&s_file);
    f_mount(nullptr, "/", 0);
    sdcDisconnect(&SDCD1);
    sdcStop(&SDCD1);
    _open = false;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Private helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

bool Logger::ring_write(const void *data, size_t n)
{
    const uint8_t *src = static_cast<const uint8_t *>(data);

    /* Check available space (keep one slot empty to distinguish full/empty). */
    size_t free_space;
    if (_head >= _tail) {
        free_space = BUF_SIZE - _head + _tail - 1U;
    } else {
        free_space = _tail - _head - 1U;
    }
    if (n > free_space) return false;  // drop entire record — never partial write

    if (_head + n <= BUF_SIZE) {
        memcpy(_buf + _head, src, n);
        _head = (_head + n) % BUF_SIZE;
    } else {
        const size_t first = BUF_SIZE - _head;
        memcpy(_buf + _head, src, first);
        memcpy(_buf, src + first, n - first);
        _head = n - first;
    }
    return true;
}

size_t Logger::ring_read(void *out, size_t max_n)
{
    uint8_t *dst = static_cast<uint8_t *>(out);

    size_t avail;
    if (_head >= _tail) {
        avail = _head - _tail;
    } else {
        avail = BUF_SIZE - _tail + _head;
    }
    const size_t n = (avail < max_n) ? avail : max_n;
    if (n == 0) return 0;

    if (_tail + n <= BUF_SIZE) {
        memcpy(dst, _buf + _tail, n);
        _tail = (_tail + n) % BUF_SIZE;
    } else {
        const size_t first = BUF_SIZE - _tail;
        memcpy(dst, _buf + _tail, first);
        memcpy(dst + first, _buf, n - first);
        _tail = n - first;
    }
    return n;
}

/* ── Schema header: one FMT record per log type (ArduPilot standard) ────── */

struct __attribute__((packed)) LogFmtHdr {
    uint8_t sync1;      // 0xA3
    uint8_t sync2;      // 0x95
    uint8_t fmt_id;     // LOG_MSG_FMT = 0x80
    uint8_t type;       // msg_id being described
    uint8_t length;     // total data record size INCLUDING 3-byte header = 3 + body_size
    char    name[4];    // short type name, space-padded
    char    format[16]; // ArduPilot format codes: Q=uint64 H=uint16 f=float i=int32 h=int16 B=uint8
    char    labels[64]; // comma-separated field names; TimeUS must be first for UAV Log Viewer
};
// Total FMT record size: 89 bytes

bool Logger::write_schema_header()
{
    for (size_t i = 0; i < kNumLogDefs; i++) {
        LogFmtHdr hdr = {};
        hdr.sync1  = 0xA3U;
        hdr.sync2  = 0x95U;
        hdr.fmt_id = LOG_MSG_FMT;
        hdr.type   = kLogDefs[i].msg_id;
        hdr.length = (uint8_t)(3U + kLogDefs[i].body_size);
        strncpy(hdr.name,   kLogDefs[i].name,   sizeof(hdr.name));
        strncpy(hdr.format, kLogDefs[i].fmt,    sizeof(hdr.format) - 1U);
        strncpy(hdr.labels, kLogDefs[i].labels, sizeof(hdr.labels) - 1U);
        ring_write(&hdr, sizeof(hdr));
    }
    return true;
}

/* ── Auto-incremented log file naming: /LOGS/LOG0001.BIN … LOG9999.BIN ── */

void Logger::find_next_log_index(char *path, size_t path_size)
{
    for (uint32_t n = 1U; n <= 9999U; n++) {
        snprintf(path, path_size, "/LOGS/LOG%04u.BIN", (unsigned)n);
        FILINFO fi;
        if (f_stat(path, &fi) != FR_OK) {
            return;  // this slot is free
        }
    }
    snprintf(path, path_size, "/LOGS/LOG9999.BIN");  // all slots used
}
