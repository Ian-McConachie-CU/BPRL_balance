#include "src/coms/CalFlash.hpp"
#include "hal.h"
#include <cstring>

/*
 * STM32H743 Bank 2, sector 7 flash driver.
 *
 * Bank 2 registers (FLASH_TypeDef, stm32h743xx.h):
 *   FLASH->KEYR2  – unlock key register
 *   FLASH->CR2    – control (PG, SER, SNB, PSIZE, FW, START, LOCK)
 *   FLASH->SR2    – status  (BSY, QW, WBNE, errors)
 *   FLASH->CCR2   – clear status
 *
 * Programming unit: 256 bits = 32 bytes = 8 × 32-bit words.
 * CalibData is 128 bytes → 4 programming cycles.
 */

static constexpr uint32_t CAL_ADDR    = 0x081E0000UL;
static constexpr uint32_t CAL_MAGIC   = 0x42525943UL;   // 'BPRC'
static constexpr uint32_t CAL_VERSION = 1U;

// Key sequence to unlock Bank 2 control register
static constexpr uint32_t KEY1 = 0x45670123UL;
static constexpr uint32_t KEY2 = 0xCDEF89ABUL;

// Error bits in SR2 (STRBERR | PGSERR | INCERR | OPERR | RDPERR | RDSERR | SNECCERR | DBECCERR)
static constexpr uint32_t SR_ERR_MASK = 0x7EEA0000UL;

/* ── CRC-32/ISO-HDLC (polynomial 0xEDB88320, init 0xFFFFFFFF, final XOR 0xFFFFFFFF) ── */
static uint32_t crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc >> 1) ^ (0xEDB88320UL & -(crc & 1U));
    }
    return crc ^ 0xFFFFFFFFUL;
}

/* ── Wait for Bank 2 busy flags to clear ── */
static void b2_wait()
{
    while (FLASH->SR2 & (FLASH_SR_BSY | FLASH_SR_QW | FLASH_SR_WBNE)) {}
}

/* ── Unlock Bank 2 ── */
static void b2_unlock()
{
    if (FLASH->CR2 & FLASH_CR_LOCK) {
        FLASH->KEYR2 = KEY1;
        FLASH->KEYR2 = KEY2;
    }
}

/* ── Lock Bank 2 ── */
static void b2_lock()
{
    FLASH->CR2 |= FLASH_CR_LOCK;
}

/* ── Clear Bank 2 error flags ── */
static void b2_clear_errors()
{
    FLASH->CCR2 = SR_ERR_MASK;
}

/* ── Erase Bank 2 sector 7 ── */
static bool b2_erase_sector7()
{
    b2_wait();
    b2_clear_errors();

    // SER=1, SNB=7, PSIZE=word (0b10)
    FLASH->CR2 = FLASH_CR_SER | (7U << FLASH_CR_SNB_Pos) | FLASH_CR_PSIZE_1;
    FLASH->CR2 |= FLASH_CR_START;
    b2_wait();

    bool ok = !(FLASH->SR2 & SR_ERR_MASK);
    FLASH->CR2 = 0;
    return ok;
}

/* ── Program one 32-byte flash word at |addr| from |src| (8 × uint32_t) ── */
static bool b2_program_word(uint32_t addr, const uint32_t *src)
{
    b2_wait();
    b2_clear_errors();

    FLASH->CR2 = FLASH_CR_PG | FLASH_CR_PSIZE_1;

    volatile uint32_t *dst = reinterpret_cast<volatile uint32_t *>(addr);
    for (int i = 0; i < 8; ++i)
        dst[i] = src[i];

    // FW forces write of incomplete buffer; safe to set after full 8-word write too
    FLASH->CR2 |= FLASH_CR_FW;
    b2_wait();

    bool ok = !(FLASH->SR2 & SR_ERR_MASK);
    FLASH->CR2 = 0;
    return ok;
}

/* ════════════════════════════════════════════════════════════════════════════
 * Public API
 * ════════════════════════════════════════════════════════════════════════════ */

bool cal_load(CalibData &out)
{
    memset(&out, 0, sizeof(out));

    const CalibData *flash = reinterpret_cast<const CalibData *>(CAL_ADDR);
    if (flash->magic   != CAL_MAGIC)   return false;
    if (flash->version != CAL_VERSION) return false;

    // CRC covers bytes [0 .. sizeof(CalibData) - 4 - 44) = [0 .. 80)
    constexpr size_t crc_len = sizeof(CalibData) - sizeof(uint32_t) - 44;
    uint32_t computed = crc32(reinterpret_cast<const uint8_t *>(flash), crc_len);
    if (computed != flash->crc32) return false;

    memcpy(&out, flash, sizeof(out));
    return true;
}

bool cal_save(const CalibData &d)
{
    // Build a mutable copy with magic, version, and CRC filled in
    CalibData buf;
    memcpy(&buf, &d, sizeof(buf));
    buf.magic   = CAL_MAGIC;
    buf.version = CAL_VERSION;
    memset(buf._pad, 0xFF, sizeof(buf._pad));

    constexpr size_t crc_len = sizeof(CalibData) - sizeof(uint32_t) - 44;
    buf.crc32 = crc32(reinterpret_cast<const uint8_t *>(&buf), crc_len);

    b2_unlock();
    bool ok = b2_erase_sector7();

    if (ok) {
        const uint32_t *src = reinterpret_cast<const uint32_t *>(&buf);
        for (int word = 0; word < (int)(sizeof(buf) / 32); ++word) {
            if (!b2_program_word(CAL_ADDR + static_cast<uint32_t>(word * 32), src + word * 8)) {
                ok = false;
                break;
            }
        }
    }

    b2_lock();
    return ok;
}

void cal_clear()
{
    b2_unlock();
    b2_erase_sector7();
    b2_lock();
}
