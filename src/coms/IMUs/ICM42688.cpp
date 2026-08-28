#include "src/coms/IMUs/ICM42688.hpp"
#include <cstring>

static constexpr uint8_t REG_PWR_MGMT0     = 0x4E;
static constexpr uint8_t REG_GYRO_CONFIG0  = 0x4F;
static constexpr uint8_t REG_ACCEL_CONFIG0 = 0x50;
static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x1F;  // burst: 6 accel + 6 gyro bytes
static constexpr uint8_t REG_WHO_AM_I      = 0x75;
static constexpr uint8_t WHOAMI_VALUE      = 0x47;

// ±2000 dps, ±16 g  (FS_SEL = 0b000 for each → same as ICM-20602 at same range)
static constexpr float GYRO_SCALE  = (1.0f / 16.4f)  * (3.14159265f / 180.0f);
static constexpr float ACCEL_SCALE = (1.0f / 2048.0f) * 9.80665f;

bool ICM42688::init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast)
{
    _spid     = spid;
    _cfg_init = cfg_init;
    _cfg_fast = cfg_fast;
    _ready    = false;

    _whoami = reg_read(REG_WHO_AM_I);
    if (_whoami != WHOAMI_VALUE) { return false; }

    // Wake both sensors in low-noise mode (bits [3:2]=11 gyro LN, [1:0]=11 accel LN)
    reg_write(REG_PWR_MGMT0, 0x0F);
    chThdSleepMilliseconds(1);

    // ±2000 dps, 1 kHz ODR  (FS_SEL[7:5]=000, ODR[3:0]=0110)
    reg_write(REG_GYRO_CONFIG0,  0x06);
    // ±16 g, 1 kHz ODR
    reg_write(REG_ACCEL_CONFIG0, 0x06);
    chThdSleepMilliseconds(20);  // accel needs 20 ms after enable

    _ready = true;
    return true;
}

bool ICM42688::read(float accel_ms2[3], float gyro_rads[3])
{
    if (!_ready) { return false; }

    // Direct register burst: ACCEL_DATA_X1..Z0 (0x1F-0x24) then GYRO_DATA_X1..Z0 (0x25-0x2A)
    uint8_t raw[12];
    burst_read(REG_ACCEL_XOUT_H, raw, 12);

    // Big-endian: _X1 (high byte) precedes _X0 (low byte)
    auto be16 = [](const uint8_t *p) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
    };

    accel_ms2[0] = be16(raw + 0)  * ACCEL_SCALE;
    accel_ms2[1] = be16(raw + 2)  * ACCEL_SCALE;
    accel_ms2[2] = be16(raw + 4)  * ACCEL_SCALE;
    gyro_rads[0] = be16(raw + 6)  * GYRO_SCALE;
    gyro_rads[1] = be16(raw + 8)  * GYRO_SCALE;
    gyro_rads[2] = be16(raw + 10) * GYRO_SCALE;
    return true;
}

uint8_t ICM42688::reg_read(uint8_t reg)
{
    _txbuf[0] = reg | 0x80;
    _txbuf[1] = 0xFF;
    cacheBufferFlush(_txbuf, 32);
    spiAcquireBus(_spid);
    spiStart(_spid, _cfg_init);
    spiSelect(_spid);
    spiExchange(_spid, 2, _txbuf, _rxbuf);
    spiUnselect(_spid);
    spiReleaseBus(_spid);
    cacheBufferInvalidate(_rxbuf, 32);
    return _rxbuf[1];
}

void ICM42688::reg_write(uint8_t reg, uint8_t val)
{
    _txbuf[0] = reg & 0x7F;
    _txbuf[1] = val;
    cacheBufferFlush(_txbuf, 32);
    spiAcquireBus(_spid);
    spiStart(_spid, _cfg_init);
    spiSelect(_spid);
    spiSend(_spid, 2, _txbuf);
    spiUnselect(_spid);
    spiReleaseBus(_spid);
}

void ICM42688::burst_read(uint8_t reg, uint8_t *buf, size_t n)
{
    _txbuf[0] = reg | 0x80;
    memset(_txbuf + 1, 0xFF, n);
    cacheBufferFlush(_txbuf, 32);
    spiAcquireBus(_spid);
    spiStart(_spid, _cfg_fast);
    spiSelect(_spid);
    spiExchange(_spid, n + 1, _txbuf, _rxbuf);
    spiUnselect(_spid);
    spiReleaseBus(_spid);
    cacheBufferInvalidate(_rxbuf, 32);
    memcpy(buf, _rxbuf + 1, n);
}
