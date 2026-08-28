#include "src/coms/IMUs/ICM20602.hpp"
#include <cstring>

static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
static constexpr uint8_t REG_CONFIG       = 0x1A;
static constexpr uint8_t REG_GYRO_CFG     = 0x1B;
static constexpr uint8_t REG_ACCEL_CFG    = 0x1C;
static constexpr uint8_t REG_ACCEL_CFG2   = 0x1D;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t REG_WHO_AM_I     = 0x75;
static constexpr uint8_t WHOAMI_VALUE     = 0x12;

static constexpr float GYRO_SCALE  = (1.0f / 16.4f)  * (3.14159265f / 180.0f);
static constexpr float ACCEL_SCALE = (1.0f / 2048.0f) * 9.80665f;

bool ICM20602::init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast)
{
    _spid     = spid;
    _cfg_init = cfg_init;
    _cfg_fast = cfg_fast;
    _ready    = false;

    _whoami = reg_read(REG_WHO_AM_I);
    if (_whoami != WHOAMI_VALUE) { return false; }

    reg_write(REG_PWR_MGMT_1, 0x80);  // reset
    chThdSleepMilliseconds(100);
    reg_write(REG_PWR_MGMT_1, 0x01);  // wake, PLL clock
    chThdSleepMilliseconds(5);

    reg_write(REG_SMPLRT_DIV, 0x00);  // 1 kHz ODR
    reg_write(REG_CONFIG,     0x01);  // DLPF 184 Hz bandwidth
    reg_write(REG_GYRO_CFG,   0x18);  // ±2000 dps
    reg_write(REG_ACCEL_CFG,  0x18);  // ±16 g
    reg_write(REG_ACCEL_CFG2, 0x01);  // 99 Hz accel bandwidth

    _ready = true;
    return true;
}

bool ICM20602::read(float accel_ms2[3], float gyro_rads[3])
{
    if (!_ready) { return false; }

    uint8_t raw[14];
    burst_read(REG_ACCEL_XOUT_H, raw, 14);

    auto be16 = [](const uint8_t *p) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
    };

    accel_ms2[0] = be16(raw + 0)  * ACCEL_SCALE;
    accel_ms2[1] = be16(raw + 2)  * ACCEL_SCALE;
    accel_ms2[2] = be16(raw + 4)  * ACCEL_SCALE;
    // raw[6:7] = temperature, skipped
    gyro_rads[0] = be16(raw + 8)  * GYRO_SCALE;
    gyro_rads[1] = be16(raw + 10) * GYRO_SCALE;
    gyro_rads[2] = be16(raw + 12) * GYRO_SCALE;
    return true;
}

uint8_t ICM20602::reg_read(uint8_t reg)
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

void ICM20602::reg_write(uint8_t reg, uint8_t val)
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

void ICM20602::burst_read(uint8_t reg, uint8_t *buf, size_t n)
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
