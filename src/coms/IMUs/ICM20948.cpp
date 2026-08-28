#include "src/coms/IMUs/ICM20948.hpp"
#include <cstring>

// Register map — Bank 0
static constexpr uint8_t B0_WHO_AM_I      = 0x00;
static constexpr uint8_t B0_USER_CTRL     = 0x03;
static constexpr uint8_t B0_PWR_MGMT_1    = 0x06;
static constexpr uint8_t B0_ACCEL_XOUT_H  = 0x2D;

// Register map — Bank 2
static constexpr uint8_t B2_GYRO_SMPLRT   = 0x00;
static constexpr uint8_t B2_GYRO_CFG1     = 0x01;
static constexpr uint8_t B2_ACCEL_SMPL_1  = 0x10;
static constexpr uint8_t B2_ACCEL_SMPL_2  = 0x11;
static constexpr uint8_t B2_ACCEL_CFG     = 0x14;

static constexpr uint8_t REG_BANK_SEL     = 0x7F;
static constexpr uint8_t WHOAMI_VALUE     = 0xEA;

// ±2000 dps → rad/s,  ±16 g → m/s²
static constexpr float GYRO_SCALE  = (1.0f / 16.4f)  * (3.14159265f / 180.0f);
static constexpr float ACCEL_SCALE = (1.0f / 2048.0f) * 9.80665f;

void ICM20948::set_bank(uint8_t bank)
{
    if (bank == _bank) { return; }
    _txbuf[0] = REG_BANK_SEL & 0x7F;
    _txbuf[1] = static_cast<uint8_t>(bank << 4);
    cacheBufferFlush(_txbuf, 32);
    spiAcquireBus(_spid);
    spiStart(_spid, _cfg_init);
    spiSelect(_spid);
    spiSend(_spid, 2, _txbuf);
    spiUnselect(_spid);
    spiReleaseBus(_spid);
    _bank = bank;
}

bool ICM20948::init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast)
{
    _spid     = spid;
    _cfg_init = cfg_init;
    _cfg_fast = cfg_fast;
    _bank     = 0xFF;
    _ready    = false;

    set_bank(0);
    _whoami = reg_read(B0_WHO_AM_I);
    if (_whoami != WHOAMI_VALUE) { return false; }

    reg_write(B0_PWR_MGMT_1, 0xC1);   // reset + sleep
    chThdSleepMilliseconds(100);
    reg_write(B0_PWR_MGMT_1, 0x01);   // wake, auto clock
    chThdSleepMilliseconds(5);
    reg_write(B0_USER_CTRL,  0x10);   // SPI-only mode

    set_bank(2);
    reg_write(B2_GYRO_SMPLRT,  0x00); // 1.125 kHz ODR
    reg_write(B2_GYRO_CFG1,    0x07); // ±2000 dps, DLPF on
    reg_write(B2_ACCEL_SMPL_1, 0x00);
    reg_write(B2_ACCEL_SMPL_2, 0x00); // 1.125 kHz ODR
    reg_write(B2_ACCEL_CFG,    0x19); // ±16 g, DLPF on

    set_bank(0);
    _ready = true;
    return true;
}

bool ICM20948::read(float accel_ms2[3], float gyro_rads[3])
{
    if (!_ready) { return false; }
    set_bank(0);

    uint8_t raw[12];
    burst_read(B0_ACCEL_XOUT_H, raw, 12);

    auto be16 = [](const uint8_t *p) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
    };

    accel_ms2[0] = be16(raw + 0) * ACCEL_SCALE;
    accel_ms2[1] = be16(raw + 2) * ACCEL_SCALE;
    accel_ms2[2] = be16(raw + 4) * ACCEL_SCALE;
    gyro_rads[0] = be16(raw + 6) * GYRO_SCALE;
    gyro_rads[1] = be16(raw + 8) * GYRO_SCALE;
    gyro_rads[2] = be16(raw + 10) * GYRO_SCALE;
    return true;
}

uint8_t ICM20948::reg_read(uint8_t reg)
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

void ICM20948::reg_write(uint8_t reg, uint8_t val)
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

void ICM20948::burst_read(uint8_t reg, uint8_t *buf, size_t n)
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
