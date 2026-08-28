#pragma once
#include "hal.h"

/*
 * Driver for InvenSense ICM-20948 (9-DOF, SPI).
 * Two instances on FMUv5x:
 *   imu1 — SPI1, CS = PC2  (primary)
 *   imu2 — SPI4, CS = PE4  (external)
 *
 * Call init() once from spi_drv_init() (inside SPIThread).
 * Call read() every SPIThread tick (1 kHz).
 */
class ICM20948 {
public:
    bool init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast);
    bool read(float accel_ms2[3], float gyro_rads[3]);

private:
    void    set_bank(uint8_t bank);
    uint8_t reg_read(uint8_t reg);
    void    reg_write(uint8_t reg, uint8_t val);
    void    burst_read(uint8_t reg, uint8_t *buf, size_t n);

    SPIDriver       *_spid     = nullptr;
    const SPIConfig *_cfg_init = nullptr;
    const SPIConfig *_cfg_fast = nullptr;
    uint8_t          _bank     = 0xFF;
    bool             _ready    = false;
    uint8_t          _whoami   = 0;

public:
    uint8_t whoami() const { return _whoami; }

private:
    uint8_t _txbuf[32] __attribute__((aligned(32)));
    uint8_t _rxbuf[32] __attribute__((aligned(32)));
};
