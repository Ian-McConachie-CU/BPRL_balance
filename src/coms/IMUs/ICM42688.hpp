#pragma once
#include "hal.h"

/*
 * Driver for TDK/InvenSense ICM-42688-P (6-DOF accel/gyro, SPI Mode 3).
 * On CubeOrangePlus FMUv5x: SPI4, two instances:
 *   imu2 — CS = PC15 (ACCEL_EXT_CS)   instance 0
 *   imu3 — CS = PC13 (GYRO_EXT_CS)    instance 1
 *
 * Call init() once from spi_drv_init() (inside SPIThread).
 * Call read() every SPIThread tick (1 kHz).
 */
class ICM42688 {
public:
    bool init(SPIDriver *spid, const SPIConfig *cfg_init, const SPIConfig *cfg_fast);
    bool read(float accel_ms2[3], float gyro_rads[3]);
    uint8_t whoami() const { return _whoami; }

private:
    uint8_t reg_read(uint8_t reg);
    void    reg_write(uint8_t reg, uint8_t val);
    void    burst_read(uint8_t reg, uint8_t *buf, size_t n);

    SPIDriver       *_spid     = nullptr;
    const SPIConfig *_cfg_init = nullptr;
    const SPIConfig *_cfg_fast = nullptr;
    bool             _ready    = false;
    uint8_t          _whoami   = 0;

    uint8_t _txbuf[32] __attribute__((aligned(32)));
    uint8_t _rxbuf[32] __attribute__((aligned(32)));
};
