#include "src/coms/SPI.hpp"

/*
 * Clock sources (from mcuconf.h / PLL config):
 *   SPI1 = PLL1_Q = 50 MHz   (STM32_SPI123SEL_PLL1_Q_CK, DIVQ=16)
 *   SPI4 = PCLK2  = 100 MHz  (STM32_SPI45SEL_PCLK2,  D2PPRE2=/2)
 *
 * ICM-45686 (SPI1): MODE0 (CPOL=0, CPHA=0).  Init ~781 kHz, fast 6.25 MHz.
 * ICM-42688 (SPI4): MODE3 (CPOL=1, CPHA=1).  Init ~781 kHz, fast 6.25 MHz.
 *
 * SPIConfig v1 field order: {circular, end_cb, ssport, sspad, cfg1, cfg2,
 *                            dummytx, dummyrx}
 */

// ── IMU1: ICM-45686 — SPI1  CS=PG1  MODE0 ────────────────────────────────────
static const SPIConfig imu1_init = {
    false, nullptr, GPIOG, 1U,
    SPI_CFG1_MBR_DIV64 | SPI_CFG1_DSIZE_VALUE(7), 0, // no CPOL/CPHA = MODE0
    nullptr, nullptr
};
static const SPIConfig imu1_fast = {
    false, nullptr, GPIOG, 1U,
    SPI_CFG1_MBR_DIV8  | SPI_CFG1_DSIZE_VALUE(7), 0,
    nullptr, nullptr
};

// ── IMU2: probe as ICM-45686 — SPI4  CS=PC15  MODE0 ──────────────────────────
static const SPIConfig imu2_init = {
    false, nullptr, GPIOC, 15U,
    SPI_CFG1_MBR_DIV128 | SPI_CFG1_DSIZE_VALUE(7), 0, // MODE0
    nullptr, nullptr
};
static const SPIConfig imu2_fast = {
    false, nullptr, GPIOC, 15U,
    SPI_CFG1_MBR_DIV16  | SPI_CFG1_DSIZE_VALUE(7), 0,
    nullptr, nullptr
};

// ── IMU3: probe as ICM-45686 — SPI4  CS=PC13  MODE0 ──────────────────────────
static const SPIConfig imu3_init = {
    false, nullptr, GPIOC, 13U,
    SPI_CFG1_MBR_DIV128 | SPI_CFG1_DSIZE_VALUE(7), 0, // MODE0
    nullptr, nullptr
};
static const SPIConfig imu3_fast = {
    false, nullptr, GPIOC, 13U,
    SPI_CFG1_MBR_DIV16  | SPI_CFG1_DSIZE_VALUE(7), 0,
    nullptr, nullptr
};

ICM45686 imu1;
ICM45686 imu2;
ICM45686 imu3;

void spi_drv_init(void)
{
    imu1.init(&SPID1, &imu1_init, &imu1_fast);
    imu2.init(&SPID4, &imu2_init, &imu2_fast);
    imu3.init(&SPID4, &imu3_init, &imu3_fast);
}
