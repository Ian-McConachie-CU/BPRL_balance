/*
 * boards/CubeOrangePlus/board.h
 * CubeOrange+ — STM32H743ZI, FMUv5x hardware
 *
 * Identical pin assignments to CubeBlue H7 (same PCB).
 * Only the MCU variant differs: H743 vs H753.
 */

#ifndef BOARD_H
#define BOARD_H

/* ── Board identity ────────────────────────────────────────────────────────── */
#define BOARD_NAME              "CubeOrange+"
#define BOARD_MCU               "STM32H743ZI"

/* ── External crystal ──────────────────────────────────────────────────────── */
#define STM32_LSECLK            32768U
#define STM32_LSEDRV            (3U << 3U)
#define STM32_HSECLK            24000000U

/* ── Status LEDs ───────────────────────────────────────────────────────────── */
#define LINE_LED_ACTIVITY       PAL_LINE(GPIOB, 0U)

/* ── Debug / telemetry UART (USART3) ───────────────────────────────────────── */
#define LINE_UART3_TX           PAL_LINE(GPIOD, 8U)
#define LINE_UART3_RX           PAL_LINE(GPIOD, 9U)

/* ── Sensor power enable ────────────────────────────────────────────────────── */
#define LINE_SENSOR_PWR_EN      PAL_LINE(GPIOE, 3U)

/* ── SPI1 — primary IMU bus (ICM-20948) ────────────────────────────────────── */
#define LINE_SPI1_SCK           PAL_LINE(GPIOA, 5U)
#define LINE_SPI1_MISO          PAL_LINE(GPIOA, 6U)
#define LINE_SPI1_MOSI          PAL_LINE(GPIOA, 7U)
#define LINE_IMU1_CS            PAL_LINE(GPIOC, 2U)

/* ── SPI4 — secondary IMU / ext sensor bus ─────────────────────────────────── */
#define LINE_SPI4_SCK           PAL_LINE(GPIOE, 2U)
#define LINE_SPI4_MISO          PAL_LINE(GPIOE, 5U)
#define LINE_SPI4_MOSI          PAL_LINE(GPIOE, 6U)
#define LINE_IMU2_CS            PAL_LINE(GPIOE, 4U)
#define LINE_IMU3_CS            PAL_LINE(GPIOC, 13U)
#define LINE_BARO_CS            PAL_LINE(GPIOD, 7U)
#define LINE_BARO_EXT_CS        PAL_LINE(GPIOC, 14U)

/* ── IMU data-ready interrupts ─────────────────────────────────────────────── */
#define LINE_IMU1_DRDY          PAL_LINE(GPIOD, 15U)
#define LINE_IMU2_DRDY          PAL_LINE(GPIOD, 10U)

/* ── FDCAN1 ─────────────────────────────────────────────────────────────────── */
#define LINE_CAN1_TX            PAL_LINE(GPIOD, 1U)
#define LINE_CAN1_RX            PAL_LINE(GPIOD, 0U)

/* ── FDCAN2 ─────────────────────────────────────────────────────────────────── */
#define LINE_CAN2_TX            PAL_LINE(GPIOB, 6U)
#define LINE_CAN2_RX            PAL_LINE(GPIOB, 12U)

/* ── SDMMC1 (SD card) ───────────────────────────────────────────────────────── */
#define LINE_SDMMC1_D0          PAL_LINE(GPIOC, 8U)
#define LINE_SDMMC1_D1          PAL_LINE(GPIOC, 9U)
#define LINE_SDMMC1_D2          PAL_LINE(GPIOC, 10U)
#define LINE_SDMMC1_D3          PAL_LINE(GPIOC, 11U)
#define LINE_SDMMC1_CK          PAL_LINE(GPIOC, 12U)
#define LINE_SDMMC1_CMD         PAL_LINE(GPIOD, 2U)

/* ── I2C buses ──────────────────────────────────────────────────────────────── */
#define LINE_I2C1_SCL           PAL_LINE(GPIOB, 8U)
#define LINE_I2C1_SDA           PAL_LINE(GPIOB, 9U)
#define LINE_I2C2_SCL           PAL_LINE(GPIOB, 10U)
#define LINE_I2C2_SDA           PAL_LINE(GPIOB, 11U)

/* ── PWM / servo output (TIM1) ──────────────────────────────────────────────── */
#define LINE_FMU_CH1            PAL_LINE(GPIOE, 9U)
#define LINE_FMU_CH2            PAL_LINE(GPIOE, 11U)
#define LINE_FMU_CH3            PAL_LINE(GPIOE, 13U)
#define LINE_FMU_CH4            PAL_LINE(GPIOE, 14U)

/* ── AUX output power and voltage selection ─────────────────────────────────── */
/*
 * PA8  nVDD_5V_PERIPH_EN — active-LOW enable for the 5 V peripheral power rail.
 *   The PCB has a pull-up, so the reset default (floating input) leaves the
 *   rail DISABLED.  This rail powers the AUX-output level-shifter ICs, so
 *   driving it LOW is required for any signal to reach AUX 1-6.
 *   ArduPilot hwdef.inc: "PA8 nVDD_5V_PERIPH_EN OUTPUT LOW"
 *
 * PB4  PWM_VOLT_SEL — selects 3.3 V (HIGH) or 5 V (LOW) for AUX outputs.
 *   Must be driven HIGH to put the level-shifter in 3.3 V mode and release
 *   the output-enable of the buffer.
 *   ArduPilot hwdef.inc: "PB4 PWM_VOLT_SEL OUTPUT HIGH"
 */
#define LINE_PERIPH_5V_EN       PAL_LINE(GPIOA, 8U)   /* nVDD_5V_PERIPH_EN (active LOW) */
#define LINE_PWM_VOLT_SEL       PAL_LINE(GPIOB, 4U)   /* PWM_VOLT_SEL      (HIGH = 3.3 V) */

/* ── RC input ────────────────────────────────────────────────────────────────── */
#define LINE_RC_INPUT           PAL_LINE(GPIOC, 7U)

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
