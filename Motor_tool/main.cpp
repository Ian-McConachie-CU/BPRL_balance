/*
 * main.cpp — Motor_tool: standalone CAN motor test/setup firmware
 * Target:  CubeOrangePlus (STM32H743ZI) at 400 MHz — same board as BPRL_balance.
 * Build:   make
 * Flash:   make flash PORT=/dev/ttyACM0
 *
 * A minimal, dependency-free firmware whose only job is bringing up FDCAN1/2
 * and a USB CDC command line so you can sniff, scan, and drive CAN motors
 * from a host PC without needing the full BPRL_balance robot firmware. See
 * README.md for the command reference and tools/motor_tool.py for the ground
 * tool. Supports:
 *   - LK-TECH MG8016E-i6 (RMD-style protocol)  — src/RmdMotor.*
 *   - Steadywin GIM6010-6 (SDC102 driver)      — src/GimMotor.* (protocol
 *     unverified — read the warning at the top of GimMotor.hpp before use)
 */

#include "ch.h"
#include "hal.h"
#include "src/CAN.hpp"
#include "src/RmdMotor.hpp"
#include "src/GimMotor.hpp"
#include "src/Imx5.hpp"
#include "src/CmdShell.hpp"
#include "src/usb_serial.hpp"

/* Motor_tool has no SD card / logging use, but it shares BPRL_balance's
 * cfg/halconf.h (HAL_USE_SDC=TRUE) rather than forking it, so ChibiOS's SDC
 * driver still needs these board-callback symbols at link time. Same stub
 * BPRL_balance's Logger.cpp provides — SD-card detect is simply unused here. */
extern "C" {
bool sdc_lld_is_card_inserted(SDCDriver *sdcp)   { (void)sdcp; return false; }
bool sdc_lld_is_write_protected(SDCDriver *sdcp) { (void)sdcp; return true;  }
}

int main(void)
{
    halInit();

    /* IWDG — ~32 s timeout (LSI ~32 kHz, /256, RLR=0xFFF). Fed from the main
     * loop below; a hang here means the watchdog resets the board rather
     * than leaving a motor command latched on the bus indefinitely. */
    IWDG1->KR  = 0x5555U;
    IWDG1->PR  = 0x06U;
    IWDG1->RLR = 0xFFFU;
    IWDG1->KR  = 0xCCCCU;
    IWDG1->KR  = 0xAAAAU;

    /* LED diagnostic blinks before RTOS: 3 fast = halInit() done. */
    for (int i = 0; i < 3; i++) {
        palSetLine(LINE_LED_ACTIVITY);
        { volatile uint32_t n = 4000000U; while (n--) {} }
        palClearLine(LINE_LED_ACTIVITY);
        { volatile uint32_t n = 400000U;  while (n--) {} }
    }

    chSysInit();

    /* 5 slow blinks = chSysInit() + USB about to start. */
    for (int i = 0; i < 5; i++) {
        palSetLine(LINE_LED_ACTIVITY);
        chThdSleepMilliseconds(400);
        palClearLine(LINE_LED_ACTIVITY);
        chThdSleepMilliseconds(100);
    }

    usb_serial_init();
    chThdSleepMilliseconds(1500);   // let host USB enumeration settle

    can_drv_init();     // FDCAN1 + FDCAN2 @ 1 Mbit/s, RX thread running
    rmd_init();          // subscribes RMD feedback decoder
    gim_init();           // subscribes GIM feedback decoder
    imx5_init();           // subscribes IMX5 INS decoder (bus 2)
    cmd_shell_start();     // USB command line + safety watchdog

    /* Main thread: feed IWDG, heartbeat LED (steady 5 Hz = alive). */
    uint32_t tick = 0;
    while (true) {
        IWDG1->KR = 0xAAAAU;
        if (tick % 2 == 0) palSetLine(LINE_LED_ACTIVITY);
        else palClearLine(LINE_LED_ACTIVITY);
        tick++;
        chThdSleepMilliseconds(200);
    }
    return 0;
}
