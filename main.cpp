/*
 * main.cpp — BPRL_Balance ChibiOS firmware
 * Target:  CubeOrangePlus (STM32H743ZI) at 400 MHz
 * Build:   make
 * Flash:   make flash PORT=/dev/ttyACM0
 *
 * ── What lives where ────────────────────────────────────────────────────────
 *   main.cpp                     Init calls, thread rate sequencer.
 *   src/threads.cpp/.hpp         Thread bodies + shared state.
 *   src/coms/SPI.*               ICM-20948/20602 onboard IMU drivers.
 *   src/coms/CAN.*               FDCAN1 + FDCAN2 drivers, device registration.
 *   src/coms/CANMotor.*          6-motor CAN abstraction (RMD + ODrive).
 *   src/coms/Radio.*             SBUS radio input (USART3, TELEM2 port).
 *   src/controllers/             RobotStateMachine, BalanceController, PID.
 *   src/state_estimator/         3-lane EKF.
 *
 * ── Adding a CAN device ─────────────────────────────────────────────────────
 *   void my_cb(const CANRxFrame &f, void *ctx) { ... }
 *   bprl_can_register(CAN_BUS_2, MY_ID, my_cb, nullptr);   // in init sequence below
 *
 * ── Motor layout (all on CAN bus 1) ─────────────────────────────────────────
 *   ID 1  Hip FL   — LKMTECH MG8016E-i6  (RMD protocol)
 *   ID 2  Hip FR   — LKMTECH MG8016E-i6
 *   ID 3  Hip RL   — LKMTECH MG8016E-i6
 *   ID 4  Hip RR   — LKMTECH MG8016E-i6
 *   ID 5  Wheel L  — Steadywin GIM6010-8 on GDS68, ODrive fw, node_id 2
 *   ID 6  Wheel R  — Steadywin GIM6010-8 on GDS68, ODrive fw, node_id 3
 *
 * Wheel node_ids (2/3) are these drives' own existing ODrive config, kept
 * as-is rather than reconfigured — deliberately different from their slot
 * ids (5/6) here since 2/3 are already taken by hip FR/RL. See
 * can_motor_register()'s node_id param in src/coms/CANMotor.hpp.
 */

#include "ch.h"
#include "hal.h"
#include "src/threads.hpp"
#include "src/coms/CAN.hpp"
#include "src/coms/CANMotor.hpp"
#include "src/coms/CANPower.hpp"
#include "src/coms/Radio.hpp"
#include "src/usb_serial.hpp"
#include "chprintf.h"

int main(void)
{
    halInit();

    /* IWDG — ~32 s timeout (LSI ≈ 32 kHz, /256, RLR=0xFFF) */
    IWDG1->KR  = 0x5555U;
    IWDG1->PR  = 0x06U;
    IWDG1->RLR = 0xFFFU;
    IWDG1->KR  = 0xCCCCU;
    IWDG1->KR  = 0xAAAAU;

    /* LED diagnostic blinks before RTOS:
     *   3 fast = halInit() done
     *   5 slow = chSysInit() + USB OK */
#define BLINK_TICK  4000000U
#define BLINK_SLOW 32000000U
#define BLINK_GAP    400000U
    for (int i = 0; i < 3; i++) {
        palSetLine(LINE_LED_ACTIVITY);
        { volatile uint32_t n = BLINK_TICK; while (n--) {} }
        palClearLine(LINE_LED_ACTIVITY);
        { volatile uint32_t n = BLINK_GAP;  while (n--) {} }
    }

    chSysInit();

    for (int i = 0; i < 5; i++) {
        palSetLine(LINE_LED_ACTIVITY);
        chThdSleepMilliseconds(400);
        palClearLine(LINE_LED_ACTIVITY);
        chThdSleepMilliseconds(100);
    }

    usb_serial_init();
    chThdSleepMilliseconds(1500);

    /* ══════════════════════════════════════════════════════════════════════
     * Thread rate sequencer
     * ══════════════════════════════════════════════════════════════════════ */
    static const ThreadRates kRates = {
        /* .spi       = */ TIME_US2I(1000),   // 1 kHz
        /* .est       = */ TIME_US2I(2000),   // 500 Hz
        /* .control   = */ TIME_US2I(2500),   // 400 Hz
        /* .radio     = */ TIME_MS2I(10),     // 100 Hz
        /* .heartbeat = */ TIME_MS2I(200),    // 5 Hz
        /* .debug     = */ TIME_MS2I(100),    // 10 Hz
        /* .log       = */ { TIME_MS2I(20) }, // 50 Hz
    };

    /* ── Peripheral initialisation ─────────────────────────────────────────
     * Order matters: CAN first so callbacks can be registered before threads
     * start sending and receiving frames.                                    */
    can_drv_init();       // start FDCAN1 + FDCAN2, register IMX5 callbacks
    power_mon_init();     // Matek CAN-L4-BM BatteryInfo on bus 2

    // 6 CAN motors on bus 1
    can_motor_register(1, CAN_MOTOR_RMD);     // hip FL
    can_motor_register(2, CAN_MOTOR_RMD);     // hip FR
    can_motor_register(3, CAN_MOTOR_RMD);     // hip RL
    can_motor_register(4, CAN_MOTOR_RMD);     // hip RR
    // node_id 2/3 matches these drives' existing ODrive config (unchanged
    // from final-project-Ian-McConachie-CU) — slot 5/6 is this project's own
    // numbering, kept apart from node_id specifically so it doesn't collide
    // with hip FR/RL's slot ids 2/3 above (see CANMotor.hpp's can_motor_register()).
    can_motor_register(5, CAN_MOTOR_ODRIVE, /*node_id=*/2);  // wheel L
    can_motor_register(6, CAN_MOTOR_ODRIVE, /*node_id=*/3);  // wheel R

    radio_input_init();   // SBUS on USART3 (TELEM2 port, 100000 baud 8E2, RXINV)

    threads_start(kRates);

    /* Main thread: feed watchdog */
    while (true) {
        IWDG1->KR = 0xAAAAU;
        chThdSleepMilliseconds(1000);
    }
    return 0;
}
