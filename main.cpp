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
        /* .control   = */ TIME_US2I(10000),  // 100 Hz -- CONFIRMED 2026-09-02
                                                 // by direct measurement, not
                                                 // just theory: at 200/400 Hz
                                                 // send, hip tx_ok:rx ratio was
                                                 // only ~45-56% (RMD getting
                                                 // more commands/sec than it
                                                 // replies to); at 100 Hz it's
                                                 // ~92-98%, consistent across
                                                 // all 4 hips. tx_ok only means
                                                 // the STM32 got the frame onto
                                                 // the bus -- since RMD's reply
                                                 // IS the echo of the command
                                                 // (not an independent
                                                 // broadcast), a ratio well
                                                 // under 1 at higher send rates
                                                 // means the RMD's own command-
                                                 // processing loop can't keep up
                                                 // past ~100 Hz -- sending
                                                 // faster doesn't buy faster
                                                 // feedback, it just fills the
                                                 // bus with commands the drive
                                                 // was always going to ignore.
                                                 // (200 Hz was itself the fix
                                                 // for 400 Hz breaking USB
                                                 // command handling by
                                                 // starving USBCmdThread -- see
                                                 // git history -- but that was
                                                 // about bus TIMING budget, a
                                                 // separate concern from this
                                                 // rate's actual reply-rate
                                                 // ceiling, and 100 Hz has even
                                                 // more headroom there too.)
        /* .wheel_send= */ TIME_US2I(10000),  // 100 Hz -- CONFIRMED 2026-09-02:
                                                 // wheel Hz measured ~110-118 at
                                                 // 300 Hz send and ~99-112 here
                                                 // at 100 Hz -- statistically
                                                 // the same. Unlike hips,
                                                 // ODrive's Get_Encoder_Estimates
                                                 // is an independent periodic
                                                 // broadcast on the ODrive's OWN
                                                 // schedule, not a reply to our
                                                 // command (rx even exceeds
                                                 // tx_ok on the untested wheel in
                                                 // testing) -- so this thread's
                                                 // send rate was never what set
                                                 // wheel telemetry rate in the
                                                 // first place. Left at 100 Hz
                                                 // to match .control rather than
                                                 // for its own reason -- raising
                                                 // it back up wouldn't cost much
                                                 // (wheels don't share hips'
                                                 // bus-budget constraint: an
                                                 // ODrive command needs no reply
                                                 // -- see CANMotor.cpp), so this
                                                 // is 2 small frames/tick, not
                                                 // the 5-frame hip broadcast+
                                                 // reply exchange. Priority
                                                 // deliberately set BELOW
                                                 // USBCmdThread's (the exact
                                                 // thing that starved when
                                                 // ControlThread itself went
                                                 // to 400 Hz) so this can never
                                                 // reproduce that failure even
                                                 // if bus load estimates are
                                                 // off -- worst case it falls
                                                 // behind, it can't block USB.
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
    // sign=-1 on wheel L: mechanically mirrored relative to wheel R (see
    // can_motor_register()'s sign param in CANMotor.hpp) -- without this, a
    // positive command/reading meant opposite physical directions on the
    // two sides.
    can_motor_register(5, CAN_MOTOR_ODRIVE, /*node_id=*/2, /*sign=*/-1.0f);  // wheel L
    can_motor_register(6, CAN_MOTOR_ODRIVE, /*node_id=*/3);                  // wheel R

    radio_input_init();   // SBUS on USART3 (TELEM2 port, 100000 baud 8E2, RXINV)

    threads_start(kRates);

    /* Main thread: feed watchdog */
    while (true) {
        IWDG1->KR = 0xAAAAU;
        chThdSleepMilliseconds(1000);
    }
    return 0;
}
