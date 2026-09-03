/*
 * threads.cpp — BPRL_Balance thread function definitions.
 *
 * Thread overview:
 *   SPIThread      +30  1 kHz    Read three onboard IMUs via SPI
 *   CANThread      +28  event    Receive frames from FDCAN1 and FDCAN2
 *   StateEstThread +25  500 Hz   3-lane EKF → g_state[]
 *   ControlThread  +20  100 Hz   RobotStateMachine → hip CAN commands
 *   WheelSendThread +18 100 Hz   Wheel-only CAN commands (see its own comment)
 *   RadioThread    +10  100 Hz   SBUS → g_input[] / g_armed
 *   HeartbeatThread -5  5 Hz     LED blink
 *   DebugThread    -10  10 Hz    USB $IMU/$TEL/$EKFL stream (BPRL_DEBUG only)
 *   USBCmdThread   -20  event    USB command parser (LOG, CAL, CAN, MOTOR, POWER, RC, TGT)
 *   LogThread      -15  50 Hz    Binary SD card logging
 */

#include "src/threads.hpp"
#include "src/RobotState.hpp"
#include "src/RobotTelemetry.hpp"
#include "src/coms/SPI.hpp"
#include "src/coms/CAN.hpp"
#include "src/coms/CANMotor.hpp"
#include "src/coms/CANPower.hpp"
#include "src/coms/Radio.hpp"
#include "src/coms/SBUS.hpp"
#include "src/controllers/RobotStateMachine.hpp"
#include "src/controllers/ActuatorSafety.hpp"
#include "src/controllers/MotorTest.hpp"
#include "src/state_estimator/StateManager.hpp"
#include "src/logging/Logger.hpp"
#include "src/logging/LogMessages.hpp"
#include "src/usb_serial.hpp"
#include "src/coms/CalFlash.hpp"
#include "chprintf.h"
#include "memstreams.h"
#include "ff.h"
#include <cstring>
#include <cstdio>
#include <cmath>

/* ── Shared state definitions ────────────────────────────────────────────── */

MUTEX_DECL(state_mtx);
float   g_state[StateIdx::N]         = {};
float   g_euler[3]                   = {};
float   g_input[InputIdx::N_INPUTS]  = {};
float   g_motor_torques[6]           = {};
bool    g_armed                      = false;

MUTEX_DECL(imu_mtx);
IMURaw g_imu[3] = {};

MUTEX_DECL(can_imu_mtx);
CANIMURaw g_can_imu = {1.0f};  // q0=1: identity quaternion

MUTEX_DECL(mocap_mtx);
MocapRaw g_mocap = {};

/* ── Calibration data loaded from flash at boot ──────────────────────────── */
static CalibData g_cal      = {};
static bool      g_cal_valid = false;

/* ── USB write serialisation ─────────────────────────────────────────────── */
static MUTEX_DECL(s_usb_write_mtx);

/* ── Controller + estimator instances ───────────────────────────────────── */
static RobotStateMachine robot_sm;
static StateManager      state_mgr;
static ActuatorSafety    actuator_safety;

/* ── Thread working areas ────────────────────────────────────────────────── */
static THD_WORKING_AREA(waSPI,       2048);
static THD_WORKING_AREA(waCAN,       2048);
static THD_WORKING_AREA(waStateEst,  6144);
static THD_WORKING_AREA(waControl,   2048);
static THD_WORKING_AREA(waWheelSend, 1024);
static THD_WORKING_AREA(waRadio,     1024);
static THD_WORKING_AREA(waHeartbeat, 1024);
static THD_WORKING_AREA(waLog,       8192);
static THD_WORKING_AREA(waUSBCmd,    4096);
#ifdef BPRL_DEBUG
static THD_WORKING_AREA(waDebug,     2048);
#endif

static uint8_t __attribute__((section(".nocache"))) s_usb_dl_buf[2048];

#ifdef BPRL_DEBUG
static volatile uint32_t s_can_quat_cnt = 0;
static volatile uint32_t s_can_rate_cnt = 0;
static float s_lane_roll[3]  = {};
static float s_lane_pitch[3] = {};
static float s_lane_yaw[3]   = {};
static float s_lane_p[3]     = {};
static float s_lane_q[3]     = {};
static float s_lane_r[3]     = {};
static int   s_primary_lane  = 0;
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * SPIThread — 1 kHz  NORMALPRIO+30
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(SPIThread, arg)
{
    chRegSetThreadName("spi");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    g_cal_valid = cal_load(g_cal);
    if (!g_cal_valid) memset(&g_cal, 0, sizeof(g_cal));

    spi_drv_init();

    systime_t next = chVTGetSystemTime();
    while (true) {
        float a[3], g[3];

        if (imu1.read(a, g)) {
            static constexpr float RS = 0.70710678f;
            const float ra[3] = { RS*(a[1]-a[0]), RS*(a[1]+a[0]), -a[2] };
            const float rg[3] = { RS*(g[1]-g[0]), RS*(g[1]+g[0]), -g[2] };
            chMtxLock(&imu_mtx);
            for (int k = 0; k < 3; k++) {
                g_imu[0].accel[k] = ra[k] - g_cal.accel_bias[0][k];
                g_imu[0].gyro[k]  = rg[k] - g_cal.gyro_bias[0][k];
            }
            g_imu[0].valid = true;
            chMtxUnlock(&imu_mtx);
        }
        if (imu2.read(a, g)) {
            const float ra[3] = {-a[1], +a[0], +a[2]};
            const float rg[3] = {-g[1], +g[0], +g[2]};
            chMtxLock(&imu_mtx);
            for (int k = 0; k < 3; k++) {
                g_imu[1].accel[k] = ra[k] - g_cal.accel_bias[1][k];
                g_imu[1].gyro[k]  = rg[k] - g_cal.gyro_bias[1][k];
            }
            g_imu[1].valid = true;
            chMtxUnlock(&imu_mtx);
        }
        if (imu3.read(a, g)) {
            const float ra[3] = {-a[1], -a[0], -a[2]};
            const float rg[3] = {-g[1], -g[0], -g[2]};
            chMtxLock(&imu_mtx);
            for (int k = 0; k < 3; k++) {
                g_imu[2].accel[k] = ra[k] - g_cal.accel_bias[2][k];
                g_imu[2].gyro[k]  = rg[k] - g_cal.gyro_bias[2][k];
            }
            g_imu[2].valid = true;
            chMtxUnlock(&imu_mtx);
        }

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * CANThread — event-driven  NORMALPRIO+28
 * Polls both FDCAN1 and FDCAN2 in round-robin with short timeouts.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(CANThread, arg)
{
    (void)arg;
    chRegSetThreadName("can");

    while (true) {
        CANRxFrame rxf;
        // Bus 1 (motors, IMX5, strain sensor)
        if (canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &rxf, TIME_US2I(500)) == MSG_OK)
            can_dispatch(CAN_BUS_1, rxf);
        can_check_busoff(CAN_BUS_1);
        // Bus 2 (external IMU, current sensor)
        if (canReceiveTimeout(&CAND2, CAN_ANY_MAILBOX, &rxf, TIME_US2I(500)) == MSG_OK)
            can_dispatch(CAN_BUS_2, rxf);
        can_check_busoff(CAN_BUS_2);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * StateEstThread — 500 Hz  NORMALPRIO+25
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(StateEstThread, arg)
{
    chRegSetThreadName("est");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);
    const float dt = static_cast<float>(period)
                   / static_cast<float>(CH_CFG_ST_FREQUENCY);

    state_mgr.init();

    uint32_t can_stale_ticks = 0;
    static constexpr uint32_t CAN_TIMEOUT_TICKS = 500;

    systime_t next = chVTGetSystemTime();
    while (true) {
        CANIMURaw can_snap;
        chMtxLock(&can_imu_mtx);
        can_snap = g_can_imu;
        g_can_imu.has_new_quat  = false;
        g_can_imu.has_new_rates = false;
        chMtxUnlock(&can_imu_mtx);

        if (can_snap.has_new_quat || can_snap.has_new_rates) {
            can_stale_ticks = 0;
        } else if (++can_stale_ticks > CAN_TIMEOUT_TICKS) {
            chMtxLock(&can_imu_mtx);
            g_can_imu.valid = false;
            chMtxUnlock(&can_imu_mtx);
            can_snap.valid = false;
        }

        IMURaw imu_snap[3];
        chMtxLock(&imu_mtx);
        memcpy(imu_snap, g_imu, sizeof(g_imu));
        chMtxUnlock(&imu_mtx);

        MocapRaw mocap_snap;
        chMtxLock(&mocap_mtx);
        mocap_snap = g_mocap;
        g_mocap.has_new = false;
        chMtxUnlock(&mocap_mtx);

        // Hip motors (CAN ids 1-4, FL/FR/RL/RR) and wheel motors (ids 5,6,
        // L/R). can_motor_get_state() is internally mutex-protected
        // (motor_state_mtx), safe to call here.
        CanMotorState hip_snap[4] = {};
        for (int i = 0; i < 4; i++) can_motor_get_state((uint8_t)(i + 1), &hip_snap[i]);
        CanMotorState wheel_snap[2] = {};
        can_motor_get_state(5, &wheel_snap[0]);
        can_motor_get_state(6, &wheel_snap[1]);

        state_mgr.update(dt, imu_snap, can_snap, mocap_snap);
        state_mgr.update_legs_and_wheels(hip_snap, wheel_snap);
#ifdef BPRL_DEBUG
        if (can_snap.has_new_quat)  s_can_quat_cnt++;
        if (can_snap.has_new_rates) s_can_rate_cnt++;
#endif

        chMtxLock(&state_mtx);
        state_mgr.get_state(g_state);
        g_euler[0] = state_mgr.roll();
        g_euler[1] = state_mgr.pitch();
        g_euler[2] = state_mgr.yaw();
#ifdef BPRL_DEBUG
        for (int li = 0; li < StateManager::NUM_LANES; ++li) {
            state_mgr.get_lane_euler(li, s_lane_roll[li], s_lane_pitch[li], s_lane_yaw[li]);
            state_mgr.get_lane_pqr  (li, s_lane_p[li],   s_lane_q[li],     s_lane_r[li]);
        }
        s_primary_lane = state_mgr.primary_lane();
#endif
        chMtxUnlock(&state_mtx);

        telemetry_update(g_state, state_mgr, hip_snap, wheel_snap);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * ControlThread — sends at kRates.control, recomputes at 100 Hz  NORMALPRIO+20
 * RobotStateMachine → CAN motor torque commands.
 *
 * Zero-order hold, added 2026-09-02: this thread wakes at whatever rate
 * main.cpp's kRates.control specifies (the hip CAN send rate, tuned against
 * bus-1's timing budget -- see that comment), but only recomputes
 * state_snap/input_snap and the control law (robot_sm or MotorTest) at a
 * fixed 100 Hz regardless -- there's no benefit recomputing against
 * g_state/g_input faster than they actually change (wheel telemetry was
 * observed arriving at ~100 Hz). `compute_divisor` derives how many send
 * ticks make up one 100 Hz compute step from whatever period was actually
 * passed in, so this stays correct across rate experiments without needing
 * a hand-edited constant each time (e.g. period=200 Hz -> divisor=2,
 * period=100 Hz -> divisor=1, i.e. every tick computes). The computed
 * `torques[]` (and a wheel test's velocity target, see MotorTest's
 * wheel_velocity_target()) are held static across ticks and re-transmitted
 * on every tick regardless -- for hips this also means the RMD command-echo
 * reply (and hence hip position feedback) still refreshes at the full send
 * rate even when the higher-level decision updates less often. Total CAN
 * traffic per second is set entirely by the send rate -- this only cuts the
 * CPU cost of the control-law evaluation itself, not bus load.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(ControlThread, arg)
{
    chRegSetThreadName("ctrl");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);
    // max(1, ...) guards a send rate AT or BELOW 100 Hz (divisor would
    // otherwise be 0 -- undefined tick % 0 -- instead just computes every
    // tick, i.e. compute rate tracks send rate down below 100 Hz).
    const uint32_t compute_divisor = (uint32_t)((TIME_US2I(10000) / period) < 1 ? 1 : (TIME_US2I(10000) / period));

    static float torques[6] = {};   // ZOH buffer -- see header comment above
    uint32_t tick = 0;

    systime_t next = chVTGetSystemTime();
    while (true) {
        const bool compute_tick = ((tick++ % compute_divisor) == 0u);

        if (compute_tick) {
            float state_snap[StateIdx::N];
            float input_snap[InputIdx::N_INPUTS];
            bool  armed_snap;
            chMtxLock(&state_mtx);
            memcpy(state_snap, g_state, sizeof(state_snap));
            memcpy(input_snap, g_input, sizeof(input_snap));
            armed_snap = g_armed;
            chMtxUnlock(&state_mtx);

            if (g_motor_test.active()) {
                // Bench sweep test overrides the normal control path entirely
                // (see MotorTest.hpp) -- but only while disarmed. Arming is
                // treated as an abort signal: the operator asking for real
                // robot behavior takes priority over a running bench test,
                // no separate "did you mean to arm during a test" prompt.
                if (armed_snap) {
                    // settle=false: a blocking ~150ms mode-revert here would
                    // stall every motor's torque command for that long,
                    // worse than a best-effort revert. See MotorTest::stop().
                    g_motor_test.stop(/*settle=*/false);
                    robot_sm.update(state_snap, input_snap, armed_snap, torques);
                } else {
                    g_motor_test.update(torques);
                }
            } else {
                robot_sm.update(state_snap, input_snap, armed_snap, torques);
            }

            // Final safety gate — always applied, regardless of
            // mode/controller (see ActuatorSafety.hpp): wheel velocity soft
            // limit + hard torque clamp, fail-safe zero on missing feedback.
            // Hip bounds are enforced separately, at the source, in
            // CANMotor.cpp -- see its header comment -- so they apply here
            // too without duplicating the logic. Only needs to run on
            // compute ticks -- `torques[]` doesn't change on hold ticks.
            actuator_safety.apply(torques);
        }

        // Send hip torque commands every tick (100 Hz), computed or held --
        // ONE multi-motor broadcast frame (0x280, see CANMotor.hpp) instead
        // of 4 separate commands -- drives switched to broadcast mode
        // 2026-09-02, alongside the bus-1 bit-rate drop to 500 kbit/s (see
        // CAN.cpp). Wheels are NOT sent from here -- see WheelSendThread
        // (own 100 Hz rate, its own thread so it can never compete with
        // USBCmdThread the way raising this thread's own rate did).
        can_motor_set_hip_torques_broadcast(&torques[0]);

        // RMD hip voltage/error telemetry (0x9A) — not part of the
        // torque-command reply, so nothing else refreshes it. One hip per
        // ~100 ticks (1 Hz aggregate at this thread's 100 Hz rate, 0.25 Hz per
        // hip) is still plenty for a slow-changing value; see
        // can_motor_poll_status_round_robin().
        static uint32_t s_status_poll_tick = 0;
        if (++s_status_poll_tick >= 100) {
            s_status_poll_tick = 0;
            can_motor_poll_status_round_robin();
        }

        // Wheel encoder telemetry (ODrive Get_Encoder_Estimates): DISABLED
        // 2026-09-02, see telemetry_plan.md item C's "fallback" clause and
        // the Open Items list -- this was the planned bench check turning
        // up a real problem, not a hypothetical. Real hardware evidence:
        // (1) the ODrive already broadcasts Get_Encoder_Estimates natively
        // at a healthy rate with ZERO requests (confirmed via Motor_tool's
        // passive bus scan before any RTR code existed); (2) enabling
        // per-tick RTR requests here (2 extra blocking can_send_rtr() calls
        // every 2.5 ms tick) coincided with hip motors 2-4 losing
        // responsiveness -- a bus scan showed hip1 (first in the per-tick
        // send order, highest CAN arbitration priority) getting far more
        // traffic through than hips 2-4, and total observed bus throughput
        // far below what 1 Mbit/s should sustain -- the signature of
        // CAN TX-mailbox contention inside ControlThread itself, not bus
        // bandwidth or a decode bug. Since (1) makes the RTR requests
        // redundant anyway, removing them is a straightforward fix rather
        // than a rate-tuning tradeoff. can_motor_request_encoder() (see
        // CANMotor.hpp) is left in place if a real need for on-demand
        // polling shows up later -- just not called from the hot path.

        chMtxLock(&state_mtx);
        memcpy(g_motor_torques, torques, sizeof(g_motor_torques));
        chMtxUnlock(&state_mtx);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * WheelSendThread — 100 Hz  NORMALPRIO+18
 *
 * Priority sits just below ControlThread (+20, hips) and above RadioThread
 * (+10) -- wheel commands matter almost as much as hip commands and must
 * never be starved by lower-priority work. In particular this sits ABOVE
 * USBCmdThread (-20): USBCmdThread's read loop can end up spinning without
 * blocking when SDU1's queue is suspended (USB unplugged -- see its own
 * comment), and previously (when this thread ran at NORMALPRIO-22, below
 * USBCmdThread) that starved wheel sends completely whenever USB was
 * disconnected -- hips kept working (ControlThread outranks USBCmdThread
 * too) but wheels went dead. Being above USBCmdThread now means that
 * failure mode can't recur regardless of USBCmdThread's own behavior.
 *
 * Sends wheel (ids 5/6) CAN commands on its own clock, independent of
 * ControlThread's hip rate -- split out 2026-09-02 rather than
 * raising ControlThread's own rate, since a wheel command is cheap (one
 * frame, no reply expected -- ODrive's encoder telemetry broadcasts on its
 * own schedule regardless, see CANMotor.cpp) where a hip command is not
 * (broadcast cmd + 4 individual replies). Reads g_motor_torques[4]/[5] --
 * already safety-gated by ControlThread's actuator_safety.apply() before
 * being published there, so no gating needed here -- for the torque-path
 * case below. Whenever robot_sm.wheel_velocity_mode() is true (ROBOT_CAR,
 * ROBOT_STANDING_UP, or ROBOT_BALANCING under BALANCE_CTRL_LQR -- read
 * unlocked, same pattern USBCmdThread's TGT,status already uses), the
 * torque path is skipped entirely in favor of RobotStateMachine's own
 * wheel_vel_L()/_R() (CarController/StandUpController/LqrBalanceController's
 * velocity targets), since RobotStateMachine::update() has already
 * switched ids 5/6 into the ODrive's own velocity mode for the duration
 * (see RobotStateMachine.cpp's wants_wheel_velocity()) -- NOTE this
 * bypasses ActuatorSafety's wheel clamp entirely, same acknowledged gap
 * CarController.hpp's header already documents for CAR mode, now shared
 * by the other two velocity-mode cases; each velocity-mode controller
 * clamps its own command instead (see e.g. StandUpController.hpp's
 * WHEEL_VEL_LIMIT_RADS). MotorTest's cached wheel_velocity_target() takes
 * the same velocity-mode path when a wheel is under test (mirrors exactly
 * what ControlThread itself used to send inline) -- mutually exclusive
 * with the above in practice, since arming (required for every velocity-
 * mode case) auto-aborts any running test.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(WheelSendThread, arg)
{
    chRegSetThreadName("wheel_send");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    systime_t next = chVTGetSystemTime();
    while (true) {
        if (robot_sm.wheel_velocity_mode()) {
            can_motor_set_velocity(5, robot_sm.wheel_vel_L());
            can_motor_set_velocity(6, robot_sm.wheel_vel_R());
        } else {
            float wheel_torques[2];
            chMtxLock(&state_mtx);
            wheel_torques[0] = g_motor_torques[4];
            wheel_torques[1] = g_motor_torques[5];
            chMtxUnlock(&state_mtx);

            can_motor_set_torque(5, wheel_torques[0]);
            can_motor_set_torque(6, wheel_torques[1]);
        }

        // A wheel under MotorTest bypasses both paths above (it commands
        // the ODrive's own velocity controller, see MotorTest.hpp) -- send
        // its held target too; this lands after whichever command was sent
        // above for the same id and simply overrides it on the wire, same
        // ordering ControlThread used before this split.
        if (g_motor_test.active() && !g_motor_test.is_hip())
            can_motor_set_velocity(g_motor_test.motor_id(), g_motor_test.wheel_velocity_target());

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * RadioThread — 100 Hz  NORMALPRIO+10
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(RadioThread, arg)
{
    chRegSetThreadName("radio");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    // Height-set hold-to-zero-on-arm: while disarmed, held true so height
    // target is forced to zero regardless of where the stick physically
    // sits. On arming, stays true (so height still reads zero) until the
    // stick is brought back through the deadband's zero zone at least once
    // -- avoids a sudden jump to whatever the stick happens to be at when
    // arming. Resets to true every cycle while disarmed, so each new arm
    // cycle requires the stick to be zeroed again.
    bool height_hold_active = true;

    // Staleness watchdog: frame_lost()/failsafe() only change value inside a
    // successfully decoded SBUS frame (see SBUS.hpp), so if the receiver
    // link dies without ever sending a failsafe-flagged frame first (cable
    // pulled, receiver powered off) those flags freeze at their last value
    // instead of going true -- radio_armed() would then read the last
    // pre-loss state forever. Mirrors StateEstThread's CAN-IMU staleness
    // counter (threads.cpp, can_stale_ticks/CAN_TIMEOUT_TICKS).
    uint32_t sbus_stale_ticks = 0;
    static constexpr uint32_t SBUS_TIMEOUT_TICKS = 50;   // 500 ms at 100 Hz

    systime_t next = chVTGetSystemTime();
    while (true) {
        radio_input_update();

        if (g_sbus.take_frame_received()) {
            sbus_stale_ticks = 0;
        } else if (++sbus_stale_ticks > SBUS_TIMEOUT_TICKS) {
            g_sbus.mark_stale();
        }

        const float height_raw = radio_height_set();
        const bool  armed_now  = radio_armed();
        if (!armed_now) {
            height_hold_active = true;
        } else if (height_hold_active && height_raw == 0.0f) {
            height_hold_active = false;
        }
        const float height_tgt = height_hold_active ? 0.0f : height_raw;

        chMtxLock(&state_mtx);
        g_input[InputIdx::YAW_STICK]   = radio_yaw_stick();
        g_input[InputIdx::VEL_TGT]     = radio_vel_tgt();
        g_input[InputIdx::HEIGHT_SET]  = height_tgt;
        g_input[InputIdx::LEANOVER]    = radio_leanover();
        g_input[InputIdx::MODE_SW]     = radio_mode_sw();
        g_armed = armed_now;
        chMtxUnlock(&state_mtx);

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * HeartbeatThread — 5 Hz  NORMALPRIO-5
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(HeartbeatThread, arg)
{
    (void)arg;
    chRegSetThreadName("heartbeat");

    uint32_t tick = 0;
    systime_t next = chVTGetSystemTime();
    while (true) {
        if (tick % 10 == 0) palSetLine(LINE_LED_ACTIVITY);
        if (tick % 10 == 1) palClearLine(LINE_LED_ACTIVITY);
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, TIME_MS2I(200)));
        tick++;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * USBCmdThread — event-driven  NORMALPRIO-20
 * Commands: BOOT, LOG,*, CAL,*, CAN,*, STRAIN_RATE,read, MOTOR,status,
 *           MOTOR,<id>,request (RMD: 0x9A status; ODRIVE: RTR encoder read),
 *           MOTOR,test,<id>,start / MOTOR,test,stop, POWER,status,
 *           RC,status, TGT,status
 * ══════════════════════════════════════════════════════════════════════════ */

static void usb_log_list(void)
{
    DIR     dir;
    FILINFO fno;
    if (f_opendir(&dir, "/LOGS") != FR_OK) {
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "LOG,ERR,no_sd\r\n");
        chMtxUnlock(&s_usb_write_mtx);
        return;
    }
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        if (fno.fattrib & AM_DIR) continue;
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1,
                 "LOG,FILE,%s,%lu\r\n", fno.fname, (uint32_t)fno.fsize);
        chMtxUnlock(&s_usb_write_mtx);
    }
    f_closedir(&dir);
    chMtxLock(&s_usb_write_mtx);
    chprintf((BaseSequentialStream *)&SDU1, "LOG,LIST,END\r\n");
    chMtxUnlock(&s_usb_write_mtx);
}

static void usb_log_get(const char *fname)
{
    char path[32];
    snprintf(path, sizeof(path), "/LOGS/%s", fname);
    FILINFO fno;
    if (f_stat(path, &fno) != FR_OK) {
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "LOG,ERR,notfound\r\n");
        chMtxUnlock(&s_usb_write_mtx);
        return;
    }
    FIL fil;
    if (f_open(&fil, path, FA_READ) != FR_OK) {
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "LOG,ERR,open_failed\r\n");
        chMtxUnlock(&s_usb_write_mtx);
        return;
    }
    chMtxLock(&s_usb_write_mtx);
    chprintf((BaseSequentialStream *)&SDU1, "LOG,SIZE,%lu\r\n", (uint32_t)fno.fsize);
    UINT br;
    while (f_read(&fil, s_usb_dl_buf, sizeof(s_usb_dl_buf), &br) == FR_OK && br > 0)
        chnWriteTimeout((BaseChannel *)&SDU1, s_usb_dl_buf, br, TIME_MS2I(2000));
    chMtxUnlock(&s_usb_write_mtx);
    f_close(&fil);
}

static void usb_log_erase(void)
{
    DIR     dir;
    FILINFO fno;
    char    path[32];
    int     erased = 0;
    const char *cur    = logger.current_path();
    const char *cur_fn = nullptr;
    if (cur) { cur_fn = strrchr(cur, '/'); if (cur_fn) cur_fn++; }
    if (f_opendir(&dir, "/LOGS") != FR_OK) {
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "LOG,ERR,no_sd\r\n");
        chMtxUnlock(&s_usb_write_mtx);
        return;
    }
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        if (fno.fattrib & AM_DIR) continue;
        if (cur_fn && strcmp(fno.fname, cur_fn) == 0) continue;
        snprintf(path, sizeof(path), "/LOGS/%s", fno.fname);
        if (f_unlink(path) == FR_OK) erased++;
    }
    f_closedir(&dir);
    chMtxLock(&s_usb_write_mtx);
    chprintf((BaseSequentialStream *)&SDU1, "LOG,ERASED,%d\r\n", erased);
    chMtxUnlock(&s_usb_write_mtx);
}

static CalibData s_cal_stage = {};

static void usb_cmd_dispatch(const char *line)
{
    if (strcmp(line, "BOOT") == 0) {
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "BOOT,OK\r\n");
        chMtxUnlock(&s_usb_write_mtx);
        chThdSleepMilliseconds(50);
        NVIC_SystemReset();
    } else if (strncmp(line, "LOG,", 4) == 0) {
        const char *rest = line + 4;
        if (strcmp(rest, "status") == 0) {
            static const char *const err_str[] = {
                "not_tried", "sdcStart", "sdcConnect", "f_mount", "f_open", "ok"
            };
            uint8_t e  = logger.last_init_err();
            uint8_t ff = logger.last_ff_err();
            const char *es = (e <= 5) ? err_str[e] : "unknown";
            chMtxLock(&s_usb_write_mtx);
            if (logger.is_ready()) {
                chprintf((BaseSequentialStream *)&SDU1,
                         "LOG,STATUS,ready,file=%s\r\n", logger.current_path());
            } else if (ff != 0) {
                chprintf((BaseSequentialStream *)&SDU1,
                         "LOG,STATUS,not_ready,last_err=%u(%s),ff=%u\r\n",
                         (unsigned)e, es, (unsigned)ff);
            } else {
                chprintf((BaseSequentialStream *)&SDU1,
                         "LOG,STATUS,not_ready,last_err=%u(%s)\r\n", (unsigned)e, es);
            }
            chMtxUnlock(&s_usb_write_mtx);
        } else if (strcmp(rest, "list") == 0) {
            usb_log_list();
        } else if (strncmp(rest, "get,", 4) == 0) {
            usb_log_get(rest + 4);
        } else if (strcmp(rest, "erase") == 0) {
            usb_log_erase();
        } else {
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "LOG,ERR,unknown_cmd\r\n");
            chMtxUnlock(&s_usb_write_mtx);
        }
    } else if (strncmp(line, "CAL,", 4) == 0) {
        const char *rest = line + 4;
        if (strncmp(rest, "set,", 4) == 0) {
            int   idx = -1;
            float gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
            if (sscanf(rest + 4, "%d,%f,%f,%f,%f,%f,%f",
                       &idx, &gx, &gy, &gz, &ax, &ay, &az) == 7
                && idx >= 0 && idx <= 2)
            {
                s_cal_stage.gyro_bias[idx][0]  = gx;
                s_cal_stage.gyro_bias[idx][1]  = gy;
                s_cal_stage.gyro_bias[idx][2]  = gz;
                s_cal_stage.accel_bias[idx][0] = ax;
                s_cal_stage.accel_bias[idx][1] = ay;
                s_cal_stage.accel_bias[idx][2] = az;
                chMtxLock(&s_usb_write_mtx);
                chprintf((BaseSequentialStream *)&SDU1, "CAL,SET,%d,OK\r\n", idx);
                chMtxUnlock(&s_usb_write_mtx);
            } else {
                chMtxLock(&s_usb_write_mtx);
                chprintf((BaseSequentialStream *)&SDU1, "CAL,ERR,bad_args\r\n");
                chMtxUnlock(&s_usb_write_mtx);
            }
        } else if (strcmp(rest, "commit") == 0) {
            bool ok = cal_save(s_cal_stage);
            if (ok) { g_cal = s_cal_stage; g_cal_valid = true; }
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1,
                     ok ? "CAL,OK\r\n" : "CAL,ERR,write_failed\r\n");
            chMtxUnlock(&s_usb_write_mtx);
        } else if (strcmp(rest, "clear") == 0) {
            cal_clear();
            memset(&g_cal, 0, sizeof(g_cal));
            g_cal_valid = false;
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "CAL,OK\r\n");
            chMtxUnlock(&s_usb_write_mtx);
        } else if (strcmp(rest, "query") == 0) {
            CalibData stored = {};
            bool valid = cal_load(stored);
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1,
                "CAL,DATA,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\r\n",
                (double)stored.gyro_bias[0][0], (double)stored.gyro_bias[0][1],
                (double)stored.gyro_bias[0][2], (double)stored.gyro_bias[1][0],
                (double)stored.gyro_bias[1][1], (double)stored.gyro_bias[1][2],
                (double)stored.gyro_bias[2][0], (double)stored.gyro_bias[2][1],
                (double)stored.gyro_bias[2][2],
                (double)stored.accel_bias[0][0], (double)stored.accel_bias[0][1],
                (double)stored.accel_bias[0][2], (double)stored.accel_bias[1][0],
                (double)stored.accel_bias[1][1], (double)stored.accel_bias[1][2],
                (double)stored.accel_bias[2][0], (double)stored.accel_bias[2][1],
                (double)stored.accel_bias[2][2], (int)valid);
            chMtxUnlock(&s_usb_write_mtx);
        } else {
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "CAL,ERR,unknown_cmd\r\n");
            chMtxUnlock(&s_usb_write_mtx);
        }
    } else if (strcmp(line, "CAN,status") == 0) {
        const uint32_t psr   = FDCAN1->PSR;
        const uint32_t ecr   = FDCAN1->ECR;
        const uint32_t rxf0s = FDCAN1->RXF0S;
        const uint32_t cccr  = FDCAN1->CCCR;
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1,
                 "CAN,STATUS,psr=0x%08x,ecr=0x%08x,rxf0s=0x%08x,cccr=0x%08x\r\n",
                 (unsigned)psr, (unsigned)ecr, (unsigned)rxf0s, (unsigned)cccr);
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "CAN,diag") == 0) {
        CANDiag d = {};
        can_get_diag(CAN_BUS_1, d);
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1,
                 "CAN,DIAG,bus1,total_rx=%lu,dispatched=%lu,"
                 "last_sid=0x%03x,last_dlc=%u,tx_ok=%lu,tx_fail=%lu,"
                 "tec=%u,rec=%u,tec_peak=%u,rec_peak=%u,busoff_count=%lu,rx_fifo_lost=%lu\r\n",
                 (uint32_t)d.total_rx, (uint32_t)d.dispatched,
                 (unsigned)d.last_sid, (unsigned)d.last_dlc,
                 (uint32_t)d.tx_ok, (uint32_t)d.tx_fail,
                 (unsigned)d.tec, (unsigned)d.rec, (unsigned)d.tec_peak, (unsigned)d.rec_peak,
                 (uint32_t)d.busoff_count, (uint32_t)d.rx_fifo_lost);
        can_get_diag(CAN_BUS_2, d);
        chprintf((BaseSequentialStream *)&SDU1,
                 "CAN,DIAG,bus2,total_rx=%lu,dispatched=%lu,"
                 "last_sid=0x%03x,last_dlc=%u,tx_ok=%lu,tx_fail=%lu,"
                 "tec=%u,rec=%u,tec_peak=%u,rec_peak=%u,busoff_count=%lu,rx_fifo_lost=%lu\r\n",
                 (uint32_t)d.total_rx, (uint32_t)d.dispatched,
                 (unsigned)d.last_sid, (unsigned)d.last_dlc,
                 (uint32_t)d.tx_ok, (uint32_t)d.tx_fail,
                 (unsigned)d.tec, (unsigned)d.rec, (unsigned)d.tec_peak, (unsigned)d.rec_peak,
                 (uint32_t)d.busoff_count, (uint32_t)d.rx_fifo_lost);
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "CAN,scan,start") == 0) {
        can_scan_start();
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1, "CAN,SCAN,started\r\n");
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "CAN,scan,stop") == 0) {
        can_scan_stop();
        CANScanEntry entries[CAN_SCAN_MAX];
        int n = can_scan_get(entries, CAN_SCAN_MAX);
        chMtxLock(&s_usb_write_mtx);
        for (int i = 0; i < n; i++) {
            chprintf((BaseSequentialStream *)&SDU1,
                     "CAN,SCAN,id=%s0x%03lx,count=%lu\r\n",
                     entries[i].is_ext ? "EXT:" : "",
                     (uint32_t)entries[i].id, (uint32_t)entries[i].count);
        }
        chprintf((BaseSequentialStream *)&SDU1, "CAN,SCAN,END\r\n");
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "CAN,regdump") == 0) {
        CANRegEntry regs[12];
        int n = can_read_regs(regs, 12);
        chMtxLock(&s_usb_write_mtx);
        for (int i = 0; i < n; i++) {
            chprintf((BaseSequentialStream *)&SDU1,
                     "CAN,REG,%s=0x%08lx\r\n", regs[i].name, (uint32_t)regs[i].value);
        }
        chprintf((BaseSequentialStream *)&SDU1, "CAN,REG,END\r\n");
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "MOTOR,status") == 0) {
        // Print current motor state for all registered motors (IDs 1–6).
        // pos_rad for hips (1-4) is already robot-frame -- offset applied
        // at decode time, see CANMotor.cpp's "Hip zero-offset" section.
        chMtxLock(&s_usb_write_mtx);
        for (uint8_t id = 1; id <= 6; id++) {
            CanMotorState ms = {};
            if (can_motor_get_state(id, &ms)) {
                chprintf((BaseSequentialStream *)&SDU1,
                         "MOTOR,%u,pos=%.3f,vel=%.3f,tq=%.3f,temp=%.1f,v=%d,"
                         "tx_ok=%lu,tx_fail=%lu,rx=%lu\r\n",
                         (unsigned)id, (double)ms.pos_rad, (double)ms.vel_rads,
                         (double)ms.torque_Nm, (double)ms.temp_C, (int)ms.valid,
                         (uint32_t)ms.tx_ok, (uint32_t)ms.tx_fail, (uint32_t)ms.rx_count);
            }
        }
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strncmp(line, "MOTOR,", 6) == 0 && strstr(line, ",request") != nullptr) {
        // MOTOR,<id>,request -- actively trigger one fresh read instead of
        // waiting on whatever arrives passively. RMD (hips, 1-4): 0x9A
        // status request. ODRIVE (wheels, 5-6): RTR frame for
        // Get_Encoder_Estimates (can_motor_request_encoder() -- the same
        // function ControlThread used to call every tick before that was
        // found to overload the bus, see telemetry_plan.md item C; calling
        // it on demand from a slow ground-tool poll (~10-20 Hz) is a
        // completely different load and does not reintroduce that problem.
        unsigned id;
        bool handled = false;
        if (sscanf(line + 6, "%u,request", &id) == 1 && id >= 1 && id <= 6) {
            bool ok = (id <= 4) ? can_motor_request_status((uint8_t)id)
                                 : can_motor_request_encoder((uint8_t)id);
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "MOTOR,%u,REQUEST,%s\r\n",
                     id, ok ? "OK" : "ERR");
            chMtxUnlock(&s_usb_write_mtx);
            handled = true;
        }
        if (!handled) {
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "MOTOR,ERR,bad_request\r\n");
            chMtxUnlock(&s_usb_write_mtx);
        }
    } else if (strncmp(line, "MOTOR,test,", 11) == 0) {
        const char *rest = line + 11;
        unsigned id;
        if (sscanf(rest, "%u,start", &id) == 1 && id >= 1 && id <= 6) {
            bool ok = g_motor_test.start((uint8_t)id);
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "MOTOR,TEST,%s,id=%u\r\n",
                     ok ? "STARTED" : "ERR", id);
            if (ok) {
                chprintf((BaseSequentialStream *)&SDU1,
                         "MOTOR,TEST,NOTE,disarmed only -- arming stops the test automatically\r\n");
            }
            chMtxUnlock(&s_usb_write_mtx);
        } else if (strcmp(rest, "stop") == 0) {
            g_motor_test.stop();
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "MOTOR,TEST,STOPPED\r\n");
            chMtxUnlock(&s_usb_write_mtx);
        } else {
            chMtxLock(&s_usb_write_mtx);
            chprintf((BaseSequentialStream *)&SDU1, "MOTOR,TEST,ERR,unknown_cmd\r\n");
            chMtxUnlock(&s_usb_write_mtx);
        }
    } else if (strcmp(line, "POWER,status") == 0) {
        chMtxLock(&power_mtx);
        PowerMonState pwr = g_power;
        chMtxUnlock(&power_mtx);
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1,
                 "POWER,STATUS,valid=%d,node=%u,v=%.3f,i=%.3f\r\n",
                 (int)pwr.valid, (unsigned)pwr.node_id,
                 (double)pwr.voltage_V, (double)pwr.current_A);
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "RC,status") == 0) {
        chMtxLock(&state_mtx);
        bool armed = g_armed;
        chMtxUnlock(&state_mtx);
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1,
                 "RC,STATUS,frame_lost=%d,failsafe=%d,armed=%d",
                 (int)g_sbus.frame_lost(), (int)g_sbus.failsafe(), (int)armed);
        for (int ch = 0; ch < 16; ch++) {
            chprintf((BaseSequentialStream *)&SDU1, ",ch%d=%u",
                     ch, (unsigned)g_sbus.channel(ch));
        }
        chprintf((BaseSequentialStream *)&SDU1, "\r\n");
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "TGT,status") == 0) {
        float yaw_tgt, vel_tgt, height_tgt, lean_tgt, mode_sw;
        bool  armed;
        chMtxLock(&state_mtx);
        yaw_tgt    = g_input[InputIdx::YAW_STICK];
        vel_tgt    = g_input[InputIdx::VEL_TGT];
        height_tgt = g_input[InputIdx::HEIGHT_SET];
        lean_tgt   = g_input[InputIdx::LEANOVER];
        mode_sw    = g_input[InputIdx::MODE_SW];
        armed      = g_armed;
        chMtxUnlock(&state_mtx);

        RobotMode   mode = robot_sm.mode();
        const char *mode_name = (mode == ROBOT_IDLE)        ? "IDLE"
                               : (mode == ROBOT_STANDING_UP) ? "STANDING_UP"
                               : (mode == ROBOT_BALANCING)   ? "BALANCING"
                                                              : "CAR";
        chMtxLock(&s_usb_write_mtx);
        chprintf((BaseSequentialStream *)&SDU1,
                 "TGT,STATUS,armed=%d,mode_sw=%.3f,mode=%d,mode_name=%s,"
                 "vel_tgt=%.3f,yaw_tgt=%.3f,height_tgt=%.3f,lean_tgt=%.3f\r\n",
                 (int)armed, (double)mode_sw, (int)mode, mode_name,
                 (double)vel_tgt, (double)yaw_tgt, (double)height_tgt, (double)lean_tgt);
        chMtxUnlock(&s_usb_write_mtx);
    }
}

static THD_FUNCTION(USBCmdThread, arg)
{
    chRegSetThreadName("usbcmd");
    (void)arg;

    static char    s_line[64];
    static uint8_t s_len = 0;

    while (true) {
        msg_t byte = chnGetTimeout((BaseChannel *)&SDU1, TIME_MS2I(50));
        if (byte == MSG_TIMEOUT) continue;
        if (byte == MSG_RESET) {
            // SDU1's input queue is suspended (USB unplugged/unconfigured --
            // see sduSuspendHookI()/bqSuspendI() in hal_serial_usb.c). Once
            // suspended, chnGetTimeout() stops honoring its timeout and
            // returns MSG_RESET immediately on every call, which would
            // otherwise turn this into a 100%-CPU busy loop at this thread's
            // priority (NORMALPRIO-20) -- starving every lower-priority
            // thread, in particular WheelSendThread (NORMALPRIO-22, the only
            // thread below this one) so wheel CAN commands silently stop
            // going out while everything above -20 (hips, radio, etc.) keeps
            // running fine. Sleep explicitly so this thread actually yields
            // while the link is down.
            chThdSleepMilliseconds(50);
            continue;
        }
        char c = (char)byte;
        if (c == '\n' || c == '\r') {
            if (s_len > 0) {
                s_line[s_len] = '\0';
                usb_cmd_dispatch(s_line);
                s_len = 0;
            }
        } else if (s_len < (uint8_t)(sizeof(s_line) - 1U)) {
            s_line[s_len++] = c;
        } else {
            s_len = 0;
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * DebugThread — 10 Hz  NORMALPRIO-10  [BPRL_DEBUG only]
 * Streams $IMU, $TEL, and $EKFL telemetry lines over USB CDC.
 * ══════════════════════════════════════════════════════════════════════════ */
#ifdef BPRL_DEBUG
static THD_FUNCTION(DebugThread, arg)
{
    chRegSetThreadName("dbg");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    uint32_t prev_quat_cnt = 0, prev_rate_cnt = 0;
    uint32_t can_quat_hz   = 0, can_rate_hz   = 0;
    int      rate_tick     = 0;

    systime_t next = chVTGetSystemTime();
    while (true) {
        if (++rate_tick >= 10) {
            uint32_t qc = s_can_quat_cnt, rc = s_can_rate_cnt;
            can_quat_hz   = qc - prev_quat_cnt;
            can_rate_hz   = rc - prev_rate_cnt;
            prev_quat_cnt = qc;
            prev_rate_cnt = rc;
            rate_tick     = 0;
        }

        float roll, pitch, yaw, vel_tgt, height_set, leanover, yaw_stick;
        float p, q, r;
        float leg_L, leg_L_dot, leg_pitch, leg_pitch_dot;
        float body_u, body_w;
        float lane_roll[3], lane_pitch[3], lane_yaw[3];
        float lane_p[3],    lane_q[3],    lane_r[3];
        int   primary_lane;
        bool  armed;
        chMtxLock(&state_mtx);
        roll     = g_euler[0];
        pitch    = g_euler[1];
        yaw      = g_euler[2];
        p        = g_state[StateIdx::P];
        q        = g_state[StateIdx::Q];
        r        = g_state[StateIdx::R];
        leg_L         = g_state[StateIdx::LEG_L];
        leg_L_dot     = g_state[StateIdx::LEG_L_DOT];
        leg_pitch     = g_state[StateIdx::LEG_PITCH];
        leg_pitch_dot = g_state[StateIdx::LEG_PITCH_DOT];
        body_u   = g_state[StateIdx::U];
        body_w   = g_state[StateIdx::W];
        vel_tgt    = g_input[InputIdx::VEL_TGT];
        height_set = g_input[InputIdx::HEIGHT_SET];
        leanover   = g_input[InputIdx::LEANOVER];
        yaw_stick  = g_input[InputIdx::YAW_STICK];
        armed      = g_armed;
        for (int li = 0; li < 3; ++li) {
            lane_roll[li]  = s_lane_roll[li];
            lane_pitch[li] = s_lane_pitch[li];
            lane_yaw[li]   = s_lane_yaw[li];
            lane_p[li]     = s_lane_p[li];
            lane_q[li]     = s_lane_q[li];
            lane_r[li]     = s_lane_r[li];
        }
        primary_lane = s_primary_lane;
        chMtxUnlock(&state_mtx);

        float wheel_vel[2];
        chMtxLock(&telemetry_mtx);
        wheel_vel[0] = g_telemetry.wheel_vel_rads[0];
        wheel_vel[1] = g_telemetry.wheel_vel_rads[1];
        chMtxUnlock(&telemetry_mtx);

        float imu_ax[3], imu_ay[3], imu_az[3];
        float imu_gx[3], imu_gy[3], imu_gz[3];
        bool  imu_v[3];
        chMtxLock(&imu_mtx);
        for (int _i = 0; _i < 3; _i++) {
            imu_ax[_i] = g_imu[_i].accel[0]; imu_ay[_i] = g_imu[_i].accel[1];
            imu_az[_i] = g_imu[_i].accel[2];
            imu_gx[_i] = g_imu[_i].gyro[0];  imu_gy[_i] = g_imu[_i].gyro[1];
            imu_gz[_i] = g_imu[_i].gyro[2];
            imu_v[_i]  = g_imu[_i].valid;
        }
        chMtxUnlock(&imu_mtx);

        float can_p_snap, can_q_snap, can_r_snap;
        float can_qw, can_qx, can_qy, can_qz;
        bool  can_v;
        chMtxLock(&can_imu_mtx);
        can_v = g_can_imu.valid;
        can_p_snap = g_can_imu.p; can_q_snap = g_can_imu.q; can_r_snap = g_can_imu.r;
        can_qw = g_can_imu.q0; can_qx = g_can_imu.q1;
        can_qy = g_can_imu.q2; can_qz = g_can_imu.q3;
        chMtxUnlock(&can_imu_mtx);

        float roll_r  = atan2f(2.0f*(can_qw*can_qx + can_qy*can_qz),
                                1.0f - 2.0f*(can_qx*can_qx + can_qy*can_qy));
        float pitch_r = asinf(fmaxf(-1.0f, fminf(1.0f,
                                2.0f*(can_qw*can_qy - can_qz*can_qx))));
        float yaw_r   = atan2f(2.0f*(can_qw*can_qz + can_qx*can_qy),
                                1.0f - 2.0f*(can_qy*can_qy + can_qz*can_qz));
        float can_roll_deg  = roll_r  * 57.2958f;
        float can_pitch_deg = pitch_r * 57.2958f;
        float can_yaw_deg   = yaw_r   * 57.2958f;

        {
            static char imu_buf[320];
            MemoryStream ims;
            msObjectInit(&ims, (uint8_t *)imu_buf, sizeof(imu_buf) - 1, 0);
            chprintf((BaseSequentialStream *)&ims,
                "$IMU,%lu,"
                "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%d,"
                "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%d,"
                "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%d,"
                "%.4f,%.4f,%.4f,%d\r\n",
                (uint32_t)TIME_I2MS(chVTGetSystemTime()),
                (double)imu_ax[0], (double)imu_ay[0], (double)imu_az[0],
                (double)imu_gx[0], (double)imu_gy[0], (double)imu_gz[0], (int)imu_v[0],
                (double)imu_ax[1], (double)imu_ay[1], (double)imu_az[1],
                (double)imu_gx[1], (double)imu_gy[1], (double)imu_gz[1], (int)imu_v[1],
                (double)imu_ax[2], (double)imu_ay[2], (double)imu_az[2],
                (double)imu_gx[2], (double)imu_gy[2], (double)imu_gz[2], (int)imu_v[2],
                (double)can_p_snap, (double)can_q_snap, (double)can_r_snap, (int)can_v);
            size_t ilen = ims.eos;
            if (ilen > 0 && chMtxTryLock(&s_usb_write_mtx)) {
                chnWriteTimeout((BaseChannel *)&SDU1,
                                (uint8_t *)imu_buf, ilen, TIME_MS2I(50));
                chMtxUnlock(&s_usb_write_mtx);
            }
        }

        {
            static char tel_buf[256];
            MemoryStream ms;
            msObjectInit(&ms, (uint8_t *)tel_buf, sizeof(tel_buf) - 1, 0);
            chprintf((BaseSequentialStream *)&ms,
                "$TEL,%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%d,"
                "%d,%d,%d,%d,%lu,%lu,"
                "%.3f,%.3f,%.2f,%.4f,%.3f,%.3f,%.3f,%.3f\r\n",
                (uint32_t)TIME_I2MS(chVTGetSystemTime()),
                (double)(roll*57.2958f), (double)(pitch*57.2958f), (double)(yaw*57.2958f),
                (double)p, (double)q, (double)r,
                (double)vel_tgt, (double)height_set, (double)leanover, (double)yaw_stick,
                (int)armed,
                (int)imu_v[0], (int)imu_v[1], (int)imu_v[2], (int)can_v,
                can_quat_hz, can_rate_hz,
                (double)leg_L, (double)leg_L_dot,
                (double)(leg_pitch*57.2958f), (double)leg_pitch_dot,
                (double)wheel_vel[0], (double)wheel_vel[1],
                (double)body_u, (double)body_w);
            size_t tlen = ms.eos;
            if (tlen > 0 && chMtxTryLock(&s_usb_write_mtx)) {
                chnWriteTimeout((BaseChannel *)&SDU1,
                                (uint8_t *)tel_buf, tlen, TIME_MS2I(50));
                chMtxUnlock(&s_usb_write_mtx);
            }
        }

        {
            static char ekfl_buf[256];
            MemoryStream ekfl_ms;
            msObjectInit(&ekfl_ms, (uint8_t *)ekfl_buf, sizeof(ekfl_buf) - 1, 0);
            chprintf((BaseSequentialStream *)&ekfl_ms,
                "$EKFL,%lu,%d,"
                "%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,"
                "%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,"
                "%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,"
                "%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,"
                "%d,%d,%d,%d\r\n",
                (uint32_t)TIME_I2MS(chVTGetSystemTime()), primary_lane,
                (double)(lane_roll[0]*57.2958f), (double)(lane_pitch[0]*57.2958f),
                (double)(lane_yaw[0]*57.2958f), (double)lane_p[0],
                (double)lane_q[0], (double)lane_r[0],
                (double)(lane_roll[1]*57.2958f), (double)(lane_pitch[1]*57.2958f),
                (double)(lane_yaw[1]*57.2958f), (double)lane_p[1],
                (double)lane_q[1], (double)lane_r[1],
                (double)(lane_roll[2]*57.2958f), (double)(lane_pitch[2]*57.2958f),
                (double)(lane_yaw[2]*57.2958f), (double)lane_p[2],
                (double)lane_q[2], (double)lane_r[2],
                (double)can_roll_deg, (double)can_pitch_deg, (double)can_yaw_deg,
                (double)can_p_snap, (double)can_q_snap, (double)can_r_snap,
                (int)imu_v[0], (int)imu_v[1], (int)imu_v[2], (int)can_v);
            size_t elen = ekfl_ms.eos;
            if (elen > 0 && chMtxTryLock(&s_usb_write_mtx)) {
                chnWriteTimeout((BaseChannel *)&SDU1,
                                (uint8_t *)ekfl_buf, elen, TIME_MS2I(50));
                chMtxUnlock(&s_usb_write_mtx);
            }
        }

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}
#endif  // BPRL_DEBUG

/* ══════════════════════════════════════════════════════════════════════════
 * LogThread — 50 Hz  NORMALPRIO-15
 * Logs ATT, LIN, RCIN, OUTP per tick.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(LogThread, arg)
{
    chRegSetThreadName("log");
    const LogRates *rates = static_cast<const LogRates *>(arg);

    while (!logger.init())
        chThdSleepMilliseconds(5000);

    systime_t next = chVTGetSystemTime();
    while (true) {
        const uint64_t t_us = (uint64_t)TIME_I2MS(chVTGetSystemTime()) * 1000ULL;

        float euler[3], state[StateIdx::N], inp[InputIdx::N_INPUTS], torques[6];
        bool  armed;
        chMtxLock(&state_mtx);
        memcpy(euler,   g_euler,         sizeof(euler));
        memcpy(state,   g_state,         sizeof(state));
        memcpy(inp,     g_input,         sizeof(inp));
        memcpy(torques, g_motor_torques, sizeof(torques));
        armed = g_armed;
        chMtxUnlock(&state_mtx);

        /* ATT */
        {
            LogMsgATT msg = {};
            msg.time_us = t_us; msg.rate_hz = 50U;
            msg.roll    = euler[0]; msg.pitch = euler[1]; msg.yaw = euler[2];
            msg.p       = state[StateIdx::P];
            msg.q       = state[StateIdx::Q];
            msg.r       = state[StateIdx::R];
            msg.p_dot   = state[StateIdx::P_DOT];
            msg.q_dot   = state[StateIdx::Q_DOT];
            msg.r_dot   = state[StateIdx::R_DOT];
            logger.write(LOG_MSG_ATT, msg);
        }

        /* LIN */
        {
            LogMsgLIN msg = {};
            msg.time_us = t_us; msg.rate_hz = 50U;
            msg.x       = state[StateIdx::X];
            msg.y       = state[StateIdx::Y];
            msg.z       = state[StateIdx::Z_POS];
            msg.u       = state[StateIdx::U];
            msg.v       = state[StateIdx::V];
            msg.w       = state[StateIdx::W];
            msg.u_dot   = state[StateIdx::U_DOT];
            msg.v_dot   = state[StateIdx::V_DOT];
            msg.w_dot   = state[StateIdx::W_DOT];
            logger.write(LOG_MSG_LIN, msg);
        }

        /* RCIN */
        {
            LogMsgRCIN msg = {};
            msg.time_us   = t_us; msg.rate_hz = 50U;
            msg.yaw_stk    = inp[InputIdx::YAW_STICK];
            msg.vel_stk    = inp[InputIdx::VEL_TGT];
            msg.height_stk = inp[InputIdx::HEIGHT_SET];
            msg.lean_stk   = inp[InputIdx::LEANOVER];
            msg.armed     = (uint8_t)armed;
            msg.mode      = (uint8_t)robot_sm.mode();
            logger.write(LOG_MSG_RCIN, msg);
        }

        /* OUTP — motor torque commands */
        {
            LogMsgOUTP msg = {};
            msg.time_us  = t_us; msg.rate_hz = 50U;
            msg.tq0 = torques[0];
            msg.tq1 = torques[1];
            msg.tq2 = torques[2];
            msg.tq3 = torques[3];
            logger.write(LOG_MSG_OUTP, msg);
        }

        /* HIPS / HERR — hip encoder position, bus voltage, latched error
         * flags (CanMotorState::pos_rad/voltage_V/error_flags, ids 1-4).
         * voltage_V/error_flags only refresh at the slow rate
         * can_motor_poll_status_round_robin() polls at (see ControlThread) —
         * everything else here (pos_rad) updates every control tick. */
        {
            CanMotorState hip[4] = {};
            for (int i = 0; i < 4; i++)
                can_motor_get_state((uint8_t)(i + 1), &hip[i]);

            LogMsgHIPS hips_msg = {};
            hips_msg.time_us = t_us; hips_msg.rate_hz = 50U;
            hips_msg.pos0 = hip[0].pos_rad;   hips_msg.volt0 = hip[0].voltage_V;
            hips_msg.pos1 = hip[1].pos_rad;   hips_msg.volt1 = hip[1].voltage_V;
            hips_msg.pos2 = hip[2].pos_rad;   hips_msg.volt2 = hip[2].voltage_V;
            hips_msg.pos3 = hip[3].pos_rad;   hips_msg.volt3 = hip[3].voltage_V;
            logger.write(LOG_MSG_HIPS, hips_msg);

            LogMsgHERR herr_msg = {};
            herr_msg.time_us = t_us; herr_msg.rate_hz = 50U;
            herr_msg.err0 = hip[0].error_flags;
            herr_msg.err1 = hip[1].error_flags;
            herr_msg.err2 = hip[2].error_flags;
            herr_msg.err3 = hip[3].error_flags;
            logger.write(LOG_MSG_HERR, herr_msg);
        }

        /* LEGS / LEGV / MOTV / PWR / VNED — everything RobotTelemetry
         * consolidates (per-leg FK, individual hip/wheel velocity, battery
         * voltage/current, world-frame velocity). Snapshot once under
         * telemetry_mtx, same pattern as the g_state/g_input snapshot at
         * the top of this loop. */
        {
            RobotTelemetry tel;
            chMtxLock(&telemetry_mtx);
            tel = g_telemetry;
            chMtxUnlock(&telemetry_mtx);

            LogMsgLEGS legs_msg = {};
            legs_msg.time_us = t_us; legs_msg.rate_hz = 50U;
            legs_msg.L0 = tel.leg_L[0]; legs_msg.L0dot = tel.leg_L_dot[0];
            legs_msg.ThL0 = tel.leg_thL[0]; legs_msg.ThL0dot = tel.leg_thL_dot[0];
            legs_msg.L1 = tel.leg_L[1]; legs_msg.L1dot = tel.leg_L_dot[1];
            legs_msg.ThL1 = tel.leg_thL[1]; legs_msg.ThL1dot = tel.leg_thL_dot[1];
            logger.write(LOG_MSG_LEGS, legs_msg);

            LogMsgLEGV legv_msg = {};
            legv_msg.time_us = t_us; legv_msg.rate_hz = 50U;
            legv_msg.leg0_xdot = tel.leg_x_dot[0]; legv_msg.leg0_zdot = tel.leg_z_dot[0];
            legv_msg.leg1_xdot = tel.leg_x_dot[1]; legv_msg.leg1_zdot = tel.leg_z_dot[1];
            logger.write(LOG_MSG_LEGV, legv_msg);

            LogMsgMOTV motv_msg = {};
            motv_msg.time_us = t_us; motv_msg.rate_hz = 50U;
            motv_msg.hip_vel0 = tel.hip_vel_rads[0]; motv_msg.hip_vel1 = tel.hip_vel_rads[1];
            motv_msg.hip_vel2 = tel.hip_vel_rads[2]; motv_msg.hip_vel3 = tel.hip_vel_rads[3];
            motv_msg.wheel_vel0 = tel.wheel_vel_rads[0]; motv_msg.wheel_vel1 = tel.wheel_vel_rads[1];
            logger.write(LOG_MSG_MOTV, motv_msg);

            LogMsgPWR pwr_msg = {};
            pwr_msg.time_us = t_us; pwr_msg.rate_hz = 50U;
            pwr_msg.batt_v  = tel.battery_voltage_V;
            pwr_msg.pmon_v  = tel.power_monitor_voltage_V;
            pwr_msg.total_i = tel.total_current_A;
            logger.write(LOG_MSG_PWR, pwr_msg);

            LogMsgVNED vned_msg = {};
            vned_msg.time_us = t_us; vned_msg.rate_hz = 50U;
            vned_msg.xdot_world = tel.x_dot;
            vned_msg.zdot_world = tel.z_dot;
            logger.write(LOG_MSG_VNED, vned_msg);
        }

        logger.flush();
        next = chThdSleepUntilWindowed(next, chTimeAddX(next, rates->period));
    }
}

/* ── Thread launcher ─────────────────────────────────────────────────────── */
void threads_start(const ThreadRates &rates)
{
    chThdCreateStatic(waSPI,       sizeof(waSPI),       NORMALPRIO + 30, SPIThread,       (void *)&rates.spi);
    chThdCreateStatic(waCAN,       sizeof(waCAN),       NORMALPRIO + 28, CANThread,       nullptr);
    chThdCreateStatic(waStateEst,  sizeof(waStateEst),  NORMALPRIO + 25, StateEstThread,  (void *)&rates.est);
    chThdCreateStatic(waControl,   sizeof(waControl),   NORMALPRIO + 20, ControlThread,   (void *)&rates.control);
    chThdCreateStatic(waWheelSend, sizeof(waWheelSend), NORMALPRIO + 18, WheelSendThread, (void *)&rates.wheel_send);
    chThdCreateStatic(waRadio,     sizeof(waRadio),     NORMALPRIO + 10, RadioThread,     (void *)&rates.radio);
    chThdCreateStatic(waHeartbeat, sizeof(waHeartbeat), NORMALPRIO -  5, HeartbeatThread, (void *)&rates.heartbeat);
#ifdef BPRL_DEBUG
    chThdCreateStatic(waDebug,     sizeof(waDebug),     NORMALPRIO - 10, DebugThread,     (void *)&rates.debug);
#endif
    chThdCreateStatic(waUSBCmd,    sizeof(waUSBCmd),    NORMALPRIO - 20, USBCmdThread,    nullptr);
    chThdCreateStatic(waLog,       sizeof(waLog),       NORMALPRIO - 15, LogThread,       (void *)&rates.log);
}
