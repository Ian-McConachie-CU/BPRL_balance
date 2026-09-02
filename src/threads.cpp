/*
 * threads.cpp — BPRL_Balance thread function definitions.
 *
 * Thread overview:
 *   SPIThread      +30  1 kHz    Read three onboard IMUs via SPI
 *   CANThread      +28  event    Receive frames from FDCAN1 and FDCAN2
 *   StateEstThread +25  500 Hz   3-lane EKF → g_state[]
 *   ControlThread  +20  400 Hz   RobotStateMachine → CAN motor commands
 *   RadioThread    +10  100 Hz   SBUS → g_input[] / g_armed
 *   HeartbeatThread -5  5 Hz     LED blink
 *   DebugThread    -10  10 Hz    USB $TEL/$EKFL stream (BPRL_DEBUG only)
 *   USBCmdThread   -20  event    USB command parser (LOG, CAL, CAN, MOTOR, POWER, RC, TGT)
 *   LogThread      -15  50 Hz    Binary SD card logging
 */

#include "src/threads.hpp"
#include "src/RobotState.hpp"
#include "src/coms/SPI.hpp"
#include "src/coms/CAN.hpp"
#include "src/coms/CANMotor.hpp"
#include "src/coms/CANPower.hpp"
#include "src/coms/Radio.hpp"
#include "src/coms/SBUS.hpp"
#include "src/controllers/RobotStateMachine.hpp"
#include "src/controllers/ActuatorSafety.hpp"
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

        // Wheel motors (CAN ids 5, 6 — Wheel L, Wheel R). can_motor_get_state()
        // is internally mutex-protected (motor_state_mtx), safe to call here.
        CanMotorState wheel_snap[2] = {};
        can_motor_get_state(5, &wheel_snap[0]);
        can_motor_get_state(6, &wheel_snap[1]);

        state_mgr.update(dt, imu_snap, can_snap, mocap_snap, wheel_snap);
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

        next = chThdSleepUntilWindowed(next, chTimeAddX(next, period));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * ControlThread — 400 Hz  NORMALPRIO+20
 * RobotStateMachine → CAN motor torque commands.
 * ══════════════════════════════════════════════════════════════════════════ */
static THD_FUNCTION(ControlThread, arg)
{
    chRegSetThreadName("ctrl");
    const sysinterval_t period = *static_cast<const sysinterval_t *>(arg);

    systime_t next = chVTGetSystemTime();
    while (true) {
        float state_snap[StateIdx::N];
        float input_snap[InputIdx::N_INPUTS];
        bool  armed_snap;
        chMtxLock(&state_mtx);
        memcpy(state_snap, g_state, sizeof(state_snap));
        memcpy(input_snap, g_input, sizeof(input_snap));
        armed_snap = g_armed;
        chMtxUnlock(&state_mtx);

        float torques[6] = {};
        robot_sm.update(state_snap, input_snap, armed_snap, torques);

        // Final safety gate — always applied, regardless of mode/controller
        // (see ActuatorSafety.hpp): hip soft angle limits, velocity soft
        // limits, hard torque clamps, fail-safe zero on missing feedback.
        actuator_safety.apply(torques);

        // Send torque commands to all 6 motors
        for (int i = 0; i < 6; i++)
            can_motor_set_torque((uint8_t)(i + 1), torques[i]);

        chMtxLock(&state_mtx);
        memcpy(g_motor_torques, torques, sizeof(g_motor_torques));
        chMtxUnlock(&state_mtx);

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

    systime_t next = chVTGetSystemTime();
    while (true) {
        radio_input_update();

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
 *           POWER,status, RC,status, TGT,status
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
                 "last_sid=0x%03x,last_dlc=%u\r\n",
                 (uint32_t)d.total_rx, (uint32_t)d.dispatched,
                 (unsigned)d.last_sid, (unsigned)d.last_dlc);
        can_get_diag(CAN_BUS_2, d);
        chprintf((BaseSequentialStream *)&SDU1,
                 "CAN,DIAG,bus2,total_rx=%lu,dispatched=%lu,"
                 "last_sid=0x%03x,last_dlc=%u\r\n",
                 (uint32_t)d.total_rx, (uint32_t)d.dispatched,
                 (unsigned)d.last_sid, (unsigned)d.last_dlc);
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
        // Print current motor state for all registered motors (IDs 1–6)
        chMtxLock(&s_usb_write_mtx);
        for (uint8_t id = 1; id <= 6; id++) {
            CanMotorState ms = {};
            if (can_motor_get_state(id, &ms)) {
                chprintf((BaseSequentialStream *)&SDU1,
                         "MOTOR,%u,pos=%.3f,vel=%.3f,tq=%.3f,temp=%.1f,v=%d\r\n",
                         (unsigned)id, (double)ms.pos_rad, (double)ms.vel_rads,
                         (double)ms.torque_Nm, (double)ms.temp_C, (int)ms.valid);
            }
        }
        chMtxUnlock(&s_usb_write_mtx);
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
        const char *mode_name = (mode == ROBOT_IDLE)      ? "IDLE"
                               : (mode == ROBOT_BALANCING) ? "BALANCING"
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
        if (byte == MSG_TIMEOUT || byte == MSG_RESET) continue;
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
 * Streams $TEL and $EKFL telemetry lines over USB CDC.
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
            static char tel_buf[256];
            MemoryStream ms;
            msObjectInit(&ms, (uint8_t *)tel_buf, sizeof(tel_buf) - 1, 0);
            chprintf((BaseSequentialStream *)&ms,
                "$TEL,%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%d,"
                "%d,%d,%d,%d,%lu,%lu\r\n",
                (uint32_t)TIME_I2MS(chVTGetSystemTime()),
                (double)(roll*57.2958f), (double)(pitch*57.2958f), (double)(yaw*57.2958f),
                (double)p, (double)q, (double)r,
                (double)vel_tgt, (double)height_set, (double)leanover, (double)yaw_stick,
                (int)armed,
                (int)imu_v[0], (int)imu_v[1], (int)imu_v[2], (int)can_v,
                can_quat_hz, can_rate_hz);
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
                "%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\r\n",
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
                (double)can_p_snap, (double)can_q_snap, (double)can_r_snap);
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
    chThdCreateStatic(waRadio,     sizeof(waRadio),     NORMALPRIO + 10, RadioThread,     (void *)&rates.radio);
    chThdCreateStatic(waHeartbeat, sizeof(waHeartbeat), NORMALPRIO -  5, HeartbeatThread, (void *)&rates.heartbeat);
#ifdef BPRL_DEBUG
    chThdCreateStatic(waDebug,     sizeof(waDebug),     NORMALPRIO - 10, DebugThread,     (void *)&rates.debug);
#endif
    chThdCreateStatic(waUSBCmd,    sizeof(waUSBCmd),    NORMALPRIO - 20, USBCmdThread,    nullptr);
    chThdCreateStatic(waLog,       sizeof(waLog),       NORMALPRIO - 15, LogThread,       (void *)&rates.log);
}
