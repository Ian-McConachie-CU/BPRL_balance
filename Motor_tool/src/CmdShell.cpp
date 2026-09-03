/*
 * CmdShell.cpp — USB text command line for Motor_tool.
 *
 * Line-based protocol over USB CDC (one command per line, \n or \r\n
 * terminated), mirroring the convention already used by BPRL_balance's
 * USBCmdThread. See tools/motor_tool.py for the ground-side client and
 * README.md for the full command reference.
 */

#include "ch.h"
#include "hal.h"
#include "src/CmdShell.hpp"
#include "src/CAN.hpp"
#include "src/RmdMotor.hpp"
#include "src/GimMotor.hpp"
#include "src/OdriveMotor.hpp"
#include "src/Imx5.hpp"
#include "src/usb_serial.hpp"
#include "chprintf.h"
#include <cstring>
#include <cstdio>
#include <cstdarg>

static MUTEX_DECL(s_usb_write_mtx);
#define STREAM ((BaseSequentialStream *)&SDU1)

static void reply(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    chMtxLock(&s_usb_write_mtx);
    chvprintf(STREAM, fmt, ap);
    chMtxUnlock(&s_usb_write_mtx);
    va_end(ap);
}

/* ── Bus selection (shared by RMD/GIM commands and the scanner) ─────────── */
static CanBus s_active_bus = CAN_BUS_1;

/* ── Watchdog: if the host stops talking, kill all motor output ─────────── */
static volatile uint32_t s_last_host_ms = 0;

static void touch_watchdog(void) { s_last_host_ms = (uint32_t)TIME_I2MS(chVTGetSystemTime()); }

static THD_WORKING_AREA(waWatchdog, 1024);
static THD_FUNCTION(WatchdogThread, arg)
{
    (void)arg;
    chRegSetThreadName("mtr_wd");
    static constexpr uint32_t TIMEOUT_MS       = 500;
    static constexpr uint32_t RESEND_PERIOD_MS = 100;
    uint32_t last_stop_ms = 0;
    while (true) {
        uint32_t now = (uint32_t)TIME_I2MS(chVTGetSystemTime());
        if (now - s_last_host_ms > TIMEOUT_MS && now - last_stop_ms > RESEND_PERIOD_MS) {
            rmd_stop_all();
            gim_stop_all();
            odrive_stop_all();
            last_stop_ms = now;
        }
        chThdSleepMilliseconds(20);
    }
}

/* ── HELP text ────────────────────────────────────────────────────────────*/
static const char *const kHelp =
    "Motor_tool commands:\r\n"
    "  HELP                                  this text\r\n"
    "  PING                                  -> PONG\r\n"
    "  BOOT                                  reset into bootloader\r\n"
    "  BUS,<1|2>                             select CAN bus for RMD/GIM/scan cmds\r\n"
    "  STATUS                                dump state of all seen motors\r\n"
    "  STOP,ALL                              zero/disable every motor immediately\r\n"
    "  CAN,status                            FDCAN1+2 register snapshot\r\n"
    "  CAN,diag                              per-bus RX counters\r\n"
    "  CAN,scan,start / CAN,scan,stop        ID scanner on the active bus\r\n"
    "  CAN,monitor,start / CAN,monitor,stop  live raw frame dump (both buses)\r\n"
    "  CAN,selftest                          internal-loopback test of the active bus (no wiring needed)\r\n"
    "  CAN,send,<id_hex>,<ext0|1>,b0..b7     inject a raw 8-byte frame\r\n"
    "  RMD,<id>,TORQUE,<Nm>                  MG8016E-i6: torque command (Nm, unverified scale)\r\n"
    "  RMD,<id>,TORQUERAW,<ratio>            MG8016E-i6: torque command (raw -2000..2000, confirmed)\r\n"
    "  RMD,SCALE,<ratio_per_Nm>               get/set the TORQUE Nm->ratio scale\r\n"
    "  RMD,<id>,VELOCITY,<rad/s>             MG8016E-i6: velocity-loop command\r\n"
    "  RMD,<id>,POSITION,<rad>,<maxspd>      MG8016E-i6: position command, ABSOLUTE multi-turn target (0xA4)\r\n"
    "  RMD,<id>,SINGLETURN,<rad>,<maxspd>[,<cw0/1>] MG8016E-i6: position command, ABSOLUTE single-turn 0..360deg (0xA6, default cw=1)\r\n"
    "  RMD,<id>,INCREMENT,<rad>              MG8016E-i6: position command, RELATIVE to current position (0xA7)\r\n"
    "  RMD,<id>,STOP|OFF|RESUME              MG8016E-i6: zero out / disable / re-enable\r\n"
    "  RMD,<id>,STATUS                       MG8016E-i6: request status1 (temp/errors)\r\n"
    "  RMD,<id>,CLEARERR                     MG8016E-i6: clear latched error flags\r\n"
    "  RMD,<id>,ENCODER                      MG8016E-i6: request current encoder position (confirmed, motor-side, 16-bit)\r\n"
    "  GIM,<id>,START|STOP|PAUSE              GIM6010-6: enter/exit running state / pause current command\r\n"
    "  GIM,<id>,TORQUE,<Nm>[,<duration_ms>]   GIM6010-6: torque command (real Nm, confirmed)\r\n"
    "  GIM,<id>,VELOCITY,<rad/s>[,<duration_ms>] GIM6010-6: velocity command\r\n"
    "  GIM,<id>,POSITION,<rad>[,<duration_ms>]   GIM6010-6: position command\r\n"
    "  GIM,<id>,FAULT | ACKFAULT              GIM6010-6: request/clear fault status\r\n"
    "  GIM,<id>,IND,<ind_id>                  GIM6010-6: request one runtime indicator (0=bus V, 2=motor temp, ...)\r\n"
    "  GIM,<id>,MASTERID,<reply_id>           GIM6010-6: set the SID this id's replies actually arrive on (Host/Master CAN ID)\r\n"
    "  GIM,LIMIT,<Nm>                        set/query the GIM torque clamp\r\n"
    "  GIM,KT,<Nm_per_A> / GIM,GEAR,<ratio>  set the constants used to decode GIM torque feedback\r\n"
    "  ODRIVE,LIST                           list node_ids seen via Heartbeat -- the GDS68 scanner (no probing needed)\r\n"
    "  ODRIVE,<id>,START                     GDS68/ODrive: torque-control mode + closed-loop (0x0B+0x07)\r\n"
    "  ODRIVE,<id>,IDLE|STOP                 GDS68/ODrive: Set_Axis_State(IDLE) (0x07)\r\n"
    "  ODRIVE,<id>,MODE,TORQUE|VELOCITY      GDS68/ODrive: Set_Controller_Mode (0x0B)\r\n"
    "  ODRIVE,<id>,TORQUE,<Nm>                GDS68/ODrive: Set_Input_Torque, output-referenced (0x0E)\r\n"
    "  ODRIVE,<id>,VELOCITY,<rad/s>           GDS68/ODrive: Set_Input_Vel, output-referenced (0x0D)\r\n"
    "  ODRIVE,GEAR,<ratio>                    get/set the GIM6010-8 gear ratio used for scaling/decode\r\n"
    "  ODRIVE,LIMIT,<Nm>                      get/set the ODrive torque clamp\r\n"
    "  IMU,status                            IMX5 INS decode (bus 2, known-good reference device)\r\n";

/* ── CAN,send raw injection ──────────────────────────────────────────────*/
static void handle_can_send(const char *rest)
{
    unsigned id, ext;
    unsigned b[8];
    int n = sscanf(rest, "%x,%u,%x,%x,%x,%x,%x,%x,%x,%x",
                   &id, &ext, &b[0], &b[1], &b[2], &b[3], &b[4], &b[5], &b[6], &b[7]);
    if (n != 10) { reply("CAN,SEND,ERR,bad_args\r\n"); return; }
    uint8_t data[8];
    for (int i = 0; i < 8; i++) data[i] = (uint8_t)b[i];
    // Standard-frame only (ext flag accepted for future use, not yet sent as EFF).
    (void)ext;
    bool ok = can_send(s_active_bus, id, data, 8);
    reply("CAN,SEND,%s\r\n", ok ? "OK" : "ERR");
}

static void handle_can(const char *rest)
{
    if (strcmp(rest, "status") == 0) {
        chMtxLock(&s_usb_write_mtx);
        chprintf(STREAM, "CAN,STATUS,bus=1,psr=0x%08x,ecr=0x%08x,rxf0s=0x%08x,cccr=0x%08x\r\n",
                 (unsigned)FDCAN1->PSR, (unsigned)FDCAN1->ECR,
                 (unsigned)FDCAN1->RXF0S, (unsigned)FDCAN1->CCCR);
        chprintf(STREAM, "CAN,STATUS,bus=2,psr=0x%08x,ecr=0x%08x,rxf0s=0x%08x,cccr=0x%08x\r\n",
                 (unsigned)FDCAN2->PSR, (unsigned)FDCAN2->ECR,
                 (unsigned)FDCAN2->RXF0S, (unsigned)FDCAN2->CCCR);
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(rest, "diag") == 0) {
        CANDiag d = {};
        can_get_diag(CAN_BUS_1, d);
        chMtxLock(&s_usb_write_mtx);
        chprintf(STREAM, "CAN,DIAG,bus=1,total_rx=%lu,last_sid=0x%03x,last_dlc=%u\r\n",
                 (uint32_t)d.total_rx, (unsigned)d.last_sid, (unsigned)d.last_dlc);
        chMtxUnlock(&s_usb_write_mtx);
        can_get_diag(CAN_BUS_2, d);
        chMtxLock(&s_usb_write_mtx);
        chprintf(STREAM, "CAN,DIAG,bus=2,total_rx=%lu,last_sid=0x%03x,last_dlc=%u\r\n",
                 (uint32_t)d.total_rx, (unsigned)d.last_sid, (unsigned)d.last_dlc);
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(rest, "scan,start") == 0) {
        can_scan_start(s_active_bus);
        reply("CAN,SCAN,started,bus=%d\r\n", (int)s_active_bus + 1);
    } else if (strcmp(rest, "scan,stop") == 0) {
        can_scan_stop();
        CANScanEntry entries[CAN_SCAN_MAX];
        int n = can_scan_get(entries, CAN_SCAN_MAX);
        chMtxLock(&s_usb_write_mtx);
        for (int i = 0; i < n; i++) {
            chprintf(STREAM, "CAN,SCAN,id=%s0x%03lx,count=%lu\r\n",
                     entries[i].is_ext ? "EXT:" : "",
                     (uint32_t)entries[i].id, (uint32_t)entries[i].count);
        }
        chprintf(STREAM, "CAN,SCAN,END\r\n");
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(rest, "monitor,start") == 0) {
        can_monitor_start();
        reply("CAN,MONITOR,started\r\n");
    } else if (strcmp(rest, "monitor,stop") == 0) {
        can_monitor_stop();
        reply("CAN,MONITOR,stopped\r\n");
    } else if (strncmp(rest, "send,", 5) == 0) {
        handle_can_send(rest + 5);
    } else if (strcmp(rest, "selftest") == 0) {
        CanSelftestResult d = {};
        bool ok = can_selftest(s_active_bus, &d);
        reply("CAN,SELFTEST,%s,bus=%d,tx_accepted=%d,rx_matched=%d,rx_total=%lu->%lu\r\n",
              ok ? "PASS" : "FAIL", (int)s_active_bus + 1, (int)d.tx_accepted,
              (int)d.rx_matched, (uint32_t)d.rx_total_before, (uint32_t)d.rx_total_after);
    } else {
        reply("CAN,ERR,unknown_cmd\r\n");
    }
}

/* ── RMD (MG8016E-i6) ─────────────────────────────────────────────────────*/
static void handle_rmd(const char *rest)
{
    if (strncmp(rest, "SCALE,", 6) == 0) {
        float v;
        if (sscanf(rest + 6, "%f", &v) == 1) {
            rmd_set_torque_scale(v);
            reply("RMD,SCALE,OK,%.3f\r\n", (double)rmd_get_torque_scale());
        } else {
            reply("RMD,SCALE,%.3f\r\n", (double)rmd_get_torque_scale());
        }
        return;
    }

    unsigned id;
    int consumed = 0;
    if (sscanf(rest, "%u%n", &id, &consumed) != 1 || id < 1 || id > RMD_ID_MAX) {
        reply("RMD,ERR,bad_id\r\n");
        return;
    }
    rmd_set_bus(s_active_bus);
    const char *p = rest + consumed;
    if (*p == ',') p++;

    if (strncmp(p, "TORQUERAW,", 10) == 0) {
        int v;
        bool ok = sscanf(p + 10, "%d", &v) == 1 && rmd_torque_raw((uint8_t)id, (int16_t)v);
        reply("RMD,%u,TORQUERAW,%s\r\n", id, ok ? "OK" : "ERR");
    } else if (strncmp(p, "TORQUE,", 7) == 0) {
        float v;
        bool ok = sscanf(p + 7, "%f", &v) == 1 && rmd_torque((uint8_t)id, v);
        reply("RMD,%u,TORQUE,%s\r\n", id, ok ? "OK" : "ERR");
    } else if (strncmp(p, "VELOCITY,", 9) == 0) {
        float v;
        bool ok = sscanf(p + 9, "%f", &v) == 1 && rmd_velocity((uint8_t)id, v);
        reply("RMD,%u,VELOCITY,%s\r\n", id, ok ? "OK" : "ERR");
    } else if (strncmp(p, "POSITION,", 9) == 0) {
        float a, s;
        bool ok = sscanf(p + 9, "%f,%f", &a, &s) == 2 && rmd_position((uint8_t)id, a, s);
        reply("RMD,%u,POSITION,%s\r\n", id, ok ? "OK" : "ERR");
    } else if (strncmp(p, "INCREMENT,", 10) == 0) {
        float d;
        bool ok = sscanf(p + 10, "%f", &d) == 1 && rmd_increment_position((uint8_t)id, d);
        reply("RMD,%u,INCREMENT,%s\r\n", id, ok ? "OK" : "ERR");
    } else if (strncmp(p, "SINGLETURN,", 11) == 0) {
        float a, s; int dir = 1;   // dir defaults to CW (1) if the 3rd field is omitted
        int n = sscanf(p + 11, "%f,%f,%d", &a, &s, &dir);
        bool ok = n >= 2 && rmd_position_single_turn((uint8_t)id, a, s, dir != 0);
        reply("RMD,%u,SINGLETURN,%s\r\n", id, ok ? "OK" : "ERR");
    } else if (strcmp(p, "STOP") == 0) {
        reply("RMD,%u,STOP,%s\r\n", id, rmd_stop((uint8_t)id) ? "OK" : "ERR");
    } else if (strcmp(p, "OFF") == 0) {
        reply("RMD,%u,OFF,%s\r\n", id, rmd_off((uint8_t)id) ? "OK" : "ERR");
    } else if (strcmp(p, "RESUME") == 0) {
        reply("RMD,%u,RESUME,%s\r\n", id, rmd_resume((uint8_t)id) ? "OK" : "ERR");
    } else if (strcmp(p, "STATUS") == 0) {
        reply("RMD,%u,STATUS,%s\r\n", id, rmd_request_status((uint8_t)id) ? "OK" : "ERR");
    } else if (strcmp(p, "CLEARERR") == 0) {
        reply("RMD,%u,CLEARERR,%s\r\n", id, rmd_clear_error((uint8_t)id) ? "OK" : "ERR");
    } else if (strcmp(p, "ENCODER") == 0) {
        reply("RMD,%u,ENCODER,%s\r\n", id, rmd_read_encoder((uint8_t)id) ? "OK" : "ERR");
    } else {
        reply("RMD,%u,ERR,unknown_cmd\r\n", id);
    }
}

/* ── GIM (SteadyWin GIM6010-6) ─────────────────────────────────────────────*/
static void handle_gim(const char *rest)
{
    if (strncmp(rest, "LIMIT,", 6) == 0) {
        float v;
        if (sscanf(rest + 6, "%f", &v) == 1) {
            gim_set_torque_limit(v);
            reply("GIM,LIMIT,OK,%.3f\r\n", (double)gim_get_torque_limit());
        } else {
            reply("GIM,LIMIT,%.3f\r\n", (double)gim_get_torque_limit());
        }
        return;
    }
    if (strncmp(rest, "KT,", 3) == 0) {
        float v;
        if (sscanf(rest + 3, "%f", &v) == 1) gim_set_torque_constant(v);
        reply("GIM,KT,%.4f\r\n", (double)gim_get_torque_constant());
        return;
    }
    if (strncmp(rest, "GEAR,", 5) == 0) {
        float v;
        if (sscanf(rest + 5, "%f", &v) == 1) gim_set_gear_ratio(v);
        reply("GIM,GEAR,%.4f\r\n", (double)gim_get_gear_ratio());
        return;
    }

    unsigned id;
    int consumed = 0;
    if (sscanf(rest, "%u%n", &id, &consumed) != 1 || id < 1 || id > GIM_ID_MAX) {
        reply("GIM,ERR,bad_id\r\n");
        return;
    }
    gim_set_bus(s_active_bus);
    const char *p = rest + consumed;
    if (*p == ',') p++;

    if (strcmp(p, "START") == 0) {
        reply("GIM,%u,START,%s\r\n", id, gim_start((uint8_t)id) ? "OK" : "ERR");
    } else if (strcmp(p, "STOP") == 0) {
        reply("GIM,%u,STOP,%s\r\n", id, gim_stop((uint8_t)id) ? "OK" : "ERR");
    } else if (strcmp(p, "PAUSE") == 0) {
        reply("GIM,%u,PAUSE,%s\r\n", id, gim_pause((uint8_t)id) ? "OK" : "ERR");
    } else if (strncmp(p, "TORQUE,", 7) == 0) {
        float v; unsigned dur = 1000;
        int n = sscanf(p + 7, "%f,%u", &v, &dur);
        bool ok = n >= 1 && gim_torque((uint8_t)id, v, dur);
        reply("GIM,%u,TORQUE,%s\r\n", id, ok ? "OK" : "ERR,not_started_or_bad_args");
    } else if (strncmp(p, "VELOCITY,", 9) == 0) {
        float v; unsigned dur = 1000;
        int n = sscanf(p + 9, "%f,%u", &v, &dur);
        bool ok = n >= 1 && gim_velocity((uint8_t)id, v, dur);
        reply("GIM,%u,VELOCITY,%s\r\n", id, ok ? "OK" : "ERR,not_started_or_bad_args");
    } else if (strncmp(p, "POSITION,", 9) == 0) {
        float v; unsigned dur = 1000;
        int n = sscanf(p + 9, "%f,%u", &v, &dur);
        bool ok = n >= 1 && gim_position((uint8_t)id, v, dur);
        reply("GIM,%u,POSITION,%s\r\n", id, ok ? "OK" : "ERR,not_started_or_bad_args");
    } else if (strcmp(p, "FAULT") == 0) {
        reply("GIM,%u,FAULT,%s\r\n", id, gim_get_fault((uint8_t)id) ? "OK" : "ERR");
    } else if (strcmp(p, "ACKFAULT") == 0) {
        reply("GIM,%u,ACKFAULT,%s\r\n", id, gim_ack_fault((uint8_t)id) ? "OK" : "ERR");
    } else if (strncmp(p, "IND,", 4) == 0) {
        unsigned ind_id;
        bool ok = sscanf(p + 4, "%u", &ind_id) == 1
                  && gim_get_indicator((uint8_t)id, (uint8_t)ind_id);
        reply("GIM,%u,IND,%s\r\n", id, ok ? "OK" : "ERR");
    } else if (strncmp(p, "MASTERID,", 9) == 0) {
        unsigned reply_id;
        if (sscanf(p + 9, "%u", &reply_id) == 1) gim_set_reply_id((uint8_t)id, (uint8_t)reply_id);
        reply("GIM,%u,MASTERID,%u\r\n", id, (unsigned)gim_get_reply_id((uint8_t)id));
    } else {
        reply("GIM,%u,ERR,unknown_cmd\r\n", id);
    }
}

/* ── ODRIVE (GDS68 / Steadywin GIM6010-8) ─────────────────────────────────*/
static void handle_odrive(const char *rest)
{
    if (strcmp(rest, "LIST") == 0) {
        uint8_t ids[ODRIVE_ID_MAX + 1];
        int n = odrive_list_seen(ids, ODRIVE_ID_MAX + 1);
        uint32_t now = (uint32_t)TIME_I2MS(chVTGetSystemTime());
        chMtxLock(&s_usb_write_mtx);
        for (int i = 0; i < n; i++) {
            OdriveState st;
            odrive_get_state(ids[i], st);
            chprintf(STREAM, "ODRIVE,LIST,id=%u,axis_state=%u,axis_error=0x%08lx,age_ms=%lu\r\n",
                     (unsigned)ids[i], (unsigned)st.axis_state, (uint32_t)st.axis_error,
                     (uint32_t)(now - st.last_heartbeat_ms));
        }
        chprintf(STREAM, "ODRIVE,LIST,END\r\n");
        chMtxUnlock(&s_usb_write_mtx);
        return;
    }
    if (strncmp(rest, "GEAR,", 5) == 0) {
        float v;
        if (sscanf(rest + 5, "%f", &v) == 1) odrive_set_gear_ratio(v);
        reply("ODRIVE,GEAR,%.4f\r\n", (double)odrive_get_gear_ratio());
        return;
    }
    if (strncmp(rest, "LIMIT,", 6) == 0) {
        float v;
        if (sscanf(rest + 6, "%f", &v) == 1) odrive_set_torque_limit(v);
        reply("ODRIVE,LIMIT,%.3f\r\n", (double)odrive_get_torque_limit());
        return;
    }

    unsigned id;
    int consumed = 0;
    if (sscanf(rest, "%u%n", &id, &consumed) != 1 || id > ODRIVE_ID_MAX) {
        reply("ODRIVE,ERR,bad_id\r\n");
        return;
    }
    odrive_set_bus(s_active_bus);
    const char *p = rest + consumed;
    if (*p == ',') p++;

    if (strcmp(p, "START") == 0) {
        reply("ODRIVE,%u,START,%s\r\n", id, odrive_start((uint8_t)id) ? "OK" : "ERR");
    } else if (strcmp(p, "IDLE") == 0 || strcmp(p, "STOP") == 0) {
        reply("ODRIVE,%u,IDLE,%s\r\n", id, odrive_idle((uint8_t)id) ? "OK" : "ERR");
    } else if (strncmp(p, "MODE,", 5) == 0) {
        const char *m = p + 5;
        bool known = (strcmp(m, "TORQUE") == 0) || (strcmp(m, "VELOCITY") == 0);
        bool ok = known && odrive_set_mode((uint8_t)id, strcmp(m, "VELOCITY") == 0);
        reply("ODRIVE,%u,MODE,%s\r\n", id, ok ? "OK" : "ERR");
    } else if (strncmp(p, "TORQUE,", 7) == 0) {
        float v;
        bool ok = sscanf(p + 7, "%f", &v) == 1 && odrive_torque((uint8_t)id, v);
        reply("ODRIVE,%u,TORQUE,%s\r\n", id, ok ? "OK" : "ERR");
    } else if (strncmp(p, "VELOCITY,", 9) == 0) {
        float v;
        bool ok = sscanf(p + 9, "%f", &v) == 1 && odrive_velocity((uint8_t)id, v);
        reply("ODRIVE,%u,VELOCITY,%s\r\n", id, ok ? "OK" : "ERR");
    } else {
        reply("ODRIVE,%u,ERR,unknown_cmd\r\n", id);
    }
}

/* ── IMU (IMX5, bus 2 — known-good reference device) ─────────────────────*/
static void print_imu_line(void)
{
    Imx5State s;
    bool ok = imx5_get_state(s);
    uint32_t now = (uint32_t)TIME_I2MS(chVTGetSystemTime());
    if (!ok) {
        chprintf(STREAM, "IMU,STATUS,valid=0\r\n");
        return;
    }
    float roll, pitch, yaw;
    imx5_quat_to_euler(s.q0, s.q1, s.q2, s.q3, roll, pitch, yaw);
    chprintf(STREAM,
        "IMU,STATUS,valid=1,rpy_deg=%.2f,%.2f,%.2f,pqr=%.4f,%.4f,%.4f,"
        "accel=%.3f,%.3f,%.3f,quat_age_ms=%lu,rate_age_ms=%lu\r\n",
        (double)(roll * 57.2958f), (double)(pitch * 57.2958f), (double)(yaw * 57.2958f),
        (double)s.p, (double)s.q, (double)s.r,
        (double)s.ax, (double)s.ay, (double)s.az,
        (uint32_t)(now - s.last_quat_ms), (uint32_t)(now - s.last_rate_ms));
}

static void handle_imu(const char *rest)
{
    if (strcmp(rest, "status") == 0) {
        chMtxLock(&s_usb_write_mtx);
        print_imu_line();
        chMtxUnlock(&s_usb_write_mtx);
    } else {
        reply("IMU,ERR,unknown_cmd\r\n");
    }
}

/* ── STATUS / STOP,ALL ────────────────────────────────────────────────────*/
static void handle_status(void)
{
    uint32_t now = (uint32_t)TIME_I2MS(chVTGetSystemTime());
    chMtxLock(&s_usb_write_mtx);
    chprintf(STREAM, "STATUS,BUS,%d\r\n", (int)s_active_bus + 1);
    print_imu_line();
    for (uint8_t id = 1; id <= RMD_ID_MAX; id++) {
        RmdState s;
        if (rmd_get_state(id, s) && s.valid) {
            chprintf(STREAM,
                "STATUS,RMD,%u,pos=%.3f,vel=%.3f,tq=%.3f,temp=%.1f,volt=%.1f,err=0x%02x,age_ms=%lu\r\n",
                (unsigned)id, (double)s.pos_rad, (double)s.vel_rads, (double)s.torque_Nm,
                (double)s.temp_C, (double)s.voltage_V, (unsigned)s.error_flags,
                (uint32_t)(now - s.last_update_ms));
        }
    }
    for (uint8_t id = 1; id <= GIM_ID_MAX; id++) {
        GimState s;
        if (gim_get_state(id, s) && (s.valid || s.enabled)) {
            chprintf(STREAM,
                "STATUS,GIM,%u,pos=%.3f,vel=%.3f,tq=%.3f,temp=%.1f,fault=0x%02x,"
                "ind%u=%.3f,enabled=%d,valid=%d,age_ms=%lu\r\n",
                (unsigned)id, (double)s.pos_rad, (double)s.vel_rads, (double)s.torque_Nm,
                (double)s.temp_C, (unsigned)s.fault, (unsigned)s.last_indicator_id,
                (double)s.last_indicator, (int)s.enabled, (int)s.valid,
                (uint32_t)(now - s.last_update_ms));
        }
    }
    for (uint8_t id = 0; id <= ODRIVE_ID_MAX; id++) {
        OdriveState s;
        if (odrive_get_state(id, s) && s.valid) {
            chprintf(STREAM,
                "STATUS,ODRIVE,%u,pos=%.3f,vel=%.3f,axis_state=%u,axis_error=0x%08lx,age_ms=%lu\r\n",
                (unsigned)id, (double)s.pos_rad, (double)s.vel_rads,
                (unsigned)s.axis_state, (uint32_t)s.axis_error,
                (uint32_t)(now - s.last_heartbeat_ms));
        }
    }
    chprintf(STREAM, "STATUS,END\r\n");
    chMtxUnlock(&s_usb_write_mtx);
}

/* ── Top-level dispatch ───────────────────────────────────────────────────*/
static void dispatch(const char *line)
{
    if (strcmp(line, "HELP") == 0) {
        chMtxLock(&s_usb_write_mtx);
        chprintf(STREAM, "%s", kHelp);
        chMtxUnlock(&s_usb_write_mtx);
    } else if (strcmp(line, "PING") == 0) {
        reply("PONG\r\n");
    } else if (strcmp(line, "BOOT") == 0) {
        reply("BOOT,OK\r\n");
        chThdSleepMilliseconds(50);
        NVIC_SystemReset();
    } else if (strcmp(line, "BUS,1") == 0) {
        s_active_bus = CAN_BUS_1;
        reply("BUS,OK,1\r\n");
    } else if (strcmp(line, "BUS,2") == 0) {
        s_active_bus = CAN_BUS_2;
        reply("BUS,OK,2\r\n");
    } else if (strcmp(line, "STATUS") == 0) {
        handle_status();
    } else if (strcmp(line, "STOP,ALL") == 0) {
        rmd_stop_all();
        gim_stop_all();
        odrive_stop_all();
        reply("STOP,ALL,OK\r\n");
    } else if (strncmp(line, "CAN,", 4) == 0) {
        handle_can(line + 4);
    } else if (strncmp(line, "RMD,", 4) == 0) {
        handle_rmd(line + 4);
    } else if (strncmp(line, "GIM,", 4) == 0) {
        handle_gim(line + 4);
    } else if (strncmp(line, "ODRIVE,", 7) == 0) {
        handle_odrive(line + 7);
    } else if (strncmp(line, "IMU,", 4) == 0) {
        handle_imu(line + 4);
    } else if (line[0] != '\0') {
        reply("ERR,unknown_cmd\r\n");
    }
}

/* ── Monitor stream drain ────────────────────────────────────────────────*/
static void drain_monitor(void)
{
    if (!can_monitor_active()) return;
    CanMonFrame f;
    int budget = 16;   // cap per loop iteration so command handling stays responsive
    while (budget-- > 0 && can_monitor_pop(f)) {
        chMtxLock(&s_usb_write_mtx);
        chprintf(STREAM, "MON,%lu,%u,0x%03lx,%u,%u",
                 (uint32_t)f.t_ms, (unsigned)f.bus + 1, (uint32_t)f.id,
                 (unsigned)f.is_ext, (unsigned)f.dlc);
        for (int i = 0; i < f.dlc && i < 8; i++) chprintf(STREAM, ",%02x", f.data[i]);
        chprintf(STREAM, "\r\n");
        chMtxUnlock(&s_usb_write_mtx);
    }
}

/* ── Command thread ───────────────────────────────────────────────────────*/
static THD_WORKING_AREA(waCmd, 4096);
static THD_FUNCTION(CmdThread, arg)
{
    (void)arg;
    chRegSetThreadName("mtr_cmd");

    static char    s_line[96];
    static uint8_t s_len = 0;

    touch_watchdog();
    while (true) {
        msg_t byte = chnGetTimeout((BaseChannel *)&SDU1, TIME_MS2I(10));
        if (byte != MSG_TIMEOUT && byte != MSG_RESET) {
            touch_watchdog();
            char c = (char)byte;
            if (c == '\n' || c == '\r') {
                if (s_len > 0) {
                    s_line[s_len] = '\0';
                    dispatch(s_line);
                    s_len = 0;
                }
            } else if (s_len < (uint8_t)(sizeof(s_line) - 1U)) {
                s_line[s_len++] = c;
            } else {
                s_len = 0;   // overlong line — drop and resync
            }
        }
        drain_monitor();
    }
}

void cmd_shell_start(void)
{
    touch_watchdog();
    chThdCreateStatic(waCmd, sizeof(waCmd), NORMALPRIO, CmdThread, nullptr);
    chThdCreateStatic(waWatchdog, sizeof(waWatchdog), NORMALPRIO - 5, WatchdogThread, nullptr);
}
