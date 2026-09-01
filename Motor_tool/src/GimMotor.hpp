#pragma once
#include "src/CAN.hpp"
#include <cstdint>

/*
 * SteadyWin GIM series motor driver — for the GIM6010-6 wheel motors.
 * CONFIRMED against the vendor's own "STEADYWIN MOTOR DRIVER PROTOCOL
 * SPECIFICATION rev2.2" (user-provided) — this replaces an earlier
 * best-effort "MIT motor mode" guess that had no real basis; the actual
 * protocol is unrelated to that guess (float commands + a duration field,
 * not a packed pos/vel/kp/kd/torque frame).
 *
 * Frame (CAN): no header, 8-byte payload, standard frame, LSB byte order,
 * baud <1Mbps (drive-configurable — verify against this drive's actual
 * setting; default per the drive's own docs, not restated here).
 * Arbitration ID: commands are sent to the motor's own configured "CAN ID"
 * (ConfType 0x00 ConfID 0x12). REPLIES DO NOT necessarily land on that same
 * ID — confirmed 2026-08-31 via poll_wheels (motor_tool.py) against real
 * hardware: a GIM6010-6 configured with CAN ID 20 / Host-Master CAN ID
 * (ConfID 0x13) 21 replied on SID 21, with total silence on SID 20, where
 * this driver had been listening the whole time. This driver's old
 * same-ID-as-command assumption is only correct when Master CAN ID happens
 * to equal CAN ID — use gim_set_reply_id() whenever they're configured
 * differently, which needs to be checked per motor, not assumed.
 * Commands are still SENT to the motor's own CAN ID either way — only the
 * reply-listening address is affected.
 *
 * Command bytes used here (see the spec for the full list — configuration,
 * parameter, calibration, and firmware-update commands exist but aren't
 * wired up in this test tool):
 *   0x91 Start Motor        0x92 Stop Motor         0x97 Stop Control (pause)
 *   0x93 Torque Control     0x94 Speed Control      0x95 Position Control
 *   0xB2 Get Fault          0xB3 Acknowledge Fault  0xB4 Retrieve Indicator
 *
 * Torque/Speed/Position commands share one response layout: byte1=RES,
 * byte2=Temp, byte3-4=packed position (16-bit), byte5-7=packed speed+torque
 * (ST0-2, 12 bits each) — see GimMotor.cpp for the exact decode formulas
 * from the spec (section 3.2.7).
 */

struct GimState {
    float    pos_rad;         // decoded from the packed 16-bit position field
    float    vel_rads;        // decoded from the packed 12-bit speed field
    float    torque_Nm;       // decoded from the packed 12-bit torque field —
                               // needs this motor's torque constant (Nm/A) and
                               // gear ratio (see gim_set_torque_constant() /
                               // gim_set_gear_ratio()); defaults are placeholders,
                               // not read from the drive automatically.
    float    temp_C;
    uint8_t  fault;           // raw FaultNo bitmask from 0xB2, 0 = no fault
    float    last_indicator;      // most recent 0xB4 response value
    uint8_t  last_indicator_id;   // which IndID that value is for
    bool     enabled;         // true after Start Motor (0x91) ack'd, cleared by Stop
    bool     valid;           // true once at least one response decoded
    uint32_t last_update_ms;
};

#define GIM_ID_MAX 32

void gim_init(void);           // subscribes to CAN RX; call once after can_drv_init()
void gim_set_bus(CanBus bus);
CanBus gim_get_bus(void);

// Tell the driver which arbitration ID this motor's REPLIES actually
// arrive on, if different from its own CAN ID — see the header comment
// above. reply_id=0 (the default for every id) means "same as the motor's
// own CAN ID", preserving old behavior until this is set. Commands are
// still sent to `id` regardless of this setting.
void    gim_set_reply_id(uint8_t id, uint8_t reply_id);
uint8_t gim_get_reply_id(uint8_t id);   // returns id itself if never overridden

// Needed to decode the packed torque field into Nm (spec 3.2.7). Defaults
// are placeholders — read the real values with a Retrieve Configuration
// request (ConfID 0x03 float = Torque Constant, ConfID 0x11 int = Gear
// Ratio) if you need this to be accurate; not automated here.
void  gim_set_torque_constant(float Kt_Nm_per_A);
float gim_get_torque_constant(void);
void  gim_set_gear_ratio(float ratio);
float gim_get_gear_ratio(void);

// Clamps |torque_Nm| on every gim_torque() call — real Nm now, unlike the
// old MIT-mode driver's dimensionless placeholder.
void  gim_set_torque_limit(float limit_Nm);
float gim_get_torque_limit(void);

bool gim_start(uint8_t id);     // 0x91 — enter running state
bool gim_stop(uint8_t id);      // 0x92 — exit running state
bool gim_pause(uint8_t id);     // 0x97 — stop ongoing control command, stay running

// duration_ms: per the spec, how long the drive executes this command
// before it's no longer "ongoing" — reissue faster than duration_ms elapses
// to hold a command continuously. Requires gim_start() first (this driver's
// own safety gate, not documented behavior of the drive itself).
bool gim_torque(uint8_t id, float torque_Nm, uint32_t duration_ms = 1000);    // 0x93
bool gim_velocity(uint8_t id, float vel_rads, uint32_t duration_ms = 1000);   // 0x94 (converted to RPM)
bool gim_position(uint8_t id, float pos_rad, uint32_t duration_ms = 1000);    // 0x95

bool gim_get_fault(uint8_t id);                      // 0xB2 — request fault status
bool gim_ack_fault(uint8_t id);                      // 0xB3 — clear latched fault
bool gim_get_indicator(uint8_t id, uint8_t ind_id);  // 0xB4 — request one runtime indicator

bool gim_get_state(uint8_t id, GimState &out);

// Send Stop Motor (0x92) to every GIM id that has ever reported valid
// feedback or been started.
void gim_stop_all(void);
