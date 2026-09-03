#pragma once
#include "src/CAN.hpp"
#include <cstdint>

/*
 * ODrive CAN Simple protocol driver — for a Steadywin GIM6010-8 wheel motor
 * driven by a GDS68 board running ODrive firmware. This is this project's
 * current wheel hardware, replacing the damaged GIM6010-6/SteadyWin-native
 * drives (see plan.md's "Wheel motor CAN failure" section) — see
 * BPRL_balance/main.cpp: wheel L = node_id 2, wheel R = node_id 3.
 *
 * Protocol confidence mirrors the parent BPRL_balance firmware's
 * src/coms/CANMotor.cpp (kept in sync deliberately, same as CAN.cpp):
 *   - Set_Axis_State (0x07), Set_Controller_Mode (0x0B), Set_Input_Vel (0x0D)
 *     — CONFIRMED against real hardware in final-project-Ian-McConachie-CU
 *     (an ECEN5813 balancing-robot project using this same GIM6010-8 +
 *     ODrive combination, different CAN transceiver).
 *   - Heartbeat (0x01), Get_Encoder_Estimates (0x09), Set_Input_Torque (0x0E)
 *     — standard ODrive CAN Simple numbering for the same firmware family,
 *     NOT yet confirmed against this project's actual GDS68 traffic.
 *
 * Arbitration ID = (node_id << 5) | cmd_id (standard 11-bit frame).
 *
 * Heartbeat is broadcast by every ODrive axis continuously and
 * unconditionally (~10 Hz default) whenever the drive is powered — no
 * request needed. That makes it this driver's built-in bus scanner: unlike
 * RMD/GIM, you don't need to probe a candidate ID and wait for a reply, you
 * just listen. odrive_init() subscribes at boot, so the "seen" table below
 * fills in continuously in the background from the moment the tool starts;
 * ODRIVE,LIST (Python: `find odrive`) just reads whatever's already been
 * overheard.
 */

struct OdriveState {
    uint32_t axis_error;       // Heartbeat bytes 0-3, standard layout (unconfirmed for this hardware)
    uint8_t  axis_state;       // Heartbeat byte 4 (1=IDLE, 8=CLOSED_LOOP, ...)
    float    pos_rad;          // from Get_Encoder_Estimates, output-shaft referenced (gear-divided)
    float    vel_rads;         // ditto
    bool     valid;            // true once at least one Heartbeat has been seen
    uint32_t last_heartbeat_ms;
};

#define ODRIVE_ID_MAX 63   // node_id is the upper 6 bits of the 11-bit arbitration ID

constexpr uint32_t ODRIVE_AXIS_STATE_IDLE        = 1;
constexpr uint32_t ODRIVE_AXIS_STATE_CLOSED_LOOP = 8;
constexpr uint32_t ODRIVE_CONTROL_MODE_TORQUE    = 1;
constexpr uint32_t ODRIVE_CONTROL_MODE_VELOCITY  = 2;
constexpr uint32_t ODRIVE_INPUT_MODE_PASSTHROUGH = 1;

void odrive_init(void);           // subscribes to CAN RX; call once after can_drv_init()
void odrive_set_bus(CanBus bus);
CanBus odrive_get_bus(void);

// GIM6010-8's internal reduction ratio — matches ODRIVE_GEAR_RATIO in the
// parent BPRL_balance firmware's src/coms/CANMotor.cpp. Used to convert
// output-shaft-referenced rad / rad-s (this tool's convention, matching
// RMD/GIM) to the motor-side units the ODrive axis itself expects/reports.
void  odrive_set_gear_ratio(float ratio);
float odrive_get_gear_ratio(void);

// Clamps |torque_Nm| on every odrive_torque() call, same pattern as
// gim_set_torque_limit()/RMD_TORQUE_RATIO_MAX.
void  odrive_set_torque_limit(float limit_Nm);
float odrive_get_torque_limit(void);

// Puts the axis into torque-passthrough control and closed-loop state —
// mirrors odrive_init_axis() in the parent firmware's CANMotor.cpp.
bool odrive_start(uint8_t id);              // Set_Controller_Mode(TORQUE) + Set_Axis_State(CLOSED_LOOP)
bool odrive_idle(uint8_t id);               // Set_Axis_State(IDLE) — 0x07
bool odrive_set_mode(uint8_t id, bool velocity_mode);   // Set_Controller_Mode — 0x0B

bool odrive_torque(uint8_t id, float torque_Nm);        // Set_Input_Torque — 0x0E, output-referenced, clamped
bool odrive_velocity(uint8_t id, float vel_rads);       // Set_Input_Vel — 0x0D, output-referenced

bool odrive_get_state(uint8_t id, OdriveState &out);

// List every node_id (0..ODRIVE_ID_MAX) that has broadcast at least one
// Heartbeat within max_age_ms — the actual "scan for a GDS68" mechanism.
// Returns the count written to out[] (capped at max).
int odrive_list_seen(uint8_t *out, int max, uint32_t max_age_ms = 2000);

// Sends Set_Axis_State(IDLE) to every id that has ever reported valid.
void odrive_stop_all(void);
