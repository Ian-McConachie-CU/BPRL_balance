#pragma once
#include "ch.h"
#include "hal.h"

/*
 * CAN motor driver — up to CAN_MOTOR_MAX motors on FDCAN1 (bus 1).
 *
 * Supported protocols:
 *   CAN_MOTOR_RMD     — LK-TECH MG8016E-i6 hip motors (DG80R/C7 drive),
 *                       cmd at 0x140+id, response at THE SAME 0x140+id
 *                       (confirmed by the vendor's "CAN PROTOCOL V2.35"
 *                       document — not 0x240+id like the MyActuator/RMD-X
 *                       convention this used to be guessed from; that
 *                       guess meant this driver never received a reply
 *                       regardless of wiring, see Motor_tool/src/RmdMotor.hpp
 *                       for the full writeup of how that was found and fixed)
 *   CAN_MOTOR_SDC102  — Steadywin GDS6/SDC102 driver (GIM6010-6 wheel motors)
 *                       stub — fill frame format from GIM6010 datasheet.
 *                       A working, hardware-confirmed version of this native
 *                       protocol exists in Motor_tool/src/GimMotor.cpp/.hpp —
 *                       port that in if you switch back to this driver.
 *
 *   CAN_MOTOR_ODRIVE  — ODrive CAN Simple protocol, for a GIM6010-8 running
 *                       ODrive firmware on a GDS68 (or any ODrive axis).
 *                       Arbitration ID = (node_id << 5) | cmd_id. Command IDs
 *                       0x07 (Set_Axis_State), 0x0B (Set_Controller_Mode),
 *                       0x0D (Set_Input_Vel) are CONFIRMED against real
 *                       hardware (see final-project-Ian-McConachie-CU, an
 *                       ECEN5813 balancing-robot project using the same
 *                       GIM6010-8 + ODrive combination). 0x09
 *                       (Get_Encoder_Estimates), 0x0E (Set_Input_Torque) and
 *                       0x01 (Heartbeat) are the standard ODrive CAN Simple
 *                       numbering for the same firmware family, but were NOT
 *                       independently exercised in that project (it only
 *                       ever sent open-loop velocity commands, never read
 *                       feedback) — verify against your ODrive firmware
 *                       version's can_simple.dbc if these don't respond.
 *                       node_id must be configured on the physical drive
 *                       (odrivetool: odrv0.axis0.config.can.node_id = ...)
 *                       to match the id passed to can_motor_register().
 *
 * RMD torque scale: CAN PROTOCOL V2.35 confirms the drive's torque command
 * is iqControl, int16_t range -2048..2048 corresponding to -33..33 A for
 * the MG series specifically — NOT a dimensionless ratio, and NOT literal
 * Nm (the motor's Kt isn't documented). can_motor_set_torque() converts Nm
 * through a placeholder scale (see CAN_MOTOR_RMD_TORQUE_SCALE in
 * CANMotor.cpp) that has not been calibrated against a real load; prefer
 * can_motor_set_torque_raw() (confirmed range/units, in amps-equivalent
 * counts) when you need to know exactly what's going on the bus. Speed
 * commands are clamped to the drive's confirmed +/-24000 dps range.
 *
 * Usage (in main.cpp after can_drv_init()):
 *   can_motor_register(1, CAN_MOTOR_RMD);    // hip FL
 *   can_motor_register(5, CAN_MOTOR_ODRIVE);  // wheel L
 *
 * Control (from ControlThread):
 *   can_motor_set_torque(1, 2.5f);   // Nm (placeholder scale, see above)
 *   can_motor_set_velocity(5, 10.f); // rad/s
 */

#define CAN_MOTOR_MAX 8

typedef enum {
    CAN_MOTOR_RMD    = 0,   // MyActuator/LKMTECH RMD-X protocol
    CAN_MOTOR_SDC102 = 1,   // Steadywin GDS6 (SDC102) native protocol — stub
    CAN_MOTOR_ODRIVE = 2,   // ODrive CAN Simple protocol (GDS68 running ODrive fw)
} CanMotorProto;

struct CanMotorState {
    float   pos_rad;     // joint position (rad)
    float   vel_rads;    // joint velocity (rad/s)
    float   torque_Nm;   // actual torque feedback (Nm)
    float   temp_C;      // winding temperature (°C)
    bool    valid;       // true once at least one feedback frame has arrived
};

// Register a motor.  id is the motor's slot — the number every other
// function (can_motor_set_torque, can_motor_get_state, ActuatorSafety, ...)
// looks it up by, and (for RMD/SDC102) also the CAN node ID.
//
// node_id (ODRIVE only): the ODrive's own configured CAN node id, if it
// differs from id. Defaults to 0, meaning "same as id". Use this whenever a
// physical drive is already configured with a node id that collides with
// another motor's slot number (e.g. a drive left at node id 2 from a
// previous project, while this project's slot 2 is already the hip FR RMD
// motor) — id stays whatever this project's own numbering needs, node_id
// carries the value actually needed on the wire.
// Must be called after can_drv_init() and before threads_start().
void can_motor_register(uint8_t id, CanMotorProto proto, uint8_t node_id = 0);

// Send a torque command (Nm).  Returns false if motor id not registered.
// RMD motors: converted through can_motor_set_rmd_torque_scale() — see the
// header comment above, this Nm figure is not independently calibrated.
bool can_motor_set_torque(uint8_t id, float torque_Nm);

// RMD only: send the drive's raw iqControl value directly (confirmed range
// -2048..2048, -33..33A for MG series — see header comment). Returns false
// if id not registered or not CAN_MOTOR_RMD.
bool can_motor_set_torque_raw(uint8_t id, int16_t ratio);

// RMD only: get/set the Nm<->ratio scale used by can_motor_set_torque().
void  can_motor_set_rmd_torque_scale(float ratio_per_Nm);
float can_motor_get_rmd_torque_scale(void);

// Send a velocity command (rad/s). RMD motors are clamped to the drive's
// confirmed +/-24000 dps range.
bool can_motor_set_velocity(uint8_t id, float vel_rads);

// Copy the latest state for motor id into *out.  Returns false if not registered.
bool can_motor_get_state(uint8_t id, CanMotorState *out);

// Read-only access to the global motor state table (indexed 0..CAN_MOTOR_MAX-1).
extern CanMotorState g_motors[CAN_MOTOR_MAX];
extern mutex_t       motor_state_mtx;
