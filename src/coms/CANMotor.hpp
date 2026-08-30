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
 *                       stub — fill frame format from GIM6010 datasheet
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
 *   can_motor_register(5, CAN_MOTOR_SDC102); // wheel L
 *
 * Control (from ControlThread):
 *   can_motor_set_torque(1, 2.5f);   // Nm (placeholder scale, see above)
 *   can_motor_set_velocity(5, 10.f); // rad/s
 */

#define CAN_MOTOR_MAX 8

typedef enum {
    CAN_MOTOR_RMD    = 0,   // MyActuator/LKMTECH RMD-X protocol
    CAN_MOTOR_SDC102 = 1,   // Steadywin GDS6 (SDC102) protocol
} CanMotorProto;

struct CanMotorState {
    float   pos_rad;     // joint position (rad)
    float   vel_rads;    // joint velocity (rad/s)
    float   torque_Nm;   // actual torque feedback (Nm)
    float   temp_C;      // winding temperature (°C)
    bool    valid;       // true once at least one feedback frame has arrived
};

// Register a motor.  id is the motor's CAN node ID (1–32 for RMD).
// Must be called after can_drv_init() and before threads_start().
void can_motor_register(uint8_t id, CanMotorProto proto);

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
