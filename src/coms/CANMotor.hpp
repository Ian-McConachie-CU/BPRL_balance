#pragma once
#include "ch.h"
#include "hal.h"

/*
 * CAN motor driver — up to CAN_MOTOR_MAX motors on FDCAN1 (bus 1).
 *
 * Supported protocols:
 *   CAN_MOTOR_RMD     — MyActuator/LKMTECH RMD-X series (MG8016E-i6 hip motors)
 *                       cmd at 0x140+id, response at 0x240+id
 *   CAN_MOTOR_SDC102  — Steadywin GDS6/SDC102 driver (GIM6010-6 wheel motors)
 *                       stub — fill frame format from GIM6010 datasheet
 *
 * Usage (in main.cpp after can_drv_init()):
 *   can_motor_register(1, CAN_MOTOR_RMD);    // hip FL
 *   can_motor_register(5, CAN_MOTOR_SDC102); // wheel L
 *
 * Control (from ControlThread):
 *   can_motor_set_torque(1, 2.5f);   // Nm
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
bool can_motor_set_torque(uint8_t id, float torque_Nm);

// Send a velocity command (rad/s).
bool can_motor_set_velocity(uint8_t id, float vel_rads);

// Copy the latest state for motor id into *out.  Returns false if not registered.
bool can_motor_get_state(uint8_t id, CanMotorState *out);

// Read-only access to the global motor state table (indexed 0..CAN_MOTOR_MAX-1).
extern CanMotorState g_motors[CAN_MOTOR_MAX];
extern mutex_t       motor_state_mtx;
