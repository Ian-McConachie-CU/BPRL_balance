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
 *                       for the full writeup of how that was found and fixed).
 *                       Torque/velocity commands' own replies (0xA1/0xA2)
 *                       carry temp/iq/speed/encoder every time one is sent;
 *                       voltage_V/error_flags come from a separate 0x9A
 *                       ReadState1 request that nothing sends automatically
 *                       — see can_motor_poll_status_round_robin() below.
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
 * Hip (RMD) safety: every torque/velocity/position command to a hip motor —
 * from ControlThread's normal control path, a USB debug command, anything
 * future — passes through ONE final gate in CANMotor.cpp (hip_soft_scale(),
 * hip_clamp_velocity(), hip_clamp_position_target()) before it's ever put on
 * the bus: a progressive soft-ramp toward zero as the joint nears its angle
 * bound (HIP_ANGLE_MIN/MAX_RAD, hardcoded — edit + recompile + reflash to
 * change), a velocity soft-ramp, and a hard torque clamp. There is no way to
 * reach a hip motor that skips this. Each hip also has a hardcoded zero-
 * offset (HIP_OFFSET_RAD) and mirror sign (HIP_SIGN, +1/-1 — left/right hip
 * pairs are mechanically mirror images, same concept as can_motor_register()'s
 * ODRIVE sign param below) applied at decode time, so CanMotorState::pos_rad
 * is ALWAYS the robot-frame angle (mounting-orientation-independent, and
 * consistent in sign across every hip), never the raw encoder reading — see
 * CANMotor.cpp's "Hip zero-offset + sign + safety bounds" section for the
 * calibration workflow and all three constants.
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
    float    pos_rad;      // joint position (rad)
    float    vel_rads;     // joint velocity (rad/s)
    float    torque_Nm;    // actual torque feedback (Nm)
    float    temp_C;       // winding temperature (°C)
    float    voltage_V;    // RMD only: bus voltage at the drive, from its 0x9A status reply
                            // (0 for ODRIVE/SDC102 — no request wired up for those protocols yet)
    uint8_t  error_flags;  // RMD only: raw errorState bitmask from 0x9A (0 = no fault); ditto
    bool     valid;        // true once at least one feedback frame has arrived

    // Per-motor send/receive counters -- diagnostic, added 2026-09-02 while
    // chasing hips 2-4 responding far less often than hip1 on real hardware.
    // CAN.cpp's CANDiag tx_ok/tx_fail are bus-WIDE aggregates and couldn't
    // distinguish "we're failing to SEND to this motor specifically" (would
    // show as tx_fail climbing here) from "we send fine but IT isn't
    // replying as often" (tx_fail flat, rx_count climbing slower than a
    // healthy motor's). RMD only for now (the two call sites that increment
    // these are RMD-specific) -- ODRIVE/SDC102 entries leave both at 0.
    uint32_t tx_ok;         // can_motor_set_torque/_raw/_velocity calls that got a CAN ACK
    uint32_t tx_fail;       // ...that didn't (see CAN.cpp's can_send() -- no free mailbox etc.)
    uint32_t rx_count;      // replies received on this motor's SID, ANY command byte
                            // (unlike `valid`, counts every frame, not just decodable ones)
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
//
// sign (ODRIVE only): +1.0 (default) or -1.0. Applied to every command
// (torque/velocity, see can_motor_set_torque()/can_motor_set_velocity()) AND
// every decoded feedback value (pos_rad/vel_rads, see odrive_rx_cb() in the
// .cpp) for this motor, at the single chokepoint here -- same "gate at the
// source, every caller gets it for free" pattern as the hip offset/bounds.
// Use -1.0 for a wheel that's mechanically mirrored relative to this
// project's sign convention (e.g. wheel L vs wheel R both spinning "forward"
// for the robot, but wired/mounted as mirror images of each other) so a
// positive command means the same physical direction for both, and a
// positive pos_rad/vel_rads means the same physical direction too.
// Must be called after can_drv_init() and before threads_start().
void can_motor_register(uint8_t id, CanMotorProto proto, uint8_t node_id = 0, float sign = 1.0f);

// Send a torque command (Nm).  Returns false if motor id not registered.
// RMD motors: converted through can_motor_set_rmd_torque_scale() — see the
// header comment above, this Nm figure is not independently calibrated.
bool can_motor_set_torque(uint8_t id, float torque_Nm);

// RMD (hips 1-4) only, multi-motor broadcast (0x280): sends all 4 hips'
// torque in ONE frame instead of 4 separate can_motor_set_torque() calls.
// torques[0..3] correspond to ids 1..4. Each is independently safety-
// clamped exactly as can_motor_set_torque() would -- see CANMotor.cpp's
// "Hip zero-offset + safety bounds" section. Requires ids 1-4 all
// registered as CAN_MOTOR_RMD AND every hip drive configured for broadcast
// mode (its own GUI tool) at a matching baud rate. Returns false (sends
// nothing) if ids 1-4 aren't all RMD.
bool can_motor_set_hip_torques_broadcast(const float torques[4]);

// RMD only: send the drive's raw iqControl value directly (confirmed range
// -2048..2048, -33..33A for MG series — see header comment). Returns false
// if id not registered or not CAN_MOTOR_RMD.
bool can_motor_set_torque_raw(uint8_t id, int16_t ratio);

// RMD only: get/set the Nm<->ratio scale used by can_motor_set_torque().
void  can_motor_set_rmd_torque_scale(float ratio_per_Nm);
float can_motor_get_rmd_torque_scale(void);

// Send a velocity command (rad/s). RMD motors are clamped to the drive's
// confirmed +/-24000 dps range (and, for hips, the safety layer's soft
// angle-bound ramp -- see the header comment above).
bool can_motor_set_velocity(uint8_t id, float vel_rads);

// RMD (hip) only -- no position-mode wired up for SDC102/ODRIVE. target_rad
// is the ROBOT-FRAME angle (offset applied automatically, target clamped to
// the hip safety bounds before it's sent); maxspeed_rads clamped to the same
// hip velocity rating everything else uses. cw selects which way the motor
// turns to reach the target (default true/CW -- no shortest-path
// auto-selection). Returns false if id not registered or not CAN_MOTOR_RMD.
bool can_motor_set_position(uint8_t id, float target_rad, float maxspeed_rads, bool cw = true);

// Hip safety bounds (robot-frame, post-offset), for anything that needs to
// know the range without duplicating CANMotor.cpp's HIP_ANGLE_MIN/MAX_RAD
// (e.g. a bench sweep-test tool). Returns 0.0f if id isn't 1-4.
float can_motor_hip_angle_min(uint8_t id);
float can_motor_hip_angle_max(uint8_t id);

// ODRIVE only: switch the axis's own closed-loop controller mode between
// velocity and torque at runtime (always PASSTHROUGH input mode), via the
// same IDLE -> mode -> CLOSED_LOOP sequence odrive_init_axis() uses at boot
// -- a bare mode change while already closed-loop can fault the axis back
// to IDLE on some ODrive firmware versions (found 2026-09-02, real
// hardware: "wheel doesn't turn" after a runtime mode switch). The axis is
// set to TORQUE once at boot -- this is for a caller that needs VELOCITY
// mode temporarily (e.g. MotorTest's wheel sweep, confirmed on real
// hardware to track a commanded velocity much better than a software
// torque-PID loop, which stalled under the wheel's own static friction)
// and must switch back to TORQUE when done, since nothing else in this
// codebase expects a wheel to still be in velocity mode.
//
// settle: true blocks for ~150ms between steps (reliable; use from
// non-timing-critical callers like USBCmdThread). false fires all three
// commands back-to-back with no delay (best-effort; use from a
// timing-critical caller, e.g. ControlThread's arm-triggered abort path,
// where a 150ms stall would itself be the worse problem).
//
// Returns false if id not registered or not CAN_MOTOR_ODRIVE.
bool can_motor_set_odrive_mode(uint8_t id, bool velocity_mode, bool settle = true);

// Copy the latest state for motor id into *out.  Returns false if not registered.
bool can_motor_get_state(uint8_t id, CanMotorState *out);

// ODRIVE only: actively request Get_Encoder_Estimates via an RTR frame,
// instead of waiting on the drive's own periodic broadcast -- see
// can_send_rtr() (src/coms/CAN.hpp). Call this every ControlThread tick
// (or at whatever divisor bus load allows) to keep wheel pos_rad/vel_rads
// fresh at control-loop rate, the same way RMD hips already are via their
// torque/velocity command replies. Returns false if id not registered or
// not CAN_MOTOR_ODRIVE.
bool can_motor_request_encoder(uint8_t id);

// RMD only: request a 0x9A (ReadState1) status frame — this is what
// populates voltage_V/error_flags above. Unlike temp/pos/vel/torque (which
// piggyback on every torque/velocity command's own reply, see CANMotor.cpp),
// voltage and error state are only refreshed when this is sent — nothing
// requests it automatically. Returns false if id not registered or not
// CAN_MOTOR_RMD.
bool can_motor_request_status(uint8_t id);

// Convenience: sends can_motor_request_status() for one registered RMD
// motor per call, round-robining through all of them across successive
// calls. Call this at a slow, fixed rate (e.g. a few Hz from ControlThread,
// throttled) rather than requesting every hip's status every control tick —
// voltage/error don't change fast enough to justify 0x9A at control-loop
// rate, and this keeps that traffic off the bus otherwise. No-op if no RMD
// motors are registered.
void can_motor_poll_status_round_robin(void);

// Read-only access to the global motor state table (indexed 0..CAN_MOTOR_MAX-1).
extern CanMotorState g_motors[CAN_MOTOR_MAX];
extern mutex_t       motor_state_mtx;
