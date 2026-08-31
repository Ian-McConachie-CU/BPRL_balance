#pragma once
#include "src/CAN.hpp"
#include <cstdint>

/*
 * LK-TECH (Shanghai LingKong Technology) CAN driver for the MG8016E-i6 hip
 * motors — CONFIRMED against the vendor's own "CAN PROTOCOL V2.35" document
 * (user-provided), not the MyActuator reference this used to be guessed
 * from. Key facts from that document:
 *   - Command identifier: 0x140 + id (id = 1..32). The reply frame uses
 *     THE SAME identifier (0x140 + id), not 0x240 + id — this project's
 *     driver originally listened on 0x240+id (carried over from the
 *     MyActuator/RMD-X convention) and would never have seen a real reply.
 *     Reply arrives within 0.25 ms of the command.
 *   - Motor off (0x80) / on (0x88) / stop (0x81): confirmed, no payload.
 *   - Torque control (0xA1): iqControl is int16_t, range -2048..2048,
 *     corresponding to -33..33 A for the MG series specifically (other
 *     LK-TECH series differ). rmd_torque_raw() sends this directly.
 *     rmd_torque()'s Nm conversion is still a placeholder scale — the
 *     manual gives current, not torque; the motor's actual Kt (Nm/A) isn't
 *     documented, so calibrate rmd_set_torque_scale() against a real load.
 *   - Speed control (0xA2): int32_t, 0.01 dps/LSB — confirmed, matches what
 *     was already implemented.
 *   - Position control: 0xA3 is angle-only (int32_t, 0.01 deg/LSB, no speed
 *     limit); 0xA4 adds a speed limit (uint16_t, 1 dps/LSB) in data[2:3]
 *     ahead of the same angle field. rmd_position() takes a speed limit, so
 *     it now correctly sends 0xA4 (was incorrectly sending 0xA3 with the
 *     0xA4 payload layout before this was confirmed).
 *   - Read state1 (0x9A) / clear error (0x9B): data[1]=temp (int8, 1C/LSB),
 *     data[7]=errorState (single byte: bit0=under-voltage, bit3=over-temp,
 *     all other bits invalid/unused) confirmed against real hardware.
 *     Voltage does NOT match the vendor doc — real MG8016E-i6 replies put it
 *     at data[2:3] (LSB order) and 0.01V/LSB, not the documented data[3:4]
 *     at 0.1V/LSB (confirmed 2026-08-31 via poll hips against a known 41V
 *     supply — see CAN_config.md §1.3 for the full writeup).
 *   - Read encoder (0x90): data[2:3]=encoder, uint16_t, LSB order, already
 *     relative to the drive's configured zero point — confirmed against
 *     real hardware (2026-08-31), including cross-checking encoder against
 *     encoderRaw/encoderOffset in the same reply. One correction from the
 *     vendor doc: it's a 16-bit field here (0..65535), not the doc's generic
 *     14-bit (0..16383) example — this MG8016E-i6/DG80R7E's own GUI reports
 *     "Encoder Type: 16Bit Encoder" (Ktech_manual.pdf), and raw values seen
 *     on the bus (e.g. 54735) only make sense as a single-turn 0..360deg
 *     position at 16-bit scale.
 *   - Single loop angle control (0xA6): data[1]=spinDirection (0x00=CW,
 *     0x01=CCW — the direction commanded to REACH the target, not derived
 *     from target-vs-current), data[2:3]=maxSpeed (uint16_t, 1 dps/LSB),
 *     data[4:7]=angleControl (uint32_t, 0.01 deg/LSB, ABSOLUTE but
 *     SINGLE-TURN: 0..359.99 deg, wraps every revolution). Added 2026-08-31
 *     alongside 0xA7 — this is the mode that actually matches "move to X
 *     degrees" in the same reference frame the 0x90 encoder reads, as
 *     opposed to 0xA4's absolute multi-turn target. (0xA5 is the
 *     no-speed-limit variant, not wired up here.)
 *   - Increment angle control (0xA7): int32_t angleIncrement, 0.01 deg/LSB,
 *     signed — RELATIVE move from the drive's current position, no speed
 *     limit field (uses whatever Max Speed is configured on the drive).
 *     Added 2026-08-31 specifically because 0xA4 (Multi Loop Angle Control)
 *     targets an ABSOLUTE multi-turn position, which is a different
 *     reference frame than the single-turn 0x90 encoder reading this
 *     project uses everywhere else — after a session of hand-spinning the
 *     shaft during testing, "target 0deg"/"target 120deg" via 0xA4 could be
 *     asking for a real multi-turn travel of many hundreds of degrees, which
 *     looks identical to "barely moving" if you're comparing against the
 *     single-turn reading and waiting only a few seconds. 0xA7 sidesteps
 *     that entirely by moving relative to wherever the shaft already is.
 *   - Also documented but not yet wired up here: open-loop control (0xA0,
 *     MS series only, not applicable to MG8016E-i6), single-loop angle
 *     control without a speed limit (0xA5), increment angle with speed
 *     limit (0xA8), PID/acceleration read-write (0x30-0x34 — 0x30/0x33 sent fine over CAN
 *     per poll hips but this drive did not reply to either, so treat those
 *     specific reads as unsupported on this firmware rather than retrying),
 *     write-encoder-offset (0x91), multi/single-turn angle read (0x92/0x94),
 *     read state2/state3 (0x9C/0x9D), and a 4-motor broadcast torque frame
 *     (0x280, IDs 1-4 only).
 */

struct RmdState {
    float    pos_rad;
    float    vel_rads;
    float    torque_Nm;
    float    temp_C;
    float    voltage_V;     // from the 0x9A status1 response
    uint8_t  error_flags;   // raw byte from status1 response; bit0=under-voltage, bit3=over-temp
    bool     valid;
    uint32_t last_update_ms;
};

#define RMD_ID_MAX 32   // valid motor IDs are 1..32

void rmd_init(void);            // subscribes to CAN RX; call once after can_drv_init()
void rmd_set_bus(CanBus bus);   // bus used for subsequent commands (default CAN_BUS_1)
CanBus rmd_get_bus(void);

// Confirmed range: -2048..2048, corresponds to -33..33 A for the MG series.
// Use this directly if you don't trust the Nm conversion.
bool rmd_torque_raw(uint8_t id, int16_t ratio);                  // 0xA1

// Nm convenience wrapper around rmd_torque_raw() — see rmd_set_torque_scale().
bool rmd_torque(uint8_t id, float torque_Nm);                    // 0xA1
void  rmd_set_torque_scale(float ratio_per_Nm);
float rmd_get_torque_scale(void);

bool rmd_velocity(uint8_t id, float vel_rads);                   // 0xA2, clamped to +/-24000 dps
bool rmd_position(uint8_t id, float angle_rad, float maxspeed_rads); // 0xA4 (angle + speed limit) — ABSOLUTE multi-turn target, see header note
bool rmd_position_single_turn(uint8_t id, float angle_rad, float maxspeed_rads, bool cw = true); // 0xA6 — ABSOLUTE single-turn (0..360deg) target, same frame as rmd_read_encoder()
bool rmd_increment_position(uint8_t id, float delta_rad);        // 0xA7 — RELATIVE move from current position
bool rmd_stop(uint8_t id);      // 0x81 — zero output, stays enabled
bool rmd_off(uint8_t id);       // 0x80 — disable closed loop
bool rmd_resume(uint8_t id);    // 0x88 — resume closed loop after stop/off
bool rmd_request_status(uint8_t id);  // 0x9A — request status1 (temp/voltage/error)
bool rmd_clear_error(uint8_t id);     // 0x9B
bool rmd_read_encoder(uint8_t id);    // 0x90 — request current single-turn encoder position (non-motion, safe; confirmed against hardware, see header note)

bool rmd_get_state(uint8_t id, RmdState &out);

// Immediately zero-torque every RMD id that has ever reported valid feedback.
void rmd_stop_all(void);
