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
 *   - Read state1 (0x9A) / clear error (0x9B): confirmed layout is
 *     data[1]=temp (int8, 1C/LSB), data[3:4]=voltage (uint16, 0.1V/LSB),
 *     data[7]=errorState (single byte: bit0=under-voltage, bit3=over-temp,
 *     all other bits invalid/unused) — this project's driver previously had
 *     the voltage/error byte positions wrong (guessed from the MyActuator
 *     layout).
 *   - Also documented but not yet wired up here: open-loop control (0xA0,
 *     MS series only, not applicable to MG8016E-i6), single-loop/increment
 *     angle control (0xA5-0xA8), PID/acceleration/encoder read-write
 *     (0x30-0x34, 0x90-0x95), read state2/state3 (0x9C/0x9D), and a 4-motor
 *     broadcast torque frame (0x280, IDs 1-4 only).
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
bool rmd_position(uint8_t id, float angle_rad, float maxspeed_rads); // 0xA4 (angle + speed limit)
bool rmd_stop(uint8_t id);      // 0x81 — zero output, stays enabled
bool rmd_off(uint8_t id);       // 0x80 — disable closed loop
bool rmd_resume(uint8_t id);    // 0x88 — resume closed loop after stop/off
bool rmd_request_status(uint8_t id);  // 0x9A — request status1 (temp/voltage/error)
bool rmd_clear_error(uint8_t id);     // 0x9B

bool rmd_get_state(uint8_t id, RmdState &out);

// Immediately zero-torque every RMD id that has ever reported valid feedback.
void rmd_stop_all(void);
