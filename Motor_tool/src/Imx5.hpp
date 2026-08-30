#pragma once
#include "src/CAN.hpp"
#include <cstdint>

/*
 * Inertial Sense IMX5 INS decoder — CAN bus 2, standard IDs 0x01-0x04.
 *
 * Ported directly from BPRL_balance's src/coms/CAN.cpp (imx5_can_cb) — this
 * is the one CAN device in this tool with an already-proven byte layout
 * (not a guess), which makes it useful as a known-good reference: if the
 * IMX5 has worked before but shows nothing here, that points at a bug in
 * this tool's CAN path rather than wiring, and if it decodes fine here it
 * isolates a motor-bus (bus 1) problem from the CAN hardware/driver itself.
 */

struct Imx5State {
    // Quaternion NED->Body from CID_INS_QUATN2B [W,X,Y,Z], Hamilton convention.
    float    q0, q1, q2, q3;
    float    p, q, r;        // body-frame angular rates, rad/s
    float    ax, ay, az;     // m/s^2
    bool     valid;          // true once at least one frame decoded
    uint32_t last_quat_ms;
    uint32_t last_rate_ms;
};

void imx5_init(void);   // subscribes to CAN RX; call once after can_drv_init()
bool imx5_get_state(Imx5State &out);

// Quaternion -> roll/pitch/yaw (rad), same formulas as threads.cpp's DebugThread.
void imx5_quat_to_euler(float q0, float q1, float q2, float q3,
                         float &roll, float &pitch, float &yaw);
