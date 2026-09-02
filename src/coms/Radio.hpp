#pragma once
#include "hal.h"

/*
 * RC radio input — SBUS on TELEM2 port (USART3, PD9 RX, AF7).
 * 100000 baud, 8E2, inverted in the USART itself (CR2 RXINV) since this
 * port has no hardware inverter — see SBUS.hpp for why USART6/PC7 (the
 * carrier board's "SBUSo" pin) can't be used instead.
 * 25-byte fixed frame, 16 × 11-bit channels (172–1811, center 992). All
 * [-1,1] channels below have a small deadband applied around center (see
 * norm_axis() in Radio.cpp).
 *
 * Channel map (this robot's transmitter — a wheeled biped, not a drone, so
 * this is NOT the usual throttle/roll/pitch/yaw layout):
 *
 *   ch0  Yaw stick                    -> radio_yaw_stick()  -> InputIdx::YAW_STICK
 *   ch1  Forward velocity target      -> radio_vel_tgt()    -> InputIdx::VEL_TGT
 *                                         (INVERTED -- see Radio.cpp)
 *   ch2  Height-set switch            -> radio_height_set() -> InputIdx::HEIGHT_SET
 *                                         (placeholder -- no controller consumes
 *                                         yet; hold-to-zero-on-arm logic in
 *                                         RadioThread, see threads.cpp)
 *   ch3  Leanover switch              -> radio_leanover()   -> InputIdx::LEANOVER
 *                                         (placeholder -- no controller consumes yet)
 *   ch4  Arm switch (AuxF)            -> radio_armed()      -> g_armed
 *                                         (INVERTED: low = armed -- see Radio.cpp)
 *   ch5  AuxA                         -- reserved, not wired to anything yet
 *   ch6  Mode select switch (AuxB,    -> radio_mode_sw()    -> InputIdx::MODE_SW
 *        two-position: car/balance)
 *   ch7  AuxH                         -- reserved, not wired to anything yet
 *   ch8  AuxD                         -- reserved, not wired to anything yet
 *   ch9  AuxC                         -- reserved, not wired to anything yet
 *   ch10-15                           -- unused
 *
 * See README.md for the full channel-map table and the (not yet
 * implemented) planned car/balance mode state machine this feeds into.
 */
#define RADIO_PROTO_SBUS  0

#ifndef RADIO_PROTOCOL
#define RADIO_PROTOCOL    RADIO_PROTO_SBUS
#endif

void  radio_input_init(void);
void  radio_input_update(void);

float radio_yaw_stick(void);    /* ch0: yaw stick            [-1, 1] */
float radio_vel_tgt(void);      /* ch1: forward velocity demand [-1, 1] */
float radio_height_set(void);   /* ch2: height-set switch    [-1, 1] -- placeholder */
float radio_leanover(void);     /* ch3: leanover switch      [-1, 1] -- placeholder */
bool  radio_armed(void);        /* ch4 (AuxF): arm switch */
float radio_mode_sw(void);      /* ch6 (AuxB): car/balance mode select [-1, 1] */
