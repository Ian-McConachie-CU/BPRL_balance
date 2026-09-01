#pragma once
#include "hal.h"

/*
 * RC radio input — SBUS on TELEM2 port (USART3, PD9 RX, AF7).
 * 100000 baud, 8E2, inverted in the USART itself (CR2 RXINV) since this
 * port has no hardware inverter — see SBUS.hpp for why USART6/PC7 (the
 * carrier board's "SBUSo" pin) can't be used instead.
 * 25-byte fixed frame, 16 × 11-bit channels (172–1811, center 992).
 */
#define RADIO_PROTO_SBUS  0

#ifndef RADIO_PROTOCOL
#define RADIO_PROTOCOL    RADIO_PROTO_SBUS
#endif

void  radio_input_init(void);
void  radio_input_update(void);

float radio_thr(void);          /* throttle       [0, 1]  */
float radio_roll(void);         /* roll           [-1, 1] */
float radio_pitch(void);        /* pitch          [-1, 1] */
float radio_yaw(void);          /* yaw rate       [-1, 1] */
float radio_mode_sw(void);      /* mode switch    [-1, 1] from channel 5 */
bool  radio_armed(void);

/* Velocity target and controller-select switches are not yet wired up on
 * the transmitter -- these read placeholder channels (6, 7) so the rest of
 * the input pipeline (InputIdx::VEL_TGT / CTRL_SEL) has something to read
 * until the physical switches are assigned. Update the channel indices in
 * Radio.cpp once they are. */
float radio_vel_tgt(void);      /* forward velocity demand [-1, 1] */
float radio_ctrl_sel(void);     /* controller select       [-1, 1] */
