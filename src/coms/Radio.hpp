#pragma once
#include "hal.h"

/*
 * RC radio input — SBUS on SBUSo port (USART6, PC7 RX, AF8).
 * 100000 baud, 8E2, hardware-inverted on CubeOrangePlus PCB.
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
