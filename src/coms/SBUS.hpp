#pragma once
#include "hal.h"
#include <cstdint>

/*
 * SBUS RC input parser — USART6 (SBUSo port), PC7 RX, AF8
 *
 * Frame: 25 bytes  [0x0F][22 channel bytes][flags][0x00]
 * Serial: 100000 baud, 8E2 (8 data + even parity + 2 stop bits)
 * Signal: inverted at source; Cube HW inverter normalises before MCU pin,
 *         so no USART_CR2_RXINV needed.
 *
 * 16 channels packed as 11-bit values (range 172–1811, centre 992).
 * Flags byte (buf[23]): bit 2 = frame_lost, bit 3 = failsafe.
 */
class SbusParser {
public:
    void     init();
    void     update();                  // drain SD6, run state machine; call at ~100 Hz
    uint16_t channel(uint8_t n) const; // raw 11-bit value for channel n (0–15)
    bool     frame_lost() const  { return _frame_lost; }
    bool     failsafe()   const  { return _failsafe;   }

private:
    static void unpack(const uint8_t *payload, uint16_t *ch);

    enum class State : uint8_t { WAIT_START, IN_FRAME } _state{State::WAIT_START};
    uint8_t  _buf[25]{};
    uint8_t  _count{0};
    uint16_t _ch[16]{};
    bool     _frame_lost{true};
    bool     _failsafe{true};
};

extern SbusParser g_sbus;
