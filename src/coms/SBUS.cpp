#include "src/coms/SBUS.hpp"

SbusParser g_sbus;

/*
 * SBUS on USART3 (TELEM2 port, PD9 RX), SD3.
 * 100000 baud, 8E2 (8 data bits, even parity, 2 stop bits).
 * TELEM2 has no hardware inverter, so RXINV inverts the line in the USART.
 */
static const SerialConfig kSbusSerialCfg = {
    100000,                                  // baud
    USART_CR1_PCE | USART_CR1_M0,           // 9-bit word (8E: M0=1 sets 9-bit mode, PCE enables parity)
    USART_CR2_STOP_1 | USART_CR2_STOP_0 |   // 2 stop bits
    USART_CR2_RXINV,                        // invert RX line (SBUS idles low)
    0                                        // no CR3 flags
};

void SbusParser::init()
{
    sdStart(&SD3, &kSbusSerialCfg);
}

void SbusParser::update()
{
    uint8_t byte;
    while (chnReadTimeout(&SD3, &byte, 1, TIME_IMMEDIATE) == 1) {
        switch (_state) {
        case State::WAIT_START:
            if (byte == 0x0F) {
                _buf[0] = byte;
                _count  = 1;
                _state  = State::IN_FRAME;
            }
            break;

        case State::IN_FRAME:
            _buf[_count++] = byte;
            if (_count == 25) {
                if (_buf[24] == 0x00) {
                    unpack(&_buf[1], _ch);
                    _frame_lost     = (_buf[23] >> 2) & 0x01;
                    _failsafe       = (_buf[23] >> 3) & 0x01;
                    _frame_received = true;
                }
                _state = State::WAIT_START;
                _count = 0;
            }
            break;
        }
    }
}

uint16_t SbusParser::channel(uint8_t n) const
{
    return (n < 16) ? _ch[n] : 992u;
}

bool SbusParser::take_frame_received()
{
    const bool r = _frame_received;
    _frame_received = false;
    return r;
}

void SbusParser::mark_stale()
{
    _frame_lost = true;
    _failsafe   = true;
}

void SbusParser::unpack(const uint8_t *data, uint16_t *ch)
{
    for (int n = 0; n < 16; n++) {
        const int bit  = n * 11;
        const int bi   = bit / 8;
        const int sh   = bit % 8;
        const uint32_t w = (uint32_t)data[bi]
                         | ((uint32_t)data[bi + 1] << 8)
                         | ((uint32_t)data[bi + 2] << 16);
        ch[n] = (w >> sh) & 0x7FFu;
    }
}
