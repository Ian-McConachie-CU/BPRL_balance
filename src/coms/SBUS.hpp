#pragma once
#include "hal.h"
#include <cstdint>

/*
 * SBUS RC input parser — USART3 (TELEM2 port), PD9 RX, AF7
 *
 * NOTE: USART6/PC7 (the port this used to target, labelled "SBUSo" in
 * mcuconf.h) is NOT usable for this. On real CubeOrange/CubeOrange+
 * hardware, USART6 is the internal link between the H743 flight MCU and
 * a separate STM32F103 IO co-processor — it is never brought out to an
 * external connector. Both the carrier board's "RC IN" pin and its
 * "SBUS out" port are wired only to that IO co-processor too. Since this
 * firmware doesn't run any IO co-processor image, none of those nets are
 * reachable from application code — SBUS has to come in on a UART that's
 * wired directly to the H743, hence TELEM2/USART3 instead.
 *
 * Frame: 25 bytes  [0x0F][22 channel bytes][flags][0x00]
 * Serial: 100000 baud, 8E2 (8 data + even parity + 2 stop bits)
 * Signal: SBUS is inverted at the receiver. USART6's RC-input path has a
 *         hardware inverter on the Cube PCB, but TELEM2 does not, so this
 *         driver inverts in the USART itself via CR2 RXINV.
 *
 * 16 channels packed as 11-bit values (range 172–1811, centre 992).
 * Flags byte (buf[23]): bit 2 = frame_lost, bit 3 = failsafe.
 *
 * frame_lost()/failsafe() only change value inside a successfully decoded
 * frame -- they don't self-clear on their own. That's fine while the
 * receiver is alive (it keeps sending failsafe-flagged frames when it
 * loses the transmitter's RF signal), but if the wire itself goes dead
 * (cable pulled, receiver powered off) bytes just stop arriving and both
 * flags freeze at their last value forever. take_frame_received()/
 * mark_stale() exist so the caller (RadioThread, threads.cpp) can run a
 * tick-based watchdog on top and force the link back to the "no signal"
 * state -- mirrors StateEstThread's CAN-IMU staleness counter.
 */
class SbusParser {
public:
    void     init();
    void     update();                  // drain SD3, run state machine; call at ~100 Hz
    uint16_t channel(uint8_t n) const; // raw 11-bit value for channel n (0–15)
    bool     frame_lost() const  { return _frame_lost; }
    bool     failsafe()   const  { return _failsafe;   }

    // True if a full valid frame was decoded since the last call (consumes
    // the flag). Drive a staleness watchdog off this, not frame_lost() --
    // see class header.
    bool     take_frame_received();
    // Force frame_lost()/failsafe() back to their "no signal" default.
    // Call from a watchdog once too much time has passed with no frame
    // received, so a silently dead link still ends up disarmed.
    void     mark_stale();

private:
    static void unpack(const uint8_t *payload, uint16_t *ch);

    enum class State : uint8_t { WAIT_START, IN_FRAME } _state{State::WAIT_START};
    uint8_t  _buf[25]{};
    uint8_t  _count{0};
    uint16_t _ch[16]{};
    bool     _frame_lost{true};
    bool     _failsafe{true};
    bool     _frame_received{false};
};

extern SbusParser g_sbus;
