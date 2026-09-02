#include "src/coms/Radio.hpp"
#include "src/coms/SBUS.hpp"
#include <cmath>

/* Channel order from transmitter — see the full table in Radio.hpp. All
 * used channels are [-1, 1] switches/sticks (11-bit range 172-1811, centre
 * 992) except the arm switch, which is a plain threshold.
 *
 * A small deadband is applied around center so stick/switch noise at rest
 * doesn't produce a nonzero command; the range beyond the deadband is
 * rescaled back onto [-1, 1] so full stick throw still reaches +/-1. */
static constexpr float STICK_DEADBAND = 0.05f;   // +/-5% around center

static float norm_axis(uint16_t v)
{
    float x = (float)(v - 992) / 819.0f;
    if (x > -STICK_DEADBAND && x < STICK_DEADBAND) return 0.0f;
    float mag = (fabsf(x) - STICK_DEADBAND) / (1.0f - STICK_DEADBAND);
    return (x > 0.0f) ? mag : -mag;
}

void  radio_input_init()   { g_sbus.init();   }
void  radio_input_update() { g_sbus.update(); }

float radio_yaw_stick()   { return norm_axis(g_sbus.channel(0)); }
float radio_vel_tgt()     { return -norm_axis(g_sbus.channel(1)); }   // inverted per user request
float radio_height_set()  { return norm_axis(g_sbus.channel(2)); }
float radio_leanover()    { return norm_axis(g_sbus.channel(3)); }
bool  radio_armed()
{
    // Inverted per user request (low = armed). Before any valid SBUS frame
    // has ever been decoded, channel(4) reads 0 (SbusParser's zero-initialized
    // default) -- with a plain "< 992u" that would read as ARMED with no
    // receiver connected at all. Gate on frame_lost()/failsafe() (both
    // default true until the first good frame) so "no signal yet" is always
    // disarmed, regardless of which raw threshold direction is used.
    if (g_sbus.frame_lost() || g_sbus.failsafe()) return false;
    return g_sbus.channel(4) < 992u;
}
float radio_mode_sw()     { return norm_axis(g_sbus.channel(6)); }
