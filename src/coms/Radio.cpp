#include "src/coms/Radio.hpp"
#include "src/coms/SBUS.hpp"

/*
 * Channel order from transmitter (11-bit range 172–1811, centre 992):
 *   ch[0] = Throttle       → [0, 1]
 *   ch[1] = Roll           → [-1, 1]
 *   ch[2] = Pitch          → [-1, 1]
 *   ch[3] = Yaw            → [-1, 1]
 *   ch[4] = Arm switch     → >992 = armed
 *   ch[5] = Mode switch    → [-1, 1]
 *   ch[6] = Velocity target   → [-1, 1]  -- PLACEHOLDER, no physical switch
 *                                           assigned yet, see Radio.hpp
 *   ch[7] = Controller select → [-1, 1]  -- PLACEHOLDER, ditto
 */
static float norm_axis(uint16_t v) { return (float)(v - 992)  / 819.0f;  }
static float norm_thr (uint16_t v) { return (float)(v - 172)  / 1639.0f; }

void  radio_input_init()   { g_sbus.init();   }
void  radio_input_update() { g_sbus.update(); }

float radio_thr()         { return norm_thr (g_sbus.channel(0)); }
float radio_roll()        { return norm_axis(g_sbus.channel(1)); }
float radio_pitch()       { return -norm_axis(g_sbus.channel(2)); }
float radio_yaw()         { return norm_axis(g_sbus.channel(3)); }
bool  radio_armed()       { return g_sbus.channel(4) > 992u;    }
float radio_mode_sw()     { return norm_axis(g_sbus.channel(5)); }
float radio_vel_tgt()     { return norm_axis(g_sbus.channel(6)); }
float radio_ctrl_sel()    { return norm_axis(g_sbus.channel(7)); }
