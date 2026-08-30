#include "src/Imx5.hpp"
#include "ch.h"
#include <cmath>
#include <cstring>

static Imx5State s_state = {};

static inline int16_t le16s(const uint8_t *p)
{
    int16_t v;
    __builtin_memcpy(&v, p, sizeof(v));
    return v;
}

static void imx5_rx_cb(CanBus bus, const CANRxFrame &f, void *ctx)
{
    (void)ctx;
    if (bus != CAN_BUS_2 || f.common.XTD || f.DLC < 8) return;

    uint32_t now = (uint32_t)TIME_I2MS(chVTGetSystemTime());
    switch (f.std.SID) {
    case 0x01:
        s_state.q0 = le16s(&f.data8[0]) * (1.0f / 10000.0f);
        s_state.q1 = le16s(&f.data8[2]) * (1.0f / 10000.0f);
        s_state.q2 = le16s(&f.data8[4]) * (1.0f / 10000.0f);
        s_state.q3 = le16s(&f.data8[6]) * (1.0f / 10000.0f);
        s_state.valid = true;
        s_state.last_quat_ms = now;
        break;
    case 0x02:
        s_state.p  = le16s(&f.data8[0]) * (1.0f / 1000.0f);
        s_state.ax = le16s(&f.data8[2]) * (1.0f / 100.0f);
        s_state.valid = true;
        s_state.last_rate_ms = now;
        break;
    case 0x03:
        s_state.q  = le16s(&f.data8[0]) * (1.0f / 1000.0f);
        s_state.ay = le16s(&f.data8[2]) * (1.0f / 100.0f);
        break;
    case 0x04:
        s_state.r  = le16s(&f.data8[0]) * (1.0f / 1000.0f);
        s_state.az = le16s(&f.data8[2]) * (1.0f / 100.0f);
        break;
    default:
        break;
    }
}

void imx5_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    can_subscribe(imx5_rx_cb, nullptr);
}

bool imx5_get_state(Imx5State &out)
{
    out = s_state;
    return s_state.valid;
}

void imx5_quat_to_euler(float q0, float q1, float q2, float q3,
                         float &roll, float &pitch, float &yaw)
{
    roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                    1.0f - 2.0f * (q1 * q1 + q2 * q2));
    pitch = asinf(fmaxf(-1.0f, fminf(1.0f, 2.0f * (q0 * q2 - q3 * q1))));
    yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                    1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
