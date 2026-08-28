#pragma once
#include <cstdint>

/*
 * Cascade PID controller — ported from FreeRTOS/Tiva PID.c.
 *
 * Features:
 *   - 1st-order low-pass filtered derivative (default 30 Hz cutoff)
 *   - Anti-windup integrator clamping (±imax)
 *   - Stale-input detection: resets integrator after 200 ms gap
 *   - First-sample derivative suppression
 */
class PID {
public:
    PID(float kp, float ki, float kd, float imax, float fcut_hz = 30.0f);

    float update(float error);
    void  reset();
    void  set_gains(float kp, float ki, float kd);

private:
    static constexpr float STALE_TIMEOUT_US = 200000.0f;

    float    _kp, _ki, _kd;
    float    _imax;
    float    _fcut_hz;
    float    _integrator;
    float    _last_error;
    float    _last_derivative;
    bool     _deriv_valid;
    uint32_t _last_t_us;

    static uint32_t now_us();
};
