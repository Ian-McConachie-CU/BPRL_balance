#include "math.hpp"
#include <cmath>

/* ── Scalar helpers ──────────────────────────────────────────────────────── */

float constrain_float(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ── Signal processing ───────────────────────────────────────────────────── */

float lowpass_alpha(float fc_hz, float dt_s)
{
    // alpha = dt / (dt + 1/(2*pi*fc))
    const float tau = 1.0f / (6.2831853f * fc_hz);
    return dt_s / (dt_s + tau);
}

float lowpass(float input, float prev_out, float alpha)
{
    return alpha * input + (1.0f - alpha) * prev_out;
}

float derivative(float current, float prev, float dt_s)
{
    return (current - prev) / dt_s;
}

float integrate(float value, float dt_s)
{
    return value * dt_s;
}

/* ── 3-vector helpers ────────────────────────────────────────────────────── */

void cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

/* ── Quaternion ──────────────────────────────────────────────────────────── */

Quat quat_mul(const Quat& a, const Quat& b)
{
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

Quat quat_conj(const Quat& q)
{
    return { q.w, -q.x, -q.y, -q.z };
}

Quat quat_norm(const Quat& q)
{
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-10f) return { 1.0f, 0.0f, 0.0f, 0.0f };  // guard near-zero
    float inv = 1.0f / n;
    return { q.w*inv, q.x*inv, q.y*inv, q.z*inv };
}

float quat_dot(const Quat& a, const Quat& b)
{
    return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

void quat_to_euler(const Quat& q, float& roll, float& pitch, float& yaw)
{
    // ZYX Euler from q_NED→Body [W, X, Y, Z] (Hamilton, scalar-first).
    // roll  = atan2(2(WX + YZ), 1 - 2(X² + Y²))
    // pitch = asin( 2(WY - ZX) )
    // yaw   = atan2(2(WZ + XY), 1 - 2(Y² + Z²))
    const float w = q.w, x = q.x, y = q.y, z = q.z;

    roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));

    float sp = 2.0f*(w*y - z*x);
    sp = constrain_float(sp, -1.0f, 1.0f);
    pitch = asinf(sp);

    yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
}

void quat_to_rot_body2ned(const Quat& q, float R[3][3])
{
    // For q_NED→Body, the rotation matrix that maps body vectors to NED is
    // R_body2ned = R(q)^T = R(q*).
    // Using q* = {w, -x, -y, -z}, the DCM entries are:
    const float w = q.w, x = q.x, y = q.y, z = q.z;

    R[0][0] = 1.0f - 2.0f*(y*y + z*z);
    R[0][1] = 2.0f*(x*y - w*z);
    R[0][2] = 2.0f*(x*z + w*y);

    R[1][0] = 2.0f*(x*y + w*z);
    R[1][1] = 1.0f - 2.0f*(x*x + z*z);
    R[1][2] = 2.0f*(y*z - w*x);

    R[2][0] = 2.0f*(x*z - w*y);
    R[2][1] = 2.0f*(y*z + w*x);
    R[2][2] = 1.0f - 2.0f*(x*x + y*y);
}
