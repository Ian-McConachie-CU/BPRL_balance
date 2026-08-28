#pragma once
#include <cstdint>

/*
 * BPRL math utilities — scalar helpers, IIR filter, quaternion ops.
 *
 * Quaternion convention throughout: Hamilton scalar-first [W, X, Y, Z],
 * representing the rotation from NED frame to Body frame (q_NED→Body).
 * Kinematic propagation equation:  dq/dt = 0.5 * q ⊗ ω_pure
 * where ω_pure = {0, p, q, r} (body-frame angular velocity).
 */

/* ── Scalar helpers ──────────────────────────────────────────────────────── */

float constrain_float(float v, float lo, float hi);

/* ── Signal processing ───────────────────────────────────────────────────── */

// Compute IIR lowpass coefficient: alpha = dt / (dt + 1 / (2*pi*fc))
float lowpass_alpha(float fc_hz, float dt_s);

// First-order IIR lowpass: y_k = alpha * x_k + (1-alpha) * y_{k-1}
float lowpass(float input, float prev_out, float alpha);

// Backward-difference numerical derivative
float derivative(float current, float prev, float dt_s);

// Trapezoidal integration (stateless helper — caller owns the accumulator)
float integrate(float value, float dt_s);

/* ── 3-vector helpers ────────────────────────────────────────────────────── */

// out = a × b
void cross3(const float a[3], const float b[3], float out[3]);

/* ── Quaternion ──────────────────────────────────────────────────────────── */

struct Quat {
    float w, x, y, z;   // [W, X, Y, Z] scalar-first
};

// Hamilton product: a ⊗ b
Quat  quat_mul(const Quat& a, const Quat& b);

// Conjugate: q* = {w, -x, -y, -z}
Quat  quat_conj(const Quat& q);

// Enforce unit norm
Quat  quat_norm(const Quat& q);

// Scalar dot product (for antipodal check)
float quat_dot(const Quat& a, const Quat& b);

// Extract Euler angles (roll/pitch/yaw) from q_NED→Body.
// Convention: ZYX (yaw-pitch-roll), angles in radians.
void quat_to_euler(const Quat& q, float& roll, float& pitch, float& yaw);

// Build 3×3 rotation matrix R_body2ned (v_NED = R * v_body).
// For q = q_NED→Body: R_body2ned = R(q)^T  (conjugate rotation).
// R is stored row-major: R[row][col].
void quat_to_rot_body2ned(const Quat& q, float R[3][3]);

/* ── Fixed-size N×N matrix operations ───────────────────────────────────────
 * Template parameter Dim is the matrix dimension (compile-time constant).
 * C must not alias A or B in mat_mul.
 * ─────────────────────────────────────────────────────────────────────────── */

#include <cstring>

template<int Dim> inline void mat_mul(const float A[Dim][Dim], const float B[Dim][Dim], float C[Dim][Dim])
{
    memset(C, 0, Dim * Dim * sizeof(float));
    for (int i = 0; i < Dim; ++i)
        for (int k = 0; k < Dim; ++k) {
            if (A[i][k] == 0.0f) continue;
            for (int j = 0; j < Dim; ++j)
                C[i][j] += A[i][k] * B[k][j];
        }
}

template<int Dim> inline void mat_add(const float A[Dim][Dim], const float B[Dim][Dim], float C[Dim][Dim])
{
    for (int i = 0; i < Dim; ++i)
        for (int j = 0; j < Dim; ++j)
            C[i][j] = A[i][j] + B[i][j];
}

template<int Dim> inline void mat_trans(const float A[Dim][Dim], float AT[Dim][Dim])
{
    for (int i = 0; i < Dim; ++i)
        for (int j = 0; j < Dim; ++j)
            AT[j][i] = A[i][j];
}
