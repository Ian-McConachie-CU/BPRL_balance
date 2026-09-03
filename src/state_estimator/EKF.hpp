#pragma once
#include "src/math/math.hpp"

/*
 * EKF — single-lane 16-state Extended Kalman Filter.
 *
 * One instance runs per onboard IMU. StateManager owns three instances,
 * selects the primary lane by innovation health, and dispatches shared
 * IMX5 quaternion updates to all lanes (IMX5 is optional — the filter
 * runs from onboard IMUs alone when the IMX5 is disconnected).
 *
 * State vector (0-indexed):
 *   [0-2]   X, Y, Z           — position, inertial NED (m)
 *   [3-5]   u, v, w           — velocity, body frame (m/s)
 *   [6-9]   q0,q1,q2,q3      — quaternion NED→Body [W,X,Y,Z] Hamilton
 *   [10-12] ba_x, ba_y, ba_z — accelerometer bias (m/s²), random walk
 *   [13-15] bg_x, bg_y, bg_z — gyroscope bias (rad/s), random walk
 *
 * p/q/r, u_dot/v_dot/w_dot, and p_dot/q_dot/r_dot are NOT Kalman states;
 * StateManager computes them from bias-corrected gyros and IMU accel and
 * inserts them when assembling the 19-element StateIdx output vector.
 *
 * Quaternion convention: scalar-first [W, X, Y, Z], Hamilton,
 * representing rotation from NED frame to Body frame.
 * Kinematic equation: dq/dt = 0.5 * q ⊗ {0, p-bg_x, q-bg_y, r-bg_z}
 */
class EKF {
public:
    // Internal Kalman state dimension — 16.
    static constexpr int N = 16;

    EKF();

    // Must be called once before predict()/update_*().
    // imu_index: which g_imu[i] slot this lane reads from (0–2).
    void init(int imu_index);

    // Prediction step — call at 500 Hz.
    // accel: body-frame specific force (m/s²), gyro: body-frame rates (rad/s).
    void predict(float dt, const float accel[3], const float gyro[3]);

    // Measurement updates — called asynchronously when each sensor delivers data.

    // IMX5 quaternion at 200 Hz.  R_var: measurement noise variance per component.
    void update_quaternion(const Quat& q_meas, float R_var);

    // Gravity-vector attitude + accel-bias update — gated on |accel| ≈ g.
    // accel: raw body-frame accelerometer reading (m/s²), NOT bias-corrected.
    void update_gravity(const float accel[3], float R_var);

    // Mocap NED position fusion — active when g_mocap.valid.
    void update_position(const float xyz[3], float R_var);
    void update_ned_vel (const float vel[3], float R_var);

    // Wheel-encoder body-frame forward velocity (u only — wheels give no
    // direct v/w information). u_meas: wheel radius x average angular rate,
    // sign-corrected for mounting (see StateManager). R_var: measurement
    // noise variance.
    // NOTE: superseded by update_leg_wheel_velocity() below whenever leg FK
    // is available — StateManager only falls back to this one when it isn't
    // (see StateManager::update_legs_and_wheels()'s graceful-degradation
    // branch). Kept as its own method rather than folded in so that
    // fallback path stays a trivial, obviously-correct 1-row update.
    void update_wheel_velocity(float u_meas, float R_var);

    // Combined leg+wheel body-frame velocity fusion (u AND w) — see
    // telemetry_plan.md item F for the rigid-body-kinematics derivation.
    // u_meas/w_meas: fused pseudo-measurements combining wheel rolling
    // velocity with the leg's own FK-derived foot-point velocity and the
    // body's own pitch rate (StateManager::update_legs_and_wheels()
    // computes these). Ru/Rw: measurement noise variances — looser than
    // update_wheel_velocity's since this stacks FK/Jacobian noise on top of
    // the wheel encoder alone.
    void update_leg_wheel_velocity(float u_meas, float w_meas, float Ru, float Rw);

    // Accessors
    const float* state()          const { return _x; }
    float        innovation_norm() const { return _innov_norm; }
    bool         is_valid()        const { return _initialized; }

private:
    // ── State index aliases ────────────────────────────────────────────────
    enum {
        iX=0,  iY=1,  iZ=2,
        iU=3,  iV=4,  iW=5,
        iQ0=6, iQ1=7, iQ2=8, iQ3=9,
        iBax=10, iBay=11, iBaz=12,   // accelerometer bias
        iBgx=13, iBgy=14, iBgz=15    // gyroscope bias
        // p/q/r, u_dot/v_dot/w_dot, p_dot/q_dot/r_dot — NOT Kalman states, see StateManager.
    };

    // ── Tuning parameters — all magic numbers live here ───────────────────

    // Physical
    static constexpr float GRAVITY       = 9.80665f;  // m/s²

    // Initial error covariance diagonals (P0)
    static constexpr float P0_POS    = 1.0f;    // position (m²)
    static constexpr float P0_VEL    = 0.5f;    // body velocity (m/s)²
    static constexpr float P0_QUAT   = 0.1f;    // quaternion (unitless)
    static constexpr float P0_BIAS_A = 0.01f;   // accel bias (m/s²)² — tighter init after cal
    static constexpr float P0_BIAS_G = 1e-4f;   // gyro bias (rad/s)²  — ~0.01 rad/s 1σ init

    // Process noise (Q) per EKF step at 500 Hz
    static constexpr float Q_POS    = 1e-4f;   // position random walk
    static constexpr float Q_VEL    = 1e-3f;   // body velocity random walk
    static constexpr float Q_QUAT   = 1e-5f;   // quaternion random walk
    // Bias Q matched to ArduPilot EKF3 GBIAS_P_NSE=1e-3 formula:
    //   dAngBiasVar = sq(sq(dt) * 1e-3) / dt² → ~4e-12 (rad/s)²/step at 500 Hz.
    // We use 1e-9 (250× more permissive) because we have a direct gravity measurement
    // rather than ArduPilot's indirect zero-velocity approach; slightly more latitude
    // lets the filter track temperature-driven drift across the operating envelope.
    static constexpr float Q_BIAS_G = 1e-9f;   // gyro bias random walk (rad/s)²/step
    // Accel bias: ArduPilot ABIAS_P_NSE=2e-2 → sq(sq(dt)*2e-2)/dt² ≈ 6e-10 at 500 Hz.
    // We use 1e-7 — still conservative, accel bias drifts faster than gyro bias.
    static constexpr float Q_BIAS_A = 1e-7f;   // accel bias random walk (m/s²)²/step

    // ── Gravity-vector measurement update parameters ───────────────────────
    // Hard reject: skip update if |a| > this multiple of g (clearly bad sample)
    static constexpr float GRAV_HARD_GATE = 3.0f * GRAVITY;  // 3g outer limit

    // Chi-squared innovation gate (per-axis normalised, summed over 3 axes).
    // Mirrors ArduPilot's velTestRatio < 1 pattern:
    //   test_ratio = sum(innov²) / (sum(S_ii) * GRAV_CHI2_GATE²) < 1
    // GRAV_CHI2_GATE = 5.0 → 5σ joint gate (very permissive under vibration)
    static constexpr float GRAV_CHI2_GATE = 5.0f;

    // Adaptive measurement noise: R_eff = R_base + GRAV_R_VIBE * vibe_rms²
    // where vibe_rms² is the lowpass-filtered squared deviation from g.
    // Mirrors ArduPilot's sq(gpsNEVelVarAccScale * accNavMag) additive term.
    static constexpr float GRAV_R_VIBE    = 0.25f;  // dimensionless scale

    // Vibration filter: α for vibe_rms² IIR filter — τ ≈ 0.1 s at 500 Hz
    static constexpr float GRAV_VIBE_ALPHA = 0.02f;

    // ── Covariance variance floors and ceilings (ConstrainVariances) ──────
    // Gyro bias: floor prevents collapse, ceiling = 0.5 rad/s 1σ
    static constexpr float P_MIN_BIAS_G = 1e-12f;
    static constexpr float P_MAX_BIAS_G = 0.25f;
    // Accel bias: ceiling = 1.0 m/s² 1σ
    static constexpr float P_MIN_BIAS_A = 1e-12f;
    static constexpr float P_MAX_BIAS_A = 1.0f;
    // Quaternion: small floor prevents complete collapse
    static constexpr float P_MIN_QUAT   = 1e-12f;

    // Innovation norm exponential smoothing — higher = slower response to changes
    // τ ≈ 1/(1 - INNOV_SMOOTH) updates  (0.9 → ~10-update time constant)
    static constexpr float INNOV_SMOOTH = 0.9f;

    // ── Filter state ──────────────────────────────────────────────────────
    int   _imu_idx;
    bool  _initialized;
    float _innov_norm;    // exponentially-smoothed innovation magnitude
    float _vibe_filt;     // IIR estimate of (|a| - g)² — drives adaptive R_gravity

    float _x[N];          // state vector
    float _P[N][N];       // error covariance
    float _Q[N][N];       // process noise covariance (diagonal, set in init)

    // ── Scratch matrices (members to keep them off the thread stack) ───────
    float _F[N][N];
    float _wA[N][N];
    float _wB[N][N];

    // ── Internal helpers ───────────────────────────────────────────────────
    void _normalize_quat();
    Quat _get_quat() const;
    void _build_F(float dt, const float gyro[3]);

    // Generic EKF measurement update for m ≤ 6 scalar measurements.
    // H[m][N]: observation matrix, R_diag[m]: per-channel noise variances,
    // innov[m]: pre-computed innovations (z - h(x)).
    void _update(int m, const float H[][N], const float R_diag[], const float innov[]);

};
