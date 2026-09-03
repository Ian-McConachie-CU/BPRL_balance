#pragma once
#include "EKF.hpp"
#include "src/RobotState.hpp"
#include "src/threads.hpp"        // IMURaw, CANIMURaw, MocapRaw
#include "src/coms/CANMotor.hpp"  // CanMotorState (wheel-encoder velocity fusion)
#include "src/kinematics/FiveBarIK.hpp"

// Lowpass cutoff frequencies for derived derivative states (Hz)
#define STATEMGR_LP_UVWDOT_HZ     50.0f   // cutoff for u_dot/v_dot/w_dot
#define STATEMGR_LP_PQRDOT_HZ     50.0f   // cutoff for p_dot/q_dot/r_dot
// IMX5 angular rate blend weight: 0=pure onboard gyros, 1=pure IMX5
#define STATEMGR_IMX5_RATE_WEIGHT  0.3f

// Wheel-encoder velocity fusion (see StateManager::update, step 2.5) --
// tune these once real wheels/encoders are on the bench.
#define STATEMGR_WHEEL_RADIUS_M   0.092f  // must match wheeled_biped.m's p.R
#define STATEMGR_WHEEL_L_SIGN     (+1.0f) // flip to -1 if wheel L's encoder
#define STATEMGR_WHEEL_R_SIGN     (+1.0f) // reads backwards vs. wheel R's

// Per-leg state from the most recent update_legs_and_wheels() call.
// Exposed for RobotTelemetry (per-leg breakdown) -- see telemetry_plan.md
// items F/G. NOT the same as StateIdx::LEG_* (those are both-legs-averaged
// Kalman-adjacent outputs written into g_state by get_state()).
struct LegState {
    float L0, L0_dot;       // m, m/s
    float thL, thL_dot;     // rad, rad/s -- hip-relative (see FiveBarIK.hpp)
    float x_dot, z_dot;     // m/s, foot-point velocity rel. to body, NED convention
    bool  valid;
};

/*
 * StateManager — multi-lane EKF orchestrator.
 *
 * Runs three EKF lanes (N=13 states each), one per onboard IMU. Each lane
 * receives its own IMU's predict step; all lanes share the same IMX5
 * measurement updates.
 *
 * Quaternion output: hard-selected from the primary lane (lowest smoothed
 * innovation norm among valid lanes) — no blending to avoid antipodal issues.
 *
 * p/q/r output: soft-blended across all valid lanes weighted by
 * 1/innovation_norm, giving partial noise averaging with fault isolation.
 *
 * u_dot/v_dot/w_dot: gravity+Coriolis-corrected IMU accel, blended by the
 * same innovation-norm weights, then lowpass filtered at STATEMGR_LP_UVWDOT_HZ.
 *
 * p_dot/q_dot/r_dot: differentiated from blended p/q/r, lowpass filtered at
 * STATEMGR_LP_PQRDOT_HZ.
 *
 * Assembles the full 19-element StateIdx state vector for g_state[].
 */
class StateManager {
public:
    static constexpr int NUM_LANES = 3;

    // IMX5 / mocap / gravity measurement noise variances — tunable.
    static constexpr float R_QUAT      = 1e-3f;   // IMX5 quaternion component variance
    static constexpr float R_GRAVITY   = 0.5f;    // accel gravity-vector variance (m/s²)²
    static constexpr float R_MOCAP_POS = 1e-3f;   // mocap NED position variance (m²)
    static constexpr float R_MOCAP_VEL = 1e-2f;   // mocap NED velocity variance (m/s)²
    // Wheel-encoder forward-velocity variance — deliberately tight relative
    // to R_MOCAP_VEL/R_GRAVITY: wheel odometry is trusted as the PRIMARY
    // velocity estimate (assumes the wheel is in contact with the ground,
    // i.e. no slip detection), with IMU-only prediction (EKF::predict's
    // accel integration) filling in between updates / correcting for any
    // brief slip. See StateManager::update() step 2.5.
    static constexpr float R_WHEEL_VEL = 2e-3f;

    // Leg+wheel combined velocity fusion — looser than R_WHEEL_VEL since
    // this stacks FK/Jacobian noise on top of the wheel encoder alone.
    // R_LEG_WHEEL_W is deliberately loose: it's a soft pseudo-measurement
    // (flat-ground-contact assumption), not direct sensing like the wheel
    // encoder's rolling-velocity contribution to R_LEG_WHEEL_U. Placeholders
    // — retune on the bench, same status as R_WHEEL_VEL originally was.
    static constexpr float R_LEG_WHEEL_U = 5e-3f;
    static constexpr float R_LEG_WHEEL_W = 2e-2f;

    StateManager();

    void init();

    // Call once per StateEstThread tick (500 Hz).
    // dt: loop period in seconds.
    // imu: snapshot of g_imu[3] (taken under imu_mtx before this call).
    // can_imu: snapshot of g_can_imu (taken under can_imu_mtx before this call).
    // mocap: snapshot of g_mocap (taken under mocap_mtx before this call).
    void update(float dt, const IMURaw imu[3], const CANIMURaw& can_imu,
                const MocapRaw& mocap);

    // Call once per StateEstThread tick, right after update() — computes
    // per-leg FK from the 4 hip encoders, fuses the combined leg+wheel
    // body-velocity pseudo-measurement into U and W on every valid lane
    // (falling back to wheel-only U fusion if no leg is valid), and
    // updates the averaged StateIdx::LEG_* values get_state() will write.
    // hips: CanMotorState for ids 1-4 (FL/FR/RL/RR). wheels: ids 5,6 (L/R).
    // Both via can_motor_get_state() — internally mutex-protected, no
    // separate lock needed by the caller. See telemetry_plan.md item F.
    void update_legs_and_wheels(const CanMotorState hips[4], const CanMotorState wheels[2]);

    // Full StateIdx::N-element state output — maps the primary lane's EKF
    // state onto StateIdx ordering, fills in the 6 derived quantities
    // (uvw_dot, pqr_dot), and the 4 averaged leg states (19-22).
    void get_state(float out[StateIdx::N]) const;

    // Derived Euler angles (from primary lane quaternion).
    float roll()    const;
    float pitch()   const;
    float yaw()     const;

    // Per-lane accessors — called by StateEstThread only (no mutex needed).
    void get_lane_euler(int lane, float& roll, float& pitch, float& yaw) const;
    void get_lane_pqr  (int lane, float& p,    float& q,    float& r)    const;
    int  primary_lane  () const { return _primary; }

    // Per-leg (0=left, 1=right) FK snapshot from the most recent
    // update_legs_and_wheels() call — for RobotTelemetry's per-leg
    // breakdown (StateIdx::LEG_* is both-legs-averaged instead).
    void get_leg_state(int leg, LegState& out) const;

private:
    EKF  _lanes[NUM_LANES];
    int  _primary;
    bool _initialized;

    // Soft-blended angular rates (weighted by 1/innovation_norm across valid lanes)
    float _blended_p, _blended_q, _blended_r;

    // Body acceleration: blended gravity+Coriolis-corrected IMU accel
    float _blended_ud, _blended_vd, _blended_wd;

    // Angular acceleration via differentiation of blended rates + lowpass
    float _prev_p,    _prev_q,    _prev_r;
    float _pdot_filt, _qdot_filt, _rdot_filt;

    // Lowpass-filtered uvw_dot output
    float _ud_filt, _vd_filt, _wd_filt;

    // Per-lane bias-corrected angular rates (updated each update() call)
    float _lane_p[NUM_LANES], _lane_q[NUM_LANES], _lane_r[NUM_LANES];

    // Per-leg (0=left, 1=right) FK/velocity snapshot from
    // update_legs_and_wheels() — see get_leg_state().
    LegState _leg[2];

    // Both-legs-averaged leg state, written into StateIdx::LEG_* by
    // get_state() — theta = phi - thL (NED convention, see FiveBarIK.hpp).
    float _leg_L_avg, _leg_L_dot_avg, _leg_theta_avg, _leg_theta_dot_avg;
    bool  _leg_avg_valid;

    int  _select_primary() const;
};
