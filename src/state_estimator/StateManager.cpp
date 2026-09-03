#include "StateManager.hpp"
#include "src/kinematics/LegParams.hpp"
#include "src/math/math.hpp"
#include <cfloat>

static constexpr float GRAVITY = 9.80665f;

// State index aliases mirroring EKF's internal enum (N=16)
static constexpr int iX=0,  iY=1,  iZ=2;
static constexpr int iU=3,  iV=4,  iW=5;
static constexpr int iQ0=6, iQ1=7, iQ2=8, iQ3=9;
static constexpr int iBax=10, iBay=11, iBaz=12;
static constexpr int iBgx=13, iBgy=14, iBgz=15;

/* ── Leg geometry / hip wiring — see telemetry_plan.md item F ────────────
 * LEG_PARAMS (l1-l5) now lives in src/kinematics/LegParams.hpp -- shared
 * with HipLock's dynamic-target leg-height hold and the StandUp/Lqr
 * balance controllers, single source of truth instead of a file-local
 * duplicate. Keep that header in sync with
 * MatLab_controls/wheeled_biped.m's params() if the linkage is ever
 * remeasured.
 * Hip-to-leg pairing: main.cpp's motor map (ID1=Hip FL, ID2=Hip FR,
 * ID3=Hip RL, ID4=Hip RR) directly gives left=FL/RL, right=FR/RR; phi1 is
 * the REAR thigh (drives AB), phi4 is the FRONT thigh (drives ED) in
 * FiveBarIK's convention — high confidence from naming, not yet physically
 * confirmed (see telemetry_plan.md's Open Items — a phi1/phi4 mixup
 * specifically mirrors thL's sign since l1==l4 and l2==l3 numerically).
 * phi1/phi4 are ALSO opposite-sense conventions (phi1 CW-positive, phi4
 * CCW-positive, confirmed 2026-09-02 — real front/rear hip actuators are
 * mirror-mounted) — see FiveBarIK.hpp's header comment. hips[phi4_id].pos_rad
 * below is fed straight into fk()/ik() as-is; CANMotor.cpp's HIP_SIGN only
 * corrects LEFT/RIGHT mirroring, not this front/rear asymmetry, so this
 * relies on the front hip's raw robot-frame encoder angle already reading
 * CCW-positive after that correction — bench-verify alongside the pairing
 * above before trusting FK-derived leg state for anything control-facing. */
struct LegHipMap { uint8_t phi1_id, phi4_id; };   // 1-indexed CAN ids
static constexpr LegHipMap LEG_HIP_MAP[2] = {
    /* left  */ { /*phi1=*/3, /*phi4=*/1 },   // RL, FL
    /* right */ { /*phi1=*/4, /*phi4=*/2 },   // RR, FR
};
// Hip encoder zero-offsets are applied centrally in CANMotor.cpp's
// rmd_rx_cb() (HIP_OFFSET_RAD) -- hips[i].pos_rad below is ALREADY the
// robot-frame angle, no offset needed here. See CANMotor.hpp's header
// comment for the calibration workflow.

/* ══════════════════════════════════════════════════════════════════════════
 * Construction / initialisation
 * ══════════════════════════════════════════════════════════════════════════ */

StateManager::StateManager()
    : _primary(0), _initialized(false),
      _blended_p(0.0f), _blended_q(0.0f), _blended_r(0.0f),
      _blended_ud(0.0f), _blended_vd(0.0f), _blended_wd(0.0f),
      _prev_p(0.0f), _prev_q(0.0f), _prev_r(0.0f),
      _pdot_filt(0.0f), _qdot_filt(0.0f), _rdot_filt(0.0f),
      _ud_filt(0.0f), _vd_filt(0.0f), _wd_filt(0.0f),
      _lane_p{}, _lane_q{}, _lane_r{},
      _leg{},
      _leg_L_avg(0.0f), _leg_L_dot_avg(0.0f), _leg_theta_avg(0.0f), _leg_theta_dot_avg(0.0f),
      _leg_avg_valid(false)
{}

void StateManager::init()
{
    for (int i = 0; i < NUM_LANES; ++i)
        _lanes[i].init(i);

    _primary      = 0;
    _blended_p    = _blended_q    = _blended_r    = 0.0f;
    _blended_ud   = _blended_vd   = _blended_wd   = 0.0f;
    _prev_p       = _prev_q       = _prev_r       = 0.0f;
    _pdot_filt    = _qdot_filt    = _rdot_filt    = 0.0f;
    _ud_filt      = _vd_filt      = _wd_filt      = 0.0f;

    for (int i = 0; i < NUM_LANES; ++i)
        _lane_p[i] = _lane_q[i] = _lane_r[i] = 0.0f;

    _leg[0] = LegState{};
    _leg[1] = LegState{};
    _leg_L_avg = _leg_L_dot_avg = _leg_theta_avg = _leg_theta_dot_avg = 0.0f;
    _leg_avg_valid = false;

    _initialized = true;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Main update — called at 500 Hz from StateEstThread
 * ══════════════════════════════════════════════════════════════════════════ */

void StateManager::update(float dt, const IMURaw imu[3], const CANIMURaw& can_imu,
                           const MocapRaw& mocap)
{
    if (!_initialized) return;

    // ── 1. Predict: each lane uses its own IMU ─────────────────────────────
    for (int i = 0; i < NUM_LANES; ++i) {
        if (imu[i].valid)
            _lanes[i].predict(dt, imu[i].accel, imu[i].gyro);
    }

    // ── 1.5. Gravity-vector attitude + accel-bias update (gated on |a| ≈ g) ─
    for (int i = 0; i < NUM_LANES; ++i) {
        if (imu[i].valid)
            _lanes[i].update_gravity(imu[i].accel, R_GRAVITY);
    }

    // ── 2. IMX5 quaternion update on all lanes (200 Hz, asynchronous) ─────
    // IMX5 quaternion is body→NED (verified: Euler angles match physical attitude).
    if (can_imu.valid && can_imu.has_new_quat) {
        Quat q_meas = { can_imu.q0, can_imu.q1, can_imu.q2, can_imu.q3 };
        for (int i = 0; i < NUM_LANES; ++i)
            _lanes[i].update_quaternion(q_meas, R_QUAT);
    }

    // ── 3. Select primary lane ────────────────────────────────────────────
    _primary = _select_primary();

    // ── 4. Compute innovation-norm weights ────────────────────────────────
    float w[NUM_LANES] = {}, w_sum = 0.0f;
    for (int i = 0; i < NUM_LANES; ++i) {
        if (!_lanes[i].is_valid()) continue;
        w[i] = 1.0f / (1e-4f + _lanes[i].innovation_norm());
        w_sum += w[i];
    }
    if (w_sum > 1e-10f) {
        for (int i = 0; i < NUM_LANES; ++i) w[i] /= w_sum;
    } else {
        w[_primary] = 1.0f;
    }

    // ── 5. Mocap position + velocity fusion (all lanes, when connected) ───
    if (mocap.valid && mocap.has_new) {
        const float xyz[3] = { mocap.x,  mocap.y,  mocap.z  };
        const float vel[3] = { mocap.vx, mocap.vy, mocap.vz };
        for (int i = 0; i < NUM_LANES; ++i) {
            if (!_lanes[i].is_valid()) continue;
            _lanes[i].update_position(xyz, R_MOCAP_POS);
            _lanes[i].update_ned_vel(vel, R_MOCAP_VEL);
        }
    }

    // ── 6. Soft-blend p/q/r: bias-corrected gyros + IMX5 blend ──
    _blended_p = _blended_q = _blended_r = 0.0f;
    for (int i = 0; i < NUM_LANES; ++i) {
        if (!imu[i].valid || !_lanes[i].is_valid()) {
            _lane_p[i] = _lane_q[i] = _lane_r[i] = 0.0f;
            continue;
        }
        const float* st = _lanes[i].state();
        const float lp = imu[i].gyro[0] - st[iBgx];
        const float lq = imu[i].gyro[1] - st[iBgy];
        const float lr = imu[i].gyro[2] - st[iBgz];
        _lane_p[i] = lp;
        _lane_q[i] = lq;
        _lane_r[i] = lr;
        if (w[i] < 1e-15f) continue;
        _blended_p += w[i] * lp;
        _blended_q += w[i] * lq;
        _blended_r += w[i] * lr;
    }
    if (can_imu.valid) {
        // IMX5 rates are in NED z-down world frame. Rotate to body frame: v_body = R_b2n^T * v_NED
        float R_imx[3][3];
        {
            const float* st = _lanes[_primary].state();
            Quat q = { st[iQ0], st[iQ1], st[iQ2], st[iQ3] };
            quat_to_rot_body2ned(q, R_imx);
        }
        // NED z-down: v_body = R_b2n^T * v_NED (standard transpose multiply)
        const float p_b = R_imx[0][0]*can_imu.p + R_imx[1][0]*can_imu.q + R_imx[2][0]*can_imu.r;
        const float q_b = R_imx[0][1]*can_imu.p + R_imx[1][1]*can_imu.q + R_imx[2][1]*can_imu.r;
        const float r_b = R_imx[0][2]*can_imu.p + R_imx[1][2]*can_imu.q + R_imx[2][2]*can_imu.r;
        _blended_p = (1.0f-STATEMGR_IMX5_RATE_WEIGHT)*_blended_p + STATEMGR_IMX5_RATE_WEIGHT*p_b;
        _blended_q = (1.0f-STATEMGR_IMX5_RATE_WEIGHT)*_blended_q + STATEMGR_IMX5_RATE_WEIGHT*q_b;
        _blended_r = (1.0f-STATEMGR_IMX5_RATE_WEIGHT)*_blended_r + STATEMGR_IMX5_RATE_WEIGHT*r_b;
    }

    // ── 7. Blend uvw_dot: gravity+Coriolis-corrected IMU accel per lane ───
    _blended_ud = _blended_vd = _blended_wd = 0.0f;
    for (int i = 0; i < NUM_LANES; ++i) {
        if (w[i] < 1e-15f || !imu[i].valid) continue;
        const float* st = _lanes[i].state();

        Quat q = { st[iQ0], st[iQ1], st[iQ2], st[iQ3] };
        float R[3][3];
        quat_to_rot_body2ned(q, R);

        // Gravity in body frame: g_body = R_b2n^T * [0,0,g] = R_b2n row 2 * g
        const float g_body[3] = { R[2][0]*GRAVITY, R[2][1]*GRAVITY, R[2][2]*GRAVITY };

        // Per-lane bias-corrected gyro rates for Coriolis
        const float pqr_l[3] = {
            imu[i].gyro[0] - st[iBgx],
            imu[i].gyro[1] - st[iBgy],
            imu[i].gyro[2] - st[iBgz]
        };
        const float vel_l[3] = { st[iU], st[iV], st[iW] };
        float oxv[3];
        cross3(pqr_l, vel_l, oxv);

        // NED z-down: a_true = accel + g_body - ω×v (sensor reads -g at hover)
        _blended_ud += w[i] * (imu[i].accel[0] - st[iBax] + g_body[0] - oxv[0]);
        _blended_vd += w[i] * (imu[i].accel[1] - st[iBay] + g_body[1] - oxv[1]);
        _blended_wd += w[i] * (imu[i].accel[2] - st[iBaz] + g_body[2] - oxv[2]);
    }

    // Lowpass filter uvw_dot (recompute alpha in case dt varies)
    const float alpha_uvw = lowpass_alpha(STATEMGR_LP_UVWDOT_HZ, dt);
    _ud_filt = lowpass(_blended_ud, _ud_filt, alpha_uvw);
    _vd_filt = lowpass(_blended_vd, _vd_filt, alpha_uvw);
    _wd_filt = lowpass(_blended_wd, _wd_filt, alpha_uvw);

    // ── 8. Angular acceleration: differentiate blended rates + lowpass ─────
    const float alpha_pqr = lowpass_alpha(STATEMGR_LP_PQRDOT_HZ, dt);
    _pdot_filt = lowpass(derivative(_blended_p, _prev_p, dt), _pdot_filt, alpha_pqr);
    _qdot_filt = lowpass(derivative(_blended_q, _prev_q, dt), _qdot_filt, alpha_pqr);
    _rdot_filt = lowpass(derivative(_blended_r, _prev_r, dt), _rdot_filt, alpha_pqr);

    _prev_p = _blended_p;
    _prev_q = _blended_q;
    _prev_r = _blended_r;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Leg FK + combined leg/wheel velocity fusion — call once per tick, right
 * after update() (see telemetry_plan.md item F for the full derivation).
 * ══════════════════════════════════════════════════════════════════════════ */

void StateManager::update_legs_and_wheels(const CanMotorState hips[4], const CanMotorState wheels[2])
{
    bool  leg_ok[2] = { false, false };
    float leg_x[2] = {}, leg_z[2] = {}, leg_xdot[2] = {}, leg_zdot[2] = {};

    float L_sum = 0.0f, Ldot_sum = 0.0f, theta_sum = 0.0f, thetadot_sum = 0.0f;
    int   leg_n = 0;

    const float phi = pitch();   // current body pitch — update() has already run this tick

    for (int leg = 0; leg < 2; ++leg) {
        const LegHipMap& m = LEG_HIP_MAP[leg];
        const CanMotorState& h1 = hips[m.phi1_id - 1];
        const CanMotorState& h4 = hips[m.phi4_id - 1];

        if (!h1.valid || !h4.valid) { _leg[leg] = LegState{}; continue; }

        const float phi1 = h1.pos_rad;
        const float phi4 = h4.pos_rad;

        FiveBarPose pose;
        if (!fk(LEG_PARAMS, phi1, phi4, pose)) { _leg[leg] = LegState{}; continue; }

        const FiveBarJac J    = jac(LEG_PARAMS, phi1, phi4);
        const FiveBarVel vel  = jac_to_vel(J, h1.vel_rads, h4.vel_rads);
        const FootPointNed ft = foot_point_ned(pose, vel);

        _leg[leg].L0      = pose.L0;
        _leg[leg].L0_dot  = vel.L0_dot;
        _leg[leg].thL     = pose.thL;
        _leg[leg].thL_dot = vel.thL_dot;
        _leg[leg].x_dot   = ft.x_dot;
        _leg[leg].z_dot   = ft.z_dot;
        _leg[leg].valid   = true;

        leg_x[leg] = ft.x; leg_z[leg] = ft.z;
        leg_xdot[leg] = ft.x_dot; leg_zdot[leg] = ft.z_dot;
        leg_ok[leg] = true;

        L_sum        += pose.L0;
        Ldot_sum     += vel.L0_dot;
        theta_sum    += (phi - pose.thL);           // theta = phi - thL, see FiveBarIK.hpp
        thetadot_sum += (_blended_q - vel.thL_dot);  // phi_dot ~= Q for this application, per controls_plan.md §1
        leg_n++;
    }

    if (leg_n > 0) {
        _leg_L_avg         = L_sum / (float)leg_n;
        _leg_L_dot_avg     = Ldot_sum / (float)leg_n;
        _leg_theta_avg     = theta_sum / (float)leg_n;
        _leg_theta_dot_avg = thetadot_sum / (float)leg_n;
        _leg_avg_valid     = true;
    } else {
        _leg_avg_valid = false;
    }

    // Combined leg+wheel body-velocity fusion (rigid-body kinematics of the
    // wheel-axle/foot point, ground-contact + no-slip assumed):
    //   U_meas = u_roll - Q*z_C - x_C_dot
    //   W_meas = Q*x_C - z_C_dot
    // See telemetry_plan.md item F for the derivation.
    const float Q = _blended_q;

    float u_sum = 0.0f, w_sum = 0.0f;
    int   n     = 0;
    for (int leg = 0; leg < 2; ++leg) {
        if (!leg_ok[leg] || !wheels[leg].valid) continue;
        const float sign   = (leg == 0) ? STATEMGR_WHEEL_L_SIGN : STATEMGR_WHEEL_R_SIGN;
        const float u_roll = sign * wheels[leg].vel_rads * STATEMGR_WHEEL_RADIUS_M;
        u_sum += u_roll - Q*leg_z[leg] - leg_xdot[leg];
        w_sum +=          Q*leg_x[leg] - leg_zdot[leg];
        n++;
    }

    if (n > 0) {
        const float u_meas = u_sum / (float)n;
        const float w_meas = w_sum / (float)n;
        for (int i = 0; i < NUM_LANES; ++i) {
            if (!_lanes[i].is_valid()) continue;
            _lanes[i].update_leg_wheel_velocity(u_meas, w_meas, R_LEG_WHEEL_U, R_LEG_WHEEL_W);
        }
    } else if (wheels[0].valid || wheels[1].valid) {
        // Graceful degradation: no leg's FK is valid but a wheel is --
        // fall back to wheel-only U fusion (same math the old step-2.5
        // fusion had) rather than lose velocity fusion entirely.
        float u_sum2 = 0.0f;
        int   n2     = 0;
        if (wheels[0].valid) { u_sum2 += STATEMGR_WHEEL_L_SIGN * wheels[0].vel_rads * STATEMGR_WHEEL_RADIUS_M; n2++; }
        if (wheels[1].valid) { u_sum2 += STATEMGR_WHEEL_R_SIGN * wheels[1].vel_rads * STATEMGR_WHEEL_RADIUS_M; n2++; }
        const float u_wheel = u_sum2 / (float)n2;
        for (int i = 0; i < NUM_LANES; ++i) {
            if (!_lanes[i].is_valid()) continue;
            _lanes[i].update_wheel_velocity(u_wheel, R_WHEEL_VEL);
        }
    }
}

void StateManager::get_leg_state(int leg, LegState& out) const
{
    if (leg < 0 || leg > 1) { out = LegState{}; return; }
    out = _leg[leg];
}

/* ══════════════════════════════════════════════════════════════════════════
 * Lane selection
 * ══════════════════════════════════════════════════════════════════════════ */

int StateManager::_select_primary() const
{
    int best = _primary;
    float best_score = FLT_MAX;

    for (int i = 0; i < NUM_LANES; ++i) {
        if (!_lanes[i].is_valid()) continue;
        float score = _lanes[i].innovation_norm();
        if (score < best_score) {
            best_score = score;
            best = i;
        }
    }
    return best;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Full StateIdx::N-element state output
 * Maps 16-state EKF lanes onto the StateIdx ordering and inserts derived states.
 * ══════════════════════════════════════════════════════════════════════════ */

void StateManager::get_state(float out[StateIdx::N]) const
{
    static_assert(StateIdx::N == 23, "state size mismatch");

    const float* ekf = _lanes[_primary].state();  // EKF::N = 16 elements

    // Position (NED) — EKF[0–2] → out[0–2]
    out[StateIdx::X]     = ekf[iX];
    out[StateIdx::Y]     = ekf[iY];
    out[StateIdx::Z_POS] = ekf[iZ];

    // Body velocity — EKF[3–5] → out[3–5]
    out[StateIdx::U] = ekf[iU];
    out[StateIdx::V] = ekf[iV];
    out[StateIdx::W] = ekf[iW];

    // Body acceleration (blended + lowpass filtered) → out[6–8]
    out[StateIdx::U_DOT] = _ud_filt;
    out[StateIdx::V_DOT] = _vd_filt;
    out[StateIdx::W_DOT] = _wd_filt;

    // Quaternion Body→NED — EKF[6–9] → out[9–12]
    out[StateIdx::Q0] = ekf[iQ0];
    out[StateIdx::Q1] = ekf[iQ1];
    out[StateIdx::Q2] = ekf[iQ2];
    out[StateIdx::Q3] = ekf[iQ3];

    // Angular rates (soft-blended, NED z-down body frame) → out[13–15]
    out[StateIdx::P] = _blended_p;
    out[StateIdx::Q] = _blended_q;
    out[StateIdx::R] = _blended_r;

    // Angular acceleration (differentiated + lowpass filtered) → out[16–18]
    out[StateIdx::P_DOT] = _pdot_filt;
    out[StateIdx::Q_DOT] = _qdot_filt;
    out[StateIdx::R_DOT] = _rdot_filt;

    // Leg state (both legs averaged, from update_legs_and_wheels()) → out[19-22].
    // Zeroed (not left stale) when no leg's FK is currently valid.
    out[StateIdx::LEG_L]         = _leg_avg_valid ? _leg_L_avg         : 0.0f;
    out[StateIdx::LEG_L_DOT]     = _leg_avg_valid ? _leg_L_dot_avg     : 0.0f;
    out[StateIdx::LEG_PITCH]     = _leg_avg_valid ? _leg_theta_avg     : 0.0f;
    out[StateIdx::LEG_PITCH_DOT] = _leg_avg_valid ? _leg_theta_dot_avg : 0.0f;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Individual state accessors
 * ══════════════════════════════════════════════════════════════════════════ */

float StateManager::roll() const
{
    const float* st = _lanes[_primary].state();
    Quat q = { st[iQ0], st[iQ1], st[iQ2], st[iQ3] };
    float ro, pi, ya;
    quat_to_euler(q, ro, pi, ya);
    return ro;
}

float StateManager::pitch() const
{
    const float* st = _lanes[_primary].state();
    Quat q = { st[iQ0], st[iQ1], st[iQ2], st[iQ3] };
    float ro, pi, ya;
    quat_to_euler(q, ro, pi, ya);
    return pi;
}

float StateManager::yaw() const
{
    const float* st = _lanes[_primary].state();
    Quat q = { st[iQ0], st[iQ1], st[iQ2], st[iQ3] };
    float ro, pi, ya;
    quat_to_euler(q, ro, pi, ya);
    return ya;
}

/* ══════════════════════════════════════════════════════════════════════════
 * Per-lane accessors (called by StateEstThread only)
 * ══════════════════════════════════════════════════════════════════════════ */

void StateManager::get_lane_euler(int lane, float& roll, float& pitch, float& yaw) const
{
    if (lane < 0 || lane >= NUM_LANES || !_lanes[lane].is_valid()) {
        roll = pitch = yaw = 0.0f;
        return;
    }
    const float* st = _lanes[lane].state();
    Quat q = { st[iQ0], st[iQ1], st[iQ2], st[iQ3] };
    float ro, pi, ya;
    quat_to_euler(q, ro, pi, ya);
    roll  = ro;
    pitch = pi;
    yaw   = ya;
}

void StateManager::get_lane_pqr(int lane, float& p, float& q, float& r) const
{
    if (lane < 0 || lane >= NUM_LANES) { p = q = r = 0.0f; return; }
    p = _lane_p[lane];
    q = _lane_q[lane];
    r = _lane_r[lane];
}

