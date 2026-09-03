#pragma once

/*
 * WheelBalanceGainTable.hpp -- GENERATED FILE, do not hand-edit.
 *
 * Produced by MatLab_controls/export_wheel_balance_gains.m from
 * wheeled_biped.m's wb.schedule(p, Q, R, Lgrid, order, 'velwheel', dt) --
 * discrete-time (dt = 1/100 s, matching ControlThread's 100 Hz compute
 * rate) LQR design against the VELOCITY-CONTROLLED-WHEEL reduced model,
 * gain-scheduled against leg length L, one cubic polynomial fit per gain
 * entry over a normalized u = (L - LMID) / LHALF.
 *
 * Rows: [ax (wheel acceleration cmd, m/s^2 -- NOT a torque, see
 * WheelBalanceLQR.hpp), Tp (hip torque, N.m)]. Columns (state order matches
 * wheeled_biped.m's linearModelVel exactly): [theta, thetadot, phi, phidot].
 *
 * Regenerate by running export_wheel_balance_gains.m in MATLAB whenever
 * params()/Q/R change -- see controls_plan.md section 5.
 *
 * Q = diag([40 2 800 8]),  R = diag([1 3]),  Lgrid = linspace(0.090, 0.220, 25),  order = 3
 */
namespace WheelBalanceGainTable {
    constexpr int   NU    = 2;
    constexpr int   NX    = 4;
    constexpr int   ORDER = 3;   // cubic -- ORDER+1 = 4 coefficients per entry, highest degree first (MATLAB polyval order)
    constexpr float LMID  = 0.155000000f;
    constexpr float LHALF = 0.065000000f;

    // C[row][col][coeff], coeff 0 = highest degree (matches MATLAB polyval convention)
    constexpr float C[NU][NX][ORDER + 1] = {
        {
            { 8.502192485e-02f, -4.310499173e-01f, 2.209975412e+00f, 1.291434993e+01f },
            { 3.738295724e-03f, -2.433993598e-02f, 4.716317516e-01f, 2.257063503e+00f },
            { -2.706358581e-01f, 7.837599404e-01f, -1.964976246e+00f, 3.725633086e+00f },
            { -2.867492574e-02f, 8.544545275e-02f, -2.315406793e-01f, 7.699461515e-01f },
        },
        {
            { 9.772806254e-03f, -8.362981857e-02f, 5.709972871e-01f, 3.442110677e+00f },
            { -4.702715616e-03f, 1.352124785e-02f, -3.386403959e-02f, 5.854680631e-02f },
            { -4.754249418e-02f, 1.219543660e-01f, -3.540668357e-01f, -1.910812115e+01f },
            { -4.999325135e-03f, 1.280556685e-02f, -3.732861766e-02f, -2.119130833e+00f },
        },
    };
}
