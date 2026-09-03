#include "src/controllers/WheelBalanceLQR.hpp"
#include "src/controllers/WheelBalanceGainTable.hpp"

static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// Evaluates WheelBalanceGainTable's cubic-in-L fit at the given leg
// length, filling K[row][col] -- same math as wheeled_biped.m's
// evalGains(), mirroring the old (torque-wheel) LqrBalanceController's
// eval_gains() pattern.
static void eval_gains(float L, float K[2][4])
{
    // Clamp to the fitted grid range before evaluating -- the cubic fit
    // was only ever validated (export_wheel_balance_gains.m's closed-loop
    // eigenvalue check) over [LMID-LHALF, LMID+LHALF]. Extrapolating
    // outside it risks a wildly wrong (even sign-flipped) gain, and
    // state[StateIdx::LEG_L] reads as a literal 0.0f whenever leg FK is
    // invalid (StateManager::get_state()) -- well outside this range --
    // so this clamp is load-bearing, not just defensive.
    const float L_min = WheelBalanceGainTable::LMID - WheelBalanceGainTable::LHALF;
    const float L_max = WheelBalanceGainTable::LMID + WheelBalanceGainTable::LHALF;
    L = clampf(L, L_min, L_max);

    const float u = (L - WheelBalanceGainTable::LMID) / WheelBalanceGainTable::LHALF;
    for (int i = 0; i < WheelBalanceGainTable::NU; ++i) {
        for (int j = 0; j < WheelBalanceGainTable::NX; ++j) {
            const float *c = WheelBalanceGainTable::C[i][j];
            float v = c[0];
            for (int k = 1; k <= WheelBalanceGainTable::ORDER; ++k) v = v * u + c[k];
            K[i][j] = v;
        }
    }
}

WheelBalanceOutput wheel_balance_lqr(float L, float theta, float thetadot, float phi, float phidot)
{
    float K[2][4];
    eval_gains(L, K);

    const float x[4] = { theta, thetadot, phi, phidot };
    WheelBalanceOutput out = { 0.0f, 0.0f };
    for (int j = 0; j < 4; ++j) {
        out.ax -= K[0][j] * x[j];
        out.Tp -= K[1][j] * x[j];
    }
    return out;
}
