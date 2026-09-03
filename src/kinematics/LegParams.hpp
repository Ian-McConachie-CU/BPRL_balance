#pragma once
#include "src/kinematics/FiveBarIK.hpp"

/*
 * Shared 5-bar link-length constants — single source of truth for every
 * consumer of FiveBarParams (StateManager's leg FK, HipLock's dynamic-target
 * leg-height hold, StandUpController/LqrBalanceController's leg-height
 * commands). Previously duplicated as a file-local static in
 * StateManager.cpp; factored out here 2026-09-02 so a re-measurement only
 * needs one edit. Keep in sync with MatLab_controls/wheeled_biped.m's
 * params() (p.l1..p.l5) if the linkage is ever remeasured.
 */
constexpr FiveBarParams LEG_PARAMS = { 0.130f, 0.220f, 0.220f, 0.130f, 0.100f };
