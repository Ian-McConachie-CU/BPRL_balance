#include "src/controllers/MotorTest.hpp"
#include "src/coms/CANMotor.hpp"
#include "src/math/math.hpp"
#include "ch.h"
#include <cmath>

// Gains are gentle/placeholder on purpose -- this is a slow bench sweep,
// not a disturbance-rejecting hold (compare to HipLock's much stiffer
// KP=8.0), and output is separately capped low (see MotorTest.hpp). Tune on
// the bench like every other constant in this codebase. (Wheels no longer
// use a PID here at all -- see update()'s wheel branch and MotorTest.hpp's
// header comment for why.)
static constexpr float HIP_PID_KP = 3.0f, HIP_PID_KI = 0.0f, HIP_PID_KD = 0.15f, HIP_PID_IMAX = 1.0f;

MotorTest g_motor_test;

MotorTest::MotorTest()
    : _active(false), _motor_id(0), _is_hip(false), _dir(1),
      _hip_pid(HIP_PID_KP, HIP_PID_KI, HIP_PID_KD, HIP_PID_IMAX),
      _wheel_start_pos_rad(0.0f), _wheel_vel_target(0.0f)
{}

bool MotorTest::start(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > 6) return false;

    _active   = true;
    _motor_id = motor_id;
    _is_hip   = (motor_id <= 4);
    _dir      = 1;
    _hip_pid.reset();

    if (!_is_hip) {
        // Switch to ODrive's own native velocity mode for the duration of
        // the sweep -- see MotorTest.hpp's header comment for why (the
        // prior torque-PID approach stalled under the wheel's own static
        // friction on real hardware). can_motor_set_odrive_mode() handles
        // its own settle timing internally (see its comment in
        // CANMotor.cpp) -- no extra sleep needed here.
        can_motor_set_odrive_mode(motor_id, /*velocity_mode=*/true);
    }

    CanMotorState ms = {};
    _wheel_start_pos_rad = (can_motor_get_state(motor_id, &ms) && ms.valid) ? ms.pos_rad : 0.0f;
    return true;
}

void MotorTest::stop(bool settle)
{
    // Revert the wheel to torque mode BEFORE clearing _active -- every
    // other controller in this codebase (ActuatorSafety, BalanceController,
    // ...) drives wheels assuming torque mode; leaving one stuck in
    // velocity mode after a test would silently break normal operation the
    // next time it's armed. settle=false (ControlThread's arm-triggered
    // path) fires the same commands without blocking to fully settle them
    // -- see this class's header comment and can_motor_set_odrive_mode()'s
    // in CANMotor.hpp for why that tradeoff is deliberate there.
    //
    // Deliberately NOT gated on _active: this makes MOTOR,test,stop a safe
    // recovery action to send at any time, even after the test has already
    // ended some other way (arm-triggered auto-stop, a previous stop() that
    // didn't reach the drive, an ungraceful ground-tool exit) -- it always
    // re-asserts torque mode on whatever _motor_id was last touched.
    // Harmless no-op if that was a hip (can_motor_set_odrive_mode() rejects
    // non-ODRIVE ids) or if no test has ever run (_motor_id starts at 0,
    // same rejection).
    if (!_is_hip) {
        can_motor_set_odrive_mode(_motor_id, /*velocity_mode=*/false, settle);
    }
    _active = false;
}

void MotorTest::update(float torques[6])
{
    for (int i = 0; i < 6; ++i) torques[i] = 0.0f;
    if (!_active) return;

    CanMotorState ms = {};
    if (!can_motor_get_state(_motor_id, &ms) || !ms.valid) return;   // no verified feedback -- command nothing

    if (_is_hip) {
        const float lo = can_motor_hip_angle_min(_motor_id);
        const float hi = can_motor_hip_angle_max(_motor_id);
        float target = (_dir > 0) ? hi : lo;

        if (std::fabs(ms.pos_rad - target) < HIP_SWEEP_ARRIVE_EPS_RAD) {
            _dir = -_dir;
            _hip_pid.reset();
            target = (_dir > 0) ? hi : lo;
        }

        const float error = target - ms.pos_rad;
        torques[_motor_id - 1] = constrain_float(_hip_pid.update(error),
                                                  -HIP_SWEEP_TORQUE_LIMIT_NM, HIP_SWEEP_TORQUE_LIMIT_NM);
    } else {
        // Reverse once one full output-shaft rotation has accumulated since
        // the last reversal (pos_rad is continuously accumulating for
        // ODrive wheels -- see CANMotor.cpp's odrive_rx_cb -- no wraparound
        // handling needed). Decides the ODrive velocity target here (see
        // start()'s mode switch) rather than computing a torque via PID --
        // torques[] stays all-zero (from the reset above); the actual
        // can_motor_set_velocity() call happens in ControlThread every tick
        // (see wheel_velocity_target()), not here, so it keeps going out at
        // the full send rate even on ticks that don't call update().
        if (std::fabs(ms.pos_rad - _wheel_start_pos_rad) >= 2.0f * 3.14159265f) {
            _dir = -_dir;
            _wheel_start_pos_rad = ms.pos_rad;
        }

        _wheel_vel_target = (float)_dir * WHEEL_SWEEP_VEL_RADS;
    }
}
