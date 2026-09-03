#pragma once
#include "src/controllers/PID.hpp"

/*
 * MotorTest — standalone bench sweep-test controller, NOT part of the
 * normal balance/car control path. When active, ControlThread calls
 * MotorTest::update() INSTEAD of RobotStateMachine::update() (see
 * threads.cpp) — every motor besides the one under test is held at zero.
 *
 * Hip under test: gentle position-PID sweep, alternating its target between
 * the hip's own safety bounds (CANMotor.hpp's can_motor_hip_angle_min/max())
 * — "rotate slowly positive until it hits its bound, then negative until it
 * hits its bound, repeat." Output still goes through can_motor_set_torque(),
 * so the SAME centralized hip safety gate in CANMotor.cpp applies on top of
 * this (belt+suspenders — a bug here still can't exceed the real bounds).
 *
 * Wheel under test: 2026-09-02 -- switched to ODrive's OWN native velocity
 * mode (can_motor_set_odrive_mode(id, true) in start(), reverted to TORQUE
 * in stop()), commanding a constant slow velocity. Since 2026-09-02's
 * 100 Hz-compute/200 Hz-send ZOH split (see threads.cpp's ControlThread),
 * update() only DECIDES the target (stored in _wheel_vel_target, read via
 * wheel_velocity_target()) rather than sending it directly -- ControlThread
 * transmits it every tick regardless of whether that tick recomputed it, so
 * the wheel gets a fresh CAN frame at the full 200 Hz send rate even though
 * the target itself only changes at most every 100 Hz. A prior version tried
 * to hold a velocity via a software
 * torque-PID loop (matching how wheels are driven everywhere else in this
 * codebase, torque-mode) but stalled under the wheel's own static friction
 * on real hardware; commanding the ODrive's own velocity controller
 * directly was confirmed to track well. Reverses the commanded velocity's
 * sign once one full output-shaft rotation has accumulated since the last
 * reversal (tracked via CanMotorState::pos_rad). Note this means a wheel
 * under test is briefly in a DIFFERENT controller mode than normal
 * operation expects -- see stop()'s comment for why reverting on every
 * exit path matters.
 */
class MotorTest {
public:
    MotorTest();

    // motor_id: 1-6. Returns false (no state change) if out of range.
    bool start(uint8_t motor_id);

    // settle: passed straight through to can_motor_set_odrive_mode() (see
    // its comment in CANMotor.hpp) -- true (default) from a
    // non-timing-critical caller (the USB command handler), false from
    // ControlThread's arm-triggered auto-abort path, where blocking ~150ms
    // to fully settle the mode change would itself stall every motor's
    // torque command for that long, a worse problem than a best-effort
    // (no-sleep) revert. Safe to call even if no test is active or the
    // last test was a hip -- see the .cpp for why.
    void stop(bool settle = true);
    bool    active()   const { return _active; }
    uint8_t motor_id() const { return _motor_id; }
    bool    is_hip()   const { return _is_hip; }

    // Last velocity target decided for a wheel under test (see update()'s
    // wheel branch) -- ControlThread reads this every tick, independent of
    // whether that tick actually recomputed it, to drive the 200 Hz ZOH send.
    // Meaningless (stale/zero) when !active() or is_hip().
    float wheel_velocity_target() const { return _wheel_vel_target; }

    // Called from ControlThread on 100 Hz compute ticks while active() —
    // fills torques[6] (zero for every id except motor_id() for a hip under
    // test; ALL zero for a wheel under test, since that target goes out via
    // wheel_velocity_target() + ControlThread's own send instead).
    void update(float torques[6]);

private:
    static constexpr float HIP_SWEEP_TORQUE_LIMIT_NM = 3.0f;   // gentle -- "slowly"
    static constexpr float HIP_SWEEP_ARRIVE_EPS_RAD  = 0.03f;  // ~1.7 deg
    static constexpr float WHEEL_SWEEP_VEL_RADS      = 1.0f;   // slow

    bool    _active;
    uint8_t _motor_id;
    bool    _is_hip;
    int     _dir;               // +1 or -1 -- which bound/direction (hip) or velocity sign (wheel)

    PID   _hip_pid;
    float _wheel_start_pos_rad; // pos_rad at the last direction reversal
    float _wheel_vel_target;    // rad/s -- see wheel_velocity_target()
};

extern MotorTest g_motor_test;
