#include "src/controllers/RobotStateMachine.hpp"
#include "src/controllers/BalanceController.hpp"
#include "src/controllers/CarController.hpp"
#include "src/controllers/StandUpController.hpp"
#include "src/coms/CANMotor.hpp"
#include <cstring>

static BalanceController   s_balance;
static CarController       s_car;
static StandUpController   s_standup;

// True whenever this mode wants the wheels in ODrive VELOCITY mode
// (can_motor_set_velocity()) rather than the torque path
// (motor_torques[4]/[5]) -- see RobotStateMachine.hpp's class header.
static bool wants_wheel_velocity(RobotMode m)
{
    return m == ROBOT_CAR || m == ROBOT_STANDING_UP
        || (m == ROBOT_BALANCING && BalanceController::USES_WHEEL_VELOCITY);
}

void RobotStateMachine::update(const float state[StateIdx::N],
                                const float input[InputIdx::N_INPUTS],
                                bool        armed,
                                float       motor_torques[6])
{
    // Default: all motors zero torque
    memset(motor_torques, 0, 6 * sizeof(float));

    const RobotMode prev_mode = _mode;

    if (!armed) {
        _mode = ROBOT_IDLE;
    } else if (input[InputIdx::MODE_SW] < 0.0f) {
        _mode = ROBOT_CAR;
    } else if (prev_mode == ROBOT_IDLE || prev_mode == ROBOT_CAR) {
        // Just asked to balance from a non-balancing state -- always pass
        // through STANDING_UP first, never straight to BALANCING (see
        // class header / README's planned mode state machine).
        _mode = ROBOT_STANDING_UP;
    } else if (prev_mode == ROBOT_STANDING_UP) {
        _mode = s_standup.is_standing(state) ? ROBOT_BALANCING : ROBOT_STANDING_UP;
    } else {
        _mode = ROBOT_BALANCING;   // was already BALANCING -- stays
    }

    if (_mode != prev_mode) {
        if (_mode == ROBOT_STANDING_UP) {
            s_standup.reset(state);
        } else if (_mode == ROBOT_BALANCING && prev_mode == ROBOT_STANDING_UP) {
            // Hand off the leg-height ramp continuously -- see
            // BalanceController::reset()'s header.
            s_balance.reset(state);
        } else if (_mode == ROBOT_CAR) {
            s_car.reset();
        }

        // Wheel ODrive mode switch, on any edge crossing into/out of
        // wanting velocity mode -- generalizes what used to be a
        // ROBOT_CAR-only check (see class header). settle=false: this
        // runs inside ControlThread's compute step, and a blocking
        // ~150ms settle here would itself stall every motor's command for
        // that long -- same tradeoff MotorTest's arm-triggered abort
        // makes, see threads.cpp.
        const bool was_vel = wants_wheel_velocity(prev_mode);
        const bool now_vel = wants_wheel_velocity(_mode);
        if (now_vel && !was_vel) {
            can_motor_set_odrive_mode(5, /*velocity_mode=*/true, /*settle=*/false);
            can_motor_set_odrive_mode(6, /*velocity_mode=*/true, /*settle=*/false);
        } else if (!now_vel && was_vel) {
            // Revert to torque mode immediately so the normal torque path
            // (BalanceController under BALANCE_CTRL_PID, or this
            // function's own zero-torque IDLE default) actually reaches
            // the wheel -- nothing else in this codebase expects a wheel
            // left in velocity mode, see can_motor_set_odrive_mode()'s
            // header comment.
            can_motor_set_odrive_mode(5, /*velocity_mode=*/false, /*settle=*/false);
            can_motor_set_odrive_mode(6, /*velocity_mode=*/false, /*settle=*/false);
        }
    }

    if (!armed) return;   // motor_torques already zero -- disarm hard-idles every motor

    switch (_mode) {
    case ROBOT_STANDING_UP:
        s_standup.update(state, motor_torques);
        break;
    case ROBOT_BALANCING:
        s_balance.update(state, input, motor_torques);
        break;
    case ROBOT_CAR:
        // hips left idling (motor_torques[0..3] stay zero from the memset
        // above -- no crouch hold yet, see class header). Wheels are
        // driven entirely off the velocity path: s_car only decides the
        // targets here, read via wheel_vel_L()/_R() below by
        // WheelSendThread, which sends can_motor_set_velocity() itself
        // (see threads.cpp) -- motor_torques[4]/[5] stay zero and go
        // unused.
        s_car.update(input);
        break;
    default:
        break;
    }
}

bool RobotStateMachine::wheel_velocity_mode() const { return wants_wheel_velocity(_mode); }

float RobotStateMachine::wheel_vel_L() const
{
    switch (_mode) {
    case ROBOT_CAR:         return s_car.left();
    case ROBOT_STANDING_UP: return s_standup.wheel_vel_L();
    case ROBOT_BALANCING:   return s_balance.wheel_vel_L();
    default:                return 0.0f;
    }
}

float RobotStateMachine::wheel_vel_R() const
{
    switch (_mode) {
    case ROBOT_CAR:         return s_car.right();
    case ROBOT_STANDING_UP: return s_standup.wheel_vel_R();
    case ROBOT_BALANCING:   return s_balance.wheel_vel_R();
    default:                return 0.0f;
    }
}
