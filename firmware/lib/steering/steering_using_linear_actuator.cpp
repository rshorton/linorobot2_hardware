#include "Arduino.h"
#include "utility/direct_pin_read.h"

#include "config.h"
#include "steering_using_linear_actuator.h"
#include "util.h"

#define DEBUG_PRINTS

namespace
{
    const long DEF_UPDATE_PERIOD_MS = 25;
    const long INACTIVE_UPDATE_PERIOD_MS = 1000;
}

SteeringUsingLinearActuator::State SteeringUsingLinearActuator::get_state() const
{
    return state_;
}

bool SteeringUsingLinearActuator::home()
{
    state_ = State::kHoming;
    actuator_.home();
    return true;
}

bool SteeringUsingLinearActuator::enable()
{
    if (state_ == State::kDisabled)
    {
        state_ = State::kControl;
        actuator_.enable();
    }
    return true;
}

bool SteeringUsingLinearActuator::disable()
{
    actuator_.disable();
    if (state_ == State::kControl)
    {
        state_ = State::kDisabled;
    }
    else
    {
        state_ = State::kInit;
    }
    return true;
}

float SteeringUsingLinearActuator::set_angle(float angle_sp_rad)
{
    if (state_ == State::kControl)
    {
        angle_sp_ = angle_sp_rad;
        actuator_sp_ = angle_mapper_.angle_to_actuator_setting_fast(angle_sp_);
        actuator_.set_target_position(actuator_sp_);
    }
    return angle_sp_;
}

float SteeringUsingLinearActuator::get_current_angle() const
{
    auto sp = actuator_.get_current_position();
    return angle_mapper_.actuator_setting_to_angle_fast(sp);
}

bool SteeringUsingLinearActuator::homing_failed() const
{
    return state_ == State::kHomingFailure;
}

void SteeringUsingLinearActuator::update()
{
    unsigned long now = millis();

    if ((long)(now - next_update_) < 0)
    {
        return;
    }

    switch (state_)
    {
        default:
        case State::kInit:
            break;

        case State::kHoming:
        {
            auto state = actuator_.get_state();
            if (state == LinearActuator::State::kControl)
            {
                state_ = State::kControl;
                // Set to current setpoint
                actuator_.set_target_position(actuator_sp_);
            }
            else if (state == LinearActuator::State::kHomingFailure)
            {
                state_ = State::kHomingFailure;
            }
            break;
        }
        case State::kHomingFailure:
            next_update_ = now + INACTIVE_UPDATE_PERIOD_MS;
            return;

        case State::kDisabled:
            next_update_ = now + INACTIVE_UPDATE_PERIOD_MS;
            return;

        case State::kControl:
            break;
    }
    next_update_ = actuator_.update();
}
