
#include "Arduino.h"
#include "utility/direct_pin_read.h"

#include "steering.h"
#include "util.h"

#undef DEBUG_PRINTS

namespace
{
    const long DEF_UPDATE_PERIOD_MS = 50;
}

Steering::Steering(uint8_t pin_left_limit_in, uint8_t left_limit_sensor_pos, uint8_t full_range_steps, uint8_t full_range_deg, float wheel_scale_factor, MotorInterface &motor,
                   Encoder &enc_shaft, Encoder &enc_wheel, PID &pid) : pin_left_limit_in_(pin_left_limit_in),
                                                                       left_limit_sensor_pos_(left_limit_sensor_pos),
                                                                       limit_left_(-full_range_steps / 2),
                                                                       limit_right_(full_range_steps / 2),
                                                                       steps_per_deg_((float)full_range_steps/(float)full_range_deg),
                                                                       wheel_scale_factor_(wheel_scale_factor),
                                                                       motor_(motor),
                                                                       enc_shaft_(enc_shaft),
                                                                       enc_wheel_(enc_wheel),
                                                                       pid_(pid),
                                                                       main_state_(State::kInit),
                                                                       shaft_pos_(0),
                                                                       homing_state_(HomingState::kInit),
                                                                       homed_(false),
                                                                       target_pos_(0),
                                                                       last_pos_change_(0),
                                                                       next_update_(0)
{
}

Steering::State Steering::get_state() const
{
    return main_state_;
}

bool Steering::home()
{
    main_state_ = State::kHoming;
    homing_state_ = HomingState::kInit;
    homed_ = false;
    return true;
}

bool Steering::disable()
{
    motor_.spin(0);
    if (homed_)
    {
        main_state_ = State::kDisabled;
    }
    else
    {
        main_state_ = State::kInit;
    }
    return true;
}

bool Steering::enable_steering_wheel()
{
    if (!homed_)
    {
        return false;
    }
    motor_.spin(0);
    // Make the wheel pos in sync with the actual pos
    target_pos_ = enc_shaft_.read();
    set_wheel_pos(target_pos_);
    main_state_ = State::kWheel;
    return true;
}

bool Steering::enable_external_control()
{
    if (!homed_)
    {
        return false;
    }
    // Make the current external set point equal to the current position
    target_pos_ = enc_shaft_.read();
    main_state_ = State::kExternal;
    return true;
}

int8_t Steering::set_position(int8_t target_pos)
{
    if (main_state_ == State::kExternal &&
        target_pos >= limit_left_ && target_pos <= limit_right_)
    {
        target_pos_ = target_pos;
    }
    return target_pos_;
}

float Steering::set_position_deg(float target_pos_deg)
{
    if (main_state_ == State::kExternal) {
        int8_t steps = target_pos_deg*steps_per_deg_;
        if (steps < limit_left_) {
            steps = limit_left_;
        } else if (steps > limit_right_) {
            steps = limit_right_;
        }            
        target_pos_ = steps;
    }
    return target_pos_;
}

// Steering wheel position is scaled to allow a larger turning range
void Steering::set_wheel_pos(int8_t pos)
{
    enc_wheel_.write((int8_t)((float)pos * wheel_scale_factor_));
}

int8_t Steering::get_wheel_pos()
{
    int8_t pos = (float)enc_wheel_.read() / wheel_scale_factor_;
    if (pos < limit_left_) {
        pos = limit_left_;
        set_wheel_pos(pos);
    } else if (pos > limit_right_) {
        pos = limit_right_;
        set_wheel_pos(pos);
    }
    return pos;
}

void Steering::homing_failed()
{
    main_state_ = State::kHomingFailure;
}

long Steering::homing_state_machine()
{
    switch (homing_state_)
    {
    default:
    case HomingState::kInit:
        homed_ = false;
        target_pos_ = 0;
        homing_state_ = HomingState::kSearch;
        motor_.spin(0);
        enc_shaft_.readAndReset();
        break;

    // Turn steering motor left until left limit sensor asserts
    case HomingState::kSearch:
    {
        int shaft_pos = enc_shaft_.read();

        if (digitalRead(pin_left_limit_in_) == 0)
        {
#if defined(DEBUG_PRINTS)            
            Serial.println("Steering at left limit");
#endif            
            motor_.spin(0);
            enc_shaft_.write(left_limit_sensor_pos_);            
            homing_state_ = HomingState::kCentering;
            break;
        }

        int diff = target_pos_ - shaft_pos;
        if (diff >= 0)
        {
            target_pos_--;
            if (target_pos_ < 2*left_limit_sensor_pos_ - 1) {
#if defined(DEBUG_PRINTS)            
                Serial.println("Error, failed to find left limit");
#endif                
                motor_.spin(0);
                homing_failed();
            }
#if defined(DEBUG_PRINTS)            
            Serial.print("Set pos: ");
            Serial.print(target_pos_);
            Serial.print(", actual: ");
            Serial.print(shaft_pos);
            Serial.print(", diff: ");
            Serial.println(diff);
#endif            
        }

        int pwm = pid_.compute(target_pos_, shaft_pos, false);
        motor_.spin(pwm);
        break;
    }

    // Center the wheels
    case HomingState::kCentering:
    {
        int shaft_pos = enc_shaft_.read();
        if (shaft_pos >= 0)
        {
#if defined(DEBUG_PRINTS)            
            Serial.println("Steering at center");
#endif            
            motor_.spin(0);
            set_wheel_pos(shaft_pos);
            homing_state_ = HomingState::kFinished;
            break;
        }

        int diff = target_pos_ - shaft_pos;
        if (diff <= 0)
        {
            target_pos_++;
#if defined(DEBUG_PRINTS)            
            Serial.print("Set pos: ");
            Serial.print(target_pos_);
            Serial.print(", shaft_pos: ");
            Serial.print(shaft_pos);
            Serial.print(", diff: ");
            Serial.println(diff);
#endif            
        }
        int pwm = pid_.compute(target_pos_, shaft_pos, false);
        motor_.spin(pwm);
        break;
    }
    }
    return DEF_UPDATE_PERIOD_MS;
}

long Steering::apply_position()
{
    if (target_pos_ < limit_left_)
    {
        target_pos_ = limit_left_;
    }
    else if (target_pos_ > limit_right_)
    {
        target_pos_ = limit_right_;
    }

    int8_t shaft_pos = enc_shaft_.read();

    int change = shaft_pos - target_pos_;

    // On a direction change, stop the motor first
    if (sgn(last_pos_change_) != sgn(change))
    {
        motor_.spin(0);
        pid_.reset();
    }
    else
    {
        int whl_pos = abs(change) > 1 ? target_pos_ : shaft_pos;
        int new_pwm = pid_.compute(whl_pos, shaft_pos, false);
        motor_.spin(new_pwm);
    }
    last_pos_change_ = change;
    return DEF_UPDATE_PERIOD_MS;
}

long Steering::steering_wheel_update()
{
    int8_t pos = get_wheel_pos();
    if (pos != target_pos_) {
#if defined(DEBUG_PRINTS)            
        Serial.print("Set wheel pos: ");
        Serial.println(target_pos_);
#endif        
        target_pos_ = pos;
    }
    return apply_position();
}

long Steering::external_control_update()
{
    // pos set via method
    return apply_position();
}

void Steering::update(bool enable)
{
    unsigned long now = millis();

    if ((long)(now - next_update_) < 0)
    {
        return;
    }
    shaft_pos_ = enc_shaft_.read();

    unsigned long next_update_delta = DEF_UPDATE_PERIOD_MS;

    // Stop control if steering is disabled.
    // This is used such when emergency stop is enabled.  In that case
    // the drive power will be disabled, so the control loop of this
    // object should go idle and not drive the loop harder to avoid
    // hammering the motor when estop is de-asserted.
    if (!enable) {
        motor_.spin(0);
        pid_.reset();
        return;
    }

    switch (main_state_)
    {
    default:
    case State::kInit:
        break;

    case State::kHoming:
        next_update_delta = homing_state_machine();
        if (homing_state_ == HomingState::kFinished)
        {
            homed_ = true;
            last_pos_change_ = 0;
            main_state_ = State::kWheel;
        }
        break;

    case State::kDisabled:
        break;

    case State::kWheel:
        next_update_delta = steering_wheel_update();
        break;

    case State::kExternal:
        next_update_delta = external_control_update();
        break;
    }
    next_update_ = now + next_update_delta;
}
