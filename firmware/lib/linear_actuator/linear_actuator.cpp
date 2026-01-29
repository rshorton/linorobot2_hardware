
#include "Arduino.h"
#include "utility/direct_pin_read.h"

#include "config.h"
#include "motor_speed_controller.h"
#include "linear_actuator.h"
#include "util.h"

#define DEBUG_PRINTS

namespace
{
    const long DEF_UPDATE_PERIOD_MS = 50;
    const int HOMING_STALL_MAX_ENC_CHANGE = 5;
    const int HOMING_STALL_MAX_COUNT = (1000/DEF_UPDATE_PERIOD_MS);
    const long HOMING_SETTLE_DURATION_MS = 1000; 
    const long DEBUG_PRINT_PERIOD_MS = 250;
}

LinearActuator::LinearActuator(HomeDetection home_detection, uint8_t pin_limit_in,
                               MotorSpeedController &motor_ctrl, EncoderInterface &encoder, PID &pid, 
                               uint16_t homing_rpm, int max_position, int pos_thresh) :
    home_detection_(home_detection),
    pin_limit_in_(pin_limit_in),
    motor_ctrl_(motor_ctrl),
    encoder_(encoder),
    pid_(pid),
    homing_rpm_(homing_rpm),
    max_position_(max_position),
    pos_thresh_(pos_thresh)
{
    pinMode(pin_limit_in_, INPUT_PULLUP);
}

LinearActuator::State LinearActuator::get_state() const
{
    return main_state_;
}

bool LinearActuator::home()
{
    main_state_ = State::kHoming;
    homing_state_ = HomingState::kInit;
    homed_ = false;
    return true;
}

bool LinearActuator::disable()
{
    motor_ctrl_.set_target_rpm(0);
    motor_ctrl_.update();
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

bool LinearActuator::enable()
{
    if (!homed_)
    {
        return false;
    }
    // Make the current external set point equal to the current position
    target_pos_ = encoder_.read();
    main_state_ = State::kControl;
    return true;
}

int32_t LinearActuator::set_target_position(int32_t target_pos)
{
    target_pos_ = target_pos;
    if (target_pos_ < 0)
    {
        target_pos_ = 0;
    }
    else if (target_pos_ > max_position_)
    {
        target_pos_ = max_position_;
    }

#if defined(DEBUG_PRINTS)            
    Serial.print("LinearActuator: set position, target: ");
    Serial.println(target_pos_);
#endif
    return target_pos_;
}

int32_t LinearActuator::get_current_position() const
{
    return encoder_.read();
}

void LinearActuator::homing_failed()
{
    main_state_ = State::kHomingFailure;
}

long LinearActuator::homing_state_machine()
{
    long next_update = DEF_UPDATE_PERIOD_MS;

    switch (homing_state_)
    {
        default:
        case HomingState::kInit:
            homed_ = false;
            target_pos_ = 0;
            last_homing_pos_ = 0;
            homing_stall_cnt_ = 0;
            homing_state_ = HomingState::kSearch;
            motor_ctrl_.set_target_rpm(0);
            pid_.reset();
            encoder_.readAndReset();
            break;

        case HomingState::kSearch:
        {
            int shaft_pos = encoder_.read();
            Serial.print("Enc pos: ");
            Serial.println(shaft_pos);

            if (abs(shaft_pos) > max_position_)
            {
                max_position_ = abs(shaft_pos);
            }

            // Still moving?
            if (max_position_ - last_homing_pos_ > HOMING_STALL_MAX_ENC_CHANGE)
            {
                homing_stall_cnt_ = 0;
                last_homing_pos_ = max_position_;
            }
            else
            {
                homing_stall_cnt_++;
            }

            // Stop if the limit switch reached or the physical limit depending on the selected mode
            if ((home_detection_ == HomeDetection::kSwitch && digitalRead(pin_limit_in_) == 0) ||
                (home_detection_ == HomeDetection::kPhysicalLimit && homing_stall_cnt_ > HOMING_STALL_MAX_COUNT))
            {
    #if defined(DEBUG_PRINTS)            
                Serial.println("Steering at limit");
    #endif            
                motor_ctrl_.stop();
                homing_state_ = HomingState::kSettle;
                settle_end_time_ = millis() + HOMING_SETTLE_DURATION_MS;
            }
            else
            {
                motor_ctrl_.set_target_rpm(-homing_rpm_);
                motor_ctrl_.update();
    // fix check for stall or timeout        
            }            
            break;
        }
        case HomingState::kSettle:
        {
            unsigned long now = millis();
            if ((long)(now - settle_end_time_) > 0)
            {
                encoder_.write(0);            
                target_pos_ = 0;
                homing_state_ = HomingState::kFinished;
            }
            else
            {
                motor_ctrl_.update();
            }
        }
    }
    return next_update;
}

long LinearActuator::control()
{
    auto shaft_pos = encoder_.read();
    int new_rpm = 0;
    if (pos_thresh_ == -1 ||
        abs(target_pos_ - shaft_pos) > pos_thresh_) {

        // If the motor controller is waiting for the motor to stop
        // so it can change directory, then don't update the position PID.            
        if (motor_ctrl_.is_changing_dir())
        {
            // Use last computed value
            new_rpm = pid_.getLastComputedValue();
        }
        else
        {            
            new_rpm = pid_.compute(target_pos_, shaft_pos, false);
        }            
    }
    else
    {
        pid_.reset();
    }

    motor_ctrl_.set_target_rpm(new_rpm);
    motor_ctrl_.update();

#ifdef DEBUG_PRINTS
    unsigned long now = millis();
    if ((long)(now - next_debug_print_time_) > 0)
    {
        Serial.print("LinearActuator: control, target: ");
        Serial.print(target_pos_);
        Serial.print(", shaft_pos: ");
        Serial.print(shaft_pos);
        Serial.print(", new_rpm: ");
        Serial.println(new_rpm);
        next_debug_print_time_ = now + DEBUG_PRINT_PERIOD_MS;       
    }
#endif            
    return DEF_UPDATE_PERIOD_MS;
}

unsigned long LinearActuator::update()
{
    unsigned long now = millis();

    if ((long)(now - next_update_) < 0)
    {
        return next_update_;
    }
    shaft_pos_ = encoder_.read();

    unsigned long next_update_delta = DEF_UPDATE_PERIOD_MS;

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
            target_pos_ = 0;
            main_state_ = State::kControl;
        }
        break;

    case State::kDisabled:
        break;

    case State::kControl:
        next_update_delta = control();
        break;
    }
    next_update_ = now + next_update_delta;
    return next_update_;
}
