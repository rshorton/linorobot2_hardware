#include "Arduino.h"
#include "utility/direct_pin_read.h"
#include "hc_sr04.h"
#include <limits>

namespace
{
    const unsigned long TRIG_PULSE_WIDTH_US = 10;
    const float US_PER_METER = 5800;
}

HCSR04 *HCSR04::instances_[2] = {nullptr, nullptr};

HCSR04::HCSR04(uint8_t instance, uint8_t pin_trig_out, uint8_t pin_echo_in, uint16_t max_dist_m) :
    instance_(instance),
    pin_trig_out_(pin_trig_out),
    pin_echo_in_(pin_echo_in),
    max_dist_m_(max_dist_m),
    state_(State::kInit),
    start_time_(0),
    timeout_duration_(0),
    echo_hi_time_(0),
    echo_lo_time_(0)
{
}

HCSR04::State HCSR04::start()
{
    if (state_ == State::kInit ||
        state_ == State::kInitFail)
    {

        if (instance_ >= MAX_INSTANCES)
        {
            state_ = State::kInitFail;
            return state_;
        }

        instances_[instance_] = this;
        pinMode(pin_trig_out_, OUTPUT);
        pinMode(pin_echo_in_, INPUT_PULLUP);

        digitalWrite(pin_trig_out_, LOW);

        attachInterrupt(digitalPinToInterrupt(pin_echo_in_),
                        instance_ == 0 ? HCSR04::echo_int_instance0 : HCSR04::echo_int_instance1, CHANGE);
    }

    start_time_ = micros();
    timeout_duration_ = (float)US_PER_METER * max_dist_m_;
    state_ = State::kRanging;

    // Start a measurement
    digitalWrite(pin_trig_out_, HIGH);
    delayMicroseconds(TRIG_PULSE_WIDTH_US);
    digitalWrite(pin_trig_out_, LOW);

    return State::kRanging;
}

bool HCSR04::get_distance_m(float &distance)
{
    distance = 0.0;
    unsigned long now = micros();

    switch (state_)
    {
    case State::kRanging:
    {
        if ((now - start_time_) > timeout_duration_)
        {
            distance = std::numeric_limits<float>::infinity();
            state_ = State::kTimeout;
            return true;
        }
        break;
    }
    case State::kRanged:
    {
        distance = (float)(echo_lo_time_ - echo_hi_time_) / US_PER_METER;
        
        auto waited = now - start_time_;
        if (waited < timeout_duration_)
        {
            timeout_duration_ = timeout_duration_ - waited;
            start_time_ = now;
            state_ = State::kSettle;            
        }
        else
        {
            state_ = State::kFinished;
        }
        return true;
    }
    case State::kSettle:
    {
        if ((now - start_time_) > timeout_duration_)
        {
            state_ = State::kFinished;
        }
        break;
    }
    default:
        break;
    }
    return false;
}

bool HCSR04::finished()
{
    return state_ == State::kFinished ||
           state_ == State::kTimeout;
}

void HCSR04::pin_change()
{
    int level = digitalRead(pin_echo_in_);
    if (state_ == State::kRanging && level == 1)
    {
        echo_hi_time_ = micros();
        state_ = State::kEdgeHi;
    }
    else if (state_ == State::kEdgeHi && level == 0)
    {
        echo_lo_time_ = micros();
        state_ = State::kRanged;
    }
}

void HCSR04::echo_int_instance0()
{
    if (instances_[0])
    {
        instances_[0]->pin_change();
    }
}

void HCSR04::echo_int_instance1()
{
    if (instances_[1])
    {
        instances_[1]->pin_change();
    }
}
