#include "Arduino.h"
#include "utility/direct_pin_read.h"

#include "hc_sr04.h"

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

    state_ = State::kRanging;
    start_time_ = micros();

    // Start a measurement
    digitalWrite(pin_trig_out_, HIGH);
    delayMicroseconds(TRIG_PULSE_WIDTH_US);
    digitalWrite(pin_trig_out_, LOW);

    return State::kRanging;
}

HCSR04::State HCSR04::get_distance_m(float &distance)
{
    distance = 0.0;
    unsigned long now = micros();

    switch (state_)
    {
    case State::kRanging:
        if ((now - start_time_) > ((float)US_PER_METER * max_dist_m_))
        {
            state_ = State::kTimeout;
        }
        break;

    case State::kFinished:
        distance = (float)(echo_lo_time_ - echo_hi_time_) / US_PER_METER;
        break;

    default:
        break;
    }
    return state_;
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
        state_ = State::kFinished;
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
