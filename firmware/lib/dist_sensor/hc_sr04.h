#ifndef HC_SR04_H
#define HC_SR04_H

#include "Arduino.h"

class HCSR04
{
public:
    static const int MAX_INSTANCES = 2;
    static constexpr float field_of_view = 15.0f;

private:
    enum class State
    {
        kInit,
        kInitFail,
        kRanging,
        kEdgeHi,
        kError,
        kRanged,
        kSettle,
        kFinished,
        kTimeout
    };

public:
    HCSR04(uint8_t instance, uint8_t pin_trig_out, uint8_t pin_echo_in, uint16_t max_dist_m);

    State start();
    bool get_distance_m(float &distance);
    bool finished();

    float get_field_of_view() { return field_of_view*M_PI/180.0; }

    void pin_change();

    static void echo_int_instance0();
    static void echo_int_instance1();

private:
    static HCSR04* instances_[MAX_INSTANCES];

    uint8_t instance_;
    uint8_t pin_trig_out_;
    int8_t pin_echo_in_;
    float max_dist_m_;
    
    unsigned long timeout_duration_{0};
    volatile State state_{State::kInit};
    unsigned long start_time_{0};
    volatile unsigned long echo_hi_time_{0};
    volatile unsigned long echo_lo_time_{0};
    float distance_{0.0f};
};

#endif // HC_SR04_H
