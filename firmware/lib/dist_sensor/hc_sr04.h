#ifndef HC_SR04_H
#define HC_SR04_H

#include "Arduino.h"

class HCSR04
{
public:
    static const int MAX_INSTANCES = 2;

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

    float get_field_of_view() { return 15.0*M_PI/180.0; }

    void pin_change();

    static void echo_int_instance0();
    static void echo_int_instance1();

private:
    static HCSR04* instances_[MAX_INSTANCES];

    uint8_t instance_;
    uint8_t pin_trig_out_;
    int8_t pin_echo_in_;
    float max_dist_m_;

    volatile State state_;
    unsigned long start_time_;
    unsigned long timeout_duration_;
    volatile unsigned long echo_hi_time_;
    volatile unsigned long echo_lo_time_;
};

#endif // HC_SR04_H
