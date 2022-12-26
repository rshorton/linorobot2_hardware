#ifndef ACCEL_PEDAL_H
#define ACCEL_PEDAL_H

#include "Arduino.h"

class AccelPedal
{
public:
    AccelPedal(uint8_t pin_accel_sw, uint8_t sw_pressed_level, uint8_t pin_accel_level, uint16_t min_level, uint16_t max_level,
               long update_period_ms, float ave_alpha);

    float get_level() const { return level_; }
    void update();

private:
    uint8_t pin_accel_sw_;
    uint8_t sw_pressed_level_;
    uint8_t pin_accel_level_;
    uint16_t min_level_;
    uint16_t max_level_;
    long update_period_ms_;
    float ave_alpha_;
    float level_;
    unsigned long next_update_;
};

#endif // ACCEL_PEDAL_H
