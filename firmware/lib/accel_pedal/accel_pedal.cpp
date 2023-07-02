
#include "Arduino.h"
#include "utility/direct_pin_read.h"

#include "accel_pedal.h"

#undef DEBUG_PRINTS

AccelPedal::AccelPedal(uint8_t pin_accel_sw, uint8_t sw_pressed_level, uint8_t pin_accel_level, uint16_t min_level, uint16_t max_level,
                       long update_period_ms, float ave_alpha):
                       pin_accel_sw_(pin_accel_sw),
                       sw_pressed_level_(sw_pressed_level),
                       pin_accel_level_(pin_accel_level),
                       min_level_(min_level),
                       max_level_(max_level),
                       update_period_ms_(update_period_ms),
                       ave_alpha_(ave_alpha),
                       level_(0.0),
                       pressed_(false),
                       next_update_(0)
{
}                       

void AccelPedal::update()
{
    unsigned long now = millis();
    if ((long)(now - next_update_) < 0)
    {
        return;
    }
    next_update_ = now + update_period_ms_;

    uint8_t accel_sw = digitalRead(pin_accel_sw_);
    uint16_t level_in = analogRead(pin_accel_level_);
    pressed_ = accel_sw == sw_pressed_level_;

    // Ignore level if pedal sw not active.
    float level = 0.0;
    if (pressed_) {
        level = (float)(level_in - min_level_)/((float)max_level_ - (float)min_level_);
        if (level < 0.0) {
            level = 0.0;
        } else if (level > 1.0) {
            level = 1.0;
        }
    }
    level_ = ave_alpha_*level_ + (1.0 - ave_alpha_)*level;

#if defined(DEBUG_PRINTS)
    Serial.print("accel_sw: ");
    Serial.print(accel_sw);
    Serial.print(", level_in: ");
    Serial.print(level_in);
    Serial.print(", level_: ");
    Serial.println(level_);

#endif
}
