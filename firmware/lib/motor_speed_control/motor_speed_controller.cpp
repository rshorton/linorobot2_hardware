
#include "Arduino.h"
#include "utility/direct_pin_read.h"

#include "config.h"
#include "motor_speed_controller.h"
#include "util.h"

#undef DEBUG_PRINTS

namespace
{
    const long DEF_UPDATE_PERIOD_MS = 50;
}

MotorSpeedController::MotorSpeedController(Motor &motor, EncoderInterface &encoder, PID &pid,
                                           float direction_change_hold_off_rpm) :
    motor_(motor),
    encoder_(encoder),
    pid_(pid),
    direction_change_hold_off_rpm_(direction_change_hold_off_rpm)
{
}

void MotorSpeedController::stop()
{
    target_rpm_= 0;
    motor_.spin(0);
    pid_.reset();
}

void MotorSpeedController::update()
{
    unsigned long now = millis();

    if ((long)(now - next_update_) < 0)
    {
        return;
    }
    next_update_ = now + DEF_UPDATE_PERIOD_MS;

    current_rpm_ = encoder_.getRPM();
    auto target = target_rpm_;

    double pwm = 0.0;

    // Stop motor on direction changes and wait until stopped
    if (abs(current_rpm_) > direction_change_hold_off_rpm_ &&
        sgn(current_rpm_) != sgn(target_rpm_)) {
        target = 0.0f;
        changing_dir_ = true;
        pid_.reset();
    } else {
        changing_dir_ = false;

        pwm = pid_.compute(target, current_rpm_, false);
        // Don't allow undershoot to spin motor in opposite direction.  This can happen
        // at slow speeds.
        if (sgn(pwm) != sgn(target_rpm_)) {
            pwm = 0.0;
            pid_.reset();
        }
    }        

#ifdef DEBUG_PRINTS    
    Serial.print("MOTOR_SPD_CTRL target ");
    Serial.print(target_rpm_);
    Serial.print("   cur ");
    Serial.print(current_rpm_);
    Serial.print("   pwm ");
    Serial.println(pwm);
#endif
    motor_.spin(pwm);
}
