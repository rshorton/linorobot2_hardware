
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

MotorSpeedController::MotorSpeedController(Motor &motor, EncoderInterface &encoder, PID &pid) :
    motor_(motor),
    encoder_(encoder),
    pid_(pid),
    next_update_(0)
{
}

void MotorSpeedController::set_rpm(float rpm)
{
    target_rpm_ = rpm;
}

float MotorSpeedController::get_rpm() const
{
    return current_rpm_;
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

    // Stop motor on direction changes and wait until stopped
    if (abs(current_rpm_) > 0.0f &&
        sgn(current_rpm_) != sgn(target_rpm_)) {
        target = 0.0f;
        changing_dir_ = true;
    } else {
        changing_dir_ = false;
    }

    auto pwm = pid_.compute(target, current_rpm_, false);
    // Don't allow undershoot to spin motor in opposite direction.  This can happen
    // at slow speeds.
    if (sgn(pwm) != sgn(target_rpm_)) {
        pwm = 0.0;
        pid_.reset();
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
