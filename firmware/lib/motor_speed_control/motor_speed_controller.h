#ifndef MOTOR_SPEED_CONTROLLER_H
#define MOTOR_SPEED_CONTROLLER_H

#include "Arduino.h"
#include "encoder_interface.h"
#include "motor.h"
#include "pid.h"

class MotorSpeedController
{
public:
    MotorSpeedController(Motor &motor,
                         EncoderInterface &encoder,
                         PID &pid);

    void set_rpm(float rpm);
    float get_rpm() const;    
    void stop();
    void update();

    PID &get_pid() const {
        return pid_;
    }

    bool is_changing_dir() const
    {
        return changing_dir_;
    }

private:
    Motor &motor_;
    EncoderInterface &encoder_;
    PID &pid_;

    float target_rpm_{0.0f};
    float current_rpm_{0.0f};
    bool changing_dir_{false};
    unsigned long next_update_{0};
};

#endif // MOTOR_SPEED_CONTROLLER_H
