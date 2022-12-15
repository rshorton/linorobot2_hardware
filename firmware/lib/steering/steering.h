#ifndef STEERING_H
#define STEERING_H

#include "Arduino.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"

class Steering
{
public:
    enum class State
    {
        kInit,
        kHoming,
        kHomingFailure,
        kWheel,
        kExternal,
        kDisabled,
    };

    enum class HomingState
    {
        kInit,
        kSearch,
        kCentering,
        kFinished
    };

public:
    Steering(uint8_t pin_left_limit_in, uint8_t full_range, float wheel_scale_factor, MotorInterface &motor,
             Encoder &enc_shaft, Encoder &enc_wheel, PID &pid);

    State get_state() const;

    bool home();
    bool disable();
    bool enable_steering_wheel();
    bool enable_external_control();

    int8_t get_position() const { return target_pos_; }
    bool set_position(int8_t target_pos);
    int8_t get_left_pos() const { return limit_left_; }
    int8_t get_right_pos() const { return limit_right_; }

    void update();

private:
    long homing_state_machine();
    long steering_wheel_update();
    long external_control_update();

    void set_wheel_pos(int8_t pos);
    int8_t get_wheel_pos();

    long apply_position();

    void homing_failed();

private:
    uint8_t pin_left_limit_in_;
    int8_t limit_left_;
    int8_t limit_right_;
    float wheel_scale_factor_;

    MotorInterface &motor_;
    Encoder &enc_shaft_;
    Encoder &enc_wheel_;
    PID &pid_;

    State main_state_;
    HomingState homing_state_;

    bool homed_;

    int8_t target_pos_;
    int8_t last_pos_change_;
    unsigned long next_update_;
};

#endif // STEERING_H
