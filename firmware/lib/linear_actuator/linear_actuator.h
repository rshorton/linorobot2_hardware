#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H

#include "Arduino.h"
#include "encoder_interface.h"
#include "pid.h"

class MotorSpeedController;

class LinearActuator
{
public:
    enum class State
    {
        kInit,
        kHoming,
        kHomingFailure,
        kControl,
        kDisabled,
    };

    enum class HomingState
    {
        kInit,
        kSearch,
        kSettle,
        kFinished
    };

    enum class HomeDetection
    {
        kSwitch,
        kPhysicalLimit
    };

public:
    LinearActuator(HomeDetection home_detection,       // method used to detect home position
                   uint8_t pin_limit_in,               // switch input (active low) if LimitDetection::kSwitch
                   MotorSpeedController &motor_ctrl,
                   EncoderInterface &encoder,
                   PID &pid,
                   uint16_t homing_rpm,                 // motor rpm while homing
                   int max_position,                    // max position
                   int pos_thresh                       // at-position threshold
                  );

    State get_state() const;

    bool home();
    bool disable();
    bool enable();

    int32_t set_target_position(int32_t target_pos);
    int32_t get_target_position() const {
        return target_pos_;
    }

    int32_t get_current_position() const;

    PID &get_pid() const {
        return pid_;
    }
    
    EncoderInterface &get_encoder() const {
        return encoder_;
    }

    unsigned long update();

private:
    long homing_state_machine();
    void homing_failed();
    long control();

private:
    HomeDetection home_detection_;
    uint8_t pin_limit_in_;

    MotorSpeedController &motor_ctrl_;
    EncoderInterface &encoder_;
    PID &pid_;

    uint16_t homing_rpm_;
    int max_position_;
    int pos_thresh_;

    State main_state_{State::kInit};
    HomingState homing_state_{HomingState::kInit};

    int32_t shaft_pos_{0};

    bool homed_{false};
    int32_t last_homing_pos_{0};
    int32_t homing_stall_cnt_{0};
    unsigned long settle_end_time_{0};

    int32_t target_pos_{0};
    unsigned long next_update_{0};

    unsigned long next_debug_print_time_{0};
};

#endif // LINEAR_ACTUATOR_H
