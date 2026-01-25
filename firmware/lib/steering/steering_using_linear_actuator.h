#ifndef STEERING_USING_LINEAR_ACTUATOR_H
#define STEERING_USING_LINEAR_ACTUATOR_H

#include "Arduino.h"
#include "linear_actuator.h"
#include "steering_angle_to_actuator_mapper.h"

class SteeringUsingLinearActuator
{
public:
    enum class State
    {
        kInit,
        kHoming,
        kHomingFailure,
        kControl,
        kDisabled
    };

public:
    SteeringUsingLinearActuator(LinearActuator &actuator, SteeringAngleToActuatorMapper &angle_mapper) :
        actuator_(actuator),
        angle_mapper_(angle_mapper)
    {
        actuator_sp_ = static_cast<int>(angle_mapper_.angle_to_actuator_setting(angle_sp_));
    }

    State get_state() const;

    bool home();
    bool disable();
    bool enable();

    bool homing_failed() const;

    float get_angle() const { return angle_sp_; }
    float set_angle(float angle_sp_);

    float get_current_angle() const;

    void update();

private:
    LinearActuator &actuator_;
    SteeringAngleToActuatorMapper &angle_mapper_;

    State state_{State::kInit};

    float angle_sp_{0.0f};
    int actuator_sp_{0};
    unsigned long next_update_{0};
};

#endif // STEERING_USING_LINEAR_ACTUATOR_H
