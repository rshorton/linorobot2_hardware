#ifndef STEERING_ANGLE_TO_ACTUATOR_MAPPER_H
#define STEERING_ANGLE_TO_ACTUATOR_MAPPER_H

#include "Arduino.h"
#include <stdio.h>
#include "config.h"

class SteeringAngleToActuatorMapper
{
public:    
    SteeringAngleToActuatorMapper() = default;
    // Maps angle in radians to equivalent actuator control value.
    // Limits angle to actuator limits.
    virtual double angle_to_actuator_setting(double angle_sp_rad) = 0;
    // Optimized looked such as using looking table
    virtual double angle_to_actuator_setting_fast(double angle_sp_rad) = 0;

    // Inverse mapping
    virtual double actuator_setting_to_angle(double act_setting) = 0;
    virtual double actuator_setting_to_angle_fast(double act_setting) = 0;

};

#endif // STEERING_ANGLE_TO_ACTUATOR_MAPPER_H
