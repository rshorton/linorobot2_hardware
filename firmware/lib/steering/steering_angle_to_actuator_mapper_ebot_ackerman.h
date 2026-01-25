#ifndef STEERING_ANGLE_TO_ACTUATOR_MAPPER_EBOT_ACKERMAN_H
#define STEERING_ANGLE_TO_ACTUATOR_MAPPER_EBOT_ACKERMAN_H

#include "Arduino.h"
#include <stdio.h>
#include "config.h"
#include "steering_angle_to_actuator_mapper.h"

class SteeringAngleToActuatorMapperEbotAckerman: public SteeringAngleToActuatorMapper
{
public:    
    struct AngleToCalcValues
    {
        double rw_angle{0.0};
        double tie_rod_act_end_pos_mm{0.0};
        double act_pos_mm{0.0};
        bool at_min_pos{false};
        bool at_max_pos{false};
    };

    SteeringAngleToActuatorMapperEbotAckerman() = default;
    ~SteeringAngleToActuatorMapperEbotAckerman()
    {
        if (lookup_table_)
        {
            delete[] lookup_table_;
        }
    }

    double angle_to_actuator_setting(double angle_sp_rad);
    double angle_to_actuator_setting_fast(double angle_sp_rad);
    double angle_to_actuator_setting(double angle_sp_rad, AngleToCalcValues* debug_out);

    double actuator_setting_to_angle(double act_setting);
    double actuator_setting_to_angle_fast(double act_setting);

private:
    struct LookupTableEntry
    {
        int angle{0};
        double act_setting{0.0};
    };

    bool build_lookup_table();

    int num_table_entries_{0};
    LookupTableEntry *lookup_table_ = nullptr;
};

#endif // STEERING_ANGLE_TO_ACTUATOR_MAPPER_EBOT_ACKERMAN_H
