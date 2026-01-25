#include "Arduino.h"
#include "utility/direct_pin_read.h"

#include "config.h"
#include "steering_angle_to_actuator_mapper_ebot_ackerman.h"
#include "util.h"

#undef DEBUG_PRINTS

// This class maps the steering angle to the actuator setting needed to steer the right wheel of the
// Elsabot robot.

//  Ebot ackermann steering diagram
//    
//    ##      o TR_SA_JOINT                          SA - steering arm  
//    ##     / \                                     TR - tie rod
//    ##  SA/   \ TR                                 RW - right front wheel
//    ##   /     \                    \/             KP - king pin
// RW ##--O-------\-------------------------- Axle   AS - actuator slide
//    ## KP     |  \   <-- AS -->                    AO - actuator origin (=> wheel left max pos)
//    ##        |   o===================== ---       ASO - actuator slide offset from axle
//    ## >|     |< KP_TO_AO           /\ ASO         KP_TO_AO - right wheel king pin to actuator origin
//    ##       >|   |< AP                            AP - actuator position
//             AO
//                      vvvvv FRONT vvvvv

namespace
{
const double tie_rod_len = 142.0; //134.0;          // TR
const double steering_arm_len = 44.0;               // SA
const double tie_rod_sa_joint_to_axle_len_at_zero_wheel_steering = 32.0;
// constexpr double steering_arm_angle_neutral = asin(tr_sa_joint_to_axle_len_at_wheel_steering/steering_arm_len);
// Above not supported but would evaluate to 46.66 deg.
const double steering_arm_angle_neutral = M_PI*46.66/180.0;     // At steering angle = 0
const double steering_act_slide_off_from_from_axle = 30.0; //20.0;      // ASO
const double rw_king_pin_to_act_min_pos_mm = 135.0; //132.0;

const double min_act_slide_pos_mm = 0.0; //8.0;
const double max_act_slide_pos_mm = 43.0; //38.0; //42.0;
const double act_mm_per_rev = 42.0/3.0;         // Measured: moved 42mm for 3 revs

const double steering_motor_gear_ratio = 70.0;
const double steering_motor_enc_ticks_per_motor_shaft_rev = 64.0;
const double steering_motor_enc_ticks_per_ext_shaft_rev = steering_motor_gear_ratio*steering_motor_enc_ticks_per_motor_shaft_rev;

const double lu_table_max_abs_angle_deg = 40;

inline double deg_to_rad(double angle)
{
    return angle*M_PI/180.0;
}

inline double rad_to_deg(double angle)
{
    return angle*180.0/M_PI;
}

}

// Angle input corresponds to the "Car-Like (Bicycle) Model" described on this page:
// https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html
// 
// FR_WHEELS_DISTANCE - wheelbase
// LR_WHEELS_DISTANCE - left to right wheel distance
double SteeringAngleToActuatorMapperEbotAckerman::angle_to_actuator_setting(double angle_sp_rad, AngleToCalcValues* calc_vals)
{
    // Convert angle input to right-wheel angle assuming ackerman steering.
    //                         (              2*wheel_base*sin(angle_sp_rad)                                 )
    // angle_right_wheel = atan( ----------------------------------------------------------------------------)
    //                         ( 2*wheel_base*cos(angle_sp_rad) + left_to_right_wheel_dist*sin(angle_sp_rad) )
    // 
    auto rw_angle = atan(2*FR_WHEELS_DISTANCE*1000.0*sin(angle_sp_rad)/
                        (2*FR_WHEELS_DISTANCE*1000.0*cos(angle_sp_rad) + LR_WHEELS_DISTANCE*1000.0*sin(angle_sp_rad)));

    // Convert right-wheel angle to actuator setting    
    // Step 1. Position of tie rod end conn on actuator slide relative to right wheel king pin
    auto tie_rod_act_end_pos_mm = 
        sqrt(pow(tie_rod_len, 2.0f) - pow(sin(steering_arm_angle_neutral + rw_angle)*steering_arm_len+steering_act_slide_off_from_from_axle, 2.0f)) +
        cos(steering_arm_angle_neutral + rw_angle)*steering_arm_len;
    // Step 2. Position of actuator slide relative to actuator 0 position in mm
    auto act_pos_mm = tie_rod_act_end_pos_mm - rw_king_pin_to_act_min_pos_mm;

    // Step 3. Limit to physical limits of actuator
    bool at_min_pos = false;
    bool at_max_pos = false;

    if (act_pos_mm < min_act_slide_pos_mm)
    {
        act_pos_mm = min_act_slide_pos_mm;
        at_min_pos = true;
    }
    else if (act_pos_mm > max_act_slide_pos_mm)
    {
        act_pos_mm = max_act_slide_pos_mm;
        at_max_pos = true;
    }

    // Step 4. Convert pos to encoder ticks
    auto act_enc_ticks = act_pos_mm/act_mm_per_rev*steering_motor_enc_ticks_per_ext_shaft_rev;

    if (calc_vals)
    {
        calc_vals->rw_angle = rw_angle;
        calc_vals->tie_rod_act_end_pos_mm = tie_rod_act_end_pos_mm;
        calc_vals->act_pos_mm = act_pos_mm;
        calc_vals->at_min_pos = at_min_pos;
        calc_vals->at_max_pos = at_max_pos;
    }
    return act_enc_ticks;
}

// Build a lookup table for faster mapping and to also support
// reverse mapping (since computationally harder)
bool SteeringAngleToActuatorMapperEbotAckerman::build_lookup_table()
{
    if (lookup_table_)
    {
        return true;
    }

    AngleToCalcValues calc_vals;

    // Note that actuator position varies inversely to the angle.

    // Find the first angle that results in the maximum actuator position
    auto min_angle = -lu_table_max_abs_angle_deg;
    for (auto angle = 0.0; angle > min_angle; angle -= 1.0)
    {
        auto rad = deg_to_rad(angle);

        angle_to_actuator_setting(rad, &calc_vals);
        if (calc_vals.at_max_pos)
        {
            min_angle = angle;
            break;
        }
    }

    // Find the first angle that results in the minimum actuator position
    auto max_angle = lu_table_max_abs_angle_deg;
    for (auto angle = 0.0; angle < max_angle; angle += 1.0)
    {
        auto rad = deg_to_rad(angle);

        angle_to_actuator_setting(rad, &calc_vals);
        if (calc_vals.at_min_pos)
        {
            max_angle = angle;
            break;
        }
    }

    // Allocate and fill the table
    // (A 1-degree increment is sufficient for the mechanical setup of elsabot)
    num_table_entries_ = static_cast<int>(max_angle - min_angle + 1);

#ifdef DEBUG_PRINTS
    Serial.print("lu table num entries: ");
    Serial.print(num_table_entries_);
    Serial.println("\r\n");
#endif    

    lookup_table_ = new LookupTableEntry[num_table_entries_];
    if (!lookup_table_)
    {
        return false;
    }

    auto angle = min_angle;
    for (int entry = 0; entry < num_table_entries_; entry++, angle += 1.0)
    {
        auto rad = deg_to_rad(angle);

        auto act_setting = angle_to_actuator_setting(rad, nullptr);
        lookup_table_[entry].angle = angle;
        lookup_table_[entry].act_setting = act_setting;

#ifdef DEBUG_PRINTS        
        Serial.print("lu table entry, idx:");
        Serial.print(entry);
        Serial.print(", angle:");
        Serial.print(lookup_table_[entry].angle);
        Serial.print(", act setting: ");
        Serial.print(lookup_table_[entry].act_setting);
        Serial.println("\r\n");
#endif        
    }
    return true;
}

// Uses lookup table
double SteeringAngleToActuatorMapperEbotAckerman::angle_to_actuator_setting_fast(double angle_sp_rad)
{
    if (!build_lookup_table())
    {
        return 0.0;
    }

    // Round to nearest degree since table stores positions at 1-degree increments.
    auto angle_deg = static_cast<int>(rad_to_deg(angle_sp_rad) + 0.5);

    auto idx_found = num_table_entries_ - 1;
    for (auto idx = 0; idx < num_table_entries_; idx++)
    {
        if (angle_deg <= lookup_table_[idx].angle)
        {
            idx_found = idx;
            break;
        }
    }
    return lookup_table_[idx_found].act_setting;
}

// Direct calculation using formulas
double SteeringAngleToActuatorMapperEbotAckerman::angle_to_actuator_setting(double angle_sp_rad)
{
    return angle_to_actuator_setting(angle_sp_rad, nullptr);
}


// Direct calculation using formulas
double SteeringAngleToActuatorMapperEbotAckerman::actuator_setting_to_angle(double act_setting)
{
    // Direct calc not supported
    return actuator_setting_to_angle_fast(act_setting);
}

// Uses lookup table
double SteeringAngleToActuatorMapperEbotAckerman::actuator_setting_to_angle_fast(double act_setting)
{
    if (!build_lookup_table())
    {
        return 0.0;
    }
    
    // Note that actuator position varies inversely to the angle.

    if (act_setting >= lookup_table_[0].act_setting)
    {
        return deg_to_rad(lookup_table_[0].angle);
    }
    else if (act_setting <= lookup_table_[num_table_entries_ - 1].act_setting)
    {
        return deg_to_rad(lookup_table_[num_table_entries_ - 1].angle);
    }
    
    for (auto idx = num_table_entries_ - 1; idx >= 0; idx--)
    {
        if (act_setting < lookup_table_[idx].act_setting)
        {
            // Assumes 1 deg step per entry
            auto angle_delta = (act_setting - lookup_table_[idx].act_setting)/
                                        (lookup_table_[idx + 1].act_setting - lookup_table_[idx].act_setting);
            return deg_to_rad(lookup_table_[idx].angle + angle_delta);                                        
        }
    }
    // Unexpected
    return 0.0;
}
