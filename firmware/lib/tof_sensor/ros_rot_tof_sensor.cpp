#include "Arduino.h"
#include <stdlib.h>
#include <stdio.h>
#include <limits>

#include "ros_rot_tof_sensor.h"
#include "logger.h"

namespace {
    constexpr int READINGS_PER_POS = 2;    
}

RosRotatingTofSensor::RosRotatingTofSensor(RosVl53l7cxTofSensor &sensor, RosTofSensorStepperBase &stepper_base) :
    sensor_(sensor),
    stepper_base_(stepper_base)
{
}

RosRotatingTofSensor::~RosRotatingTofSensor()
{
    stepper_base_.stop();
}


void RosRotatingTofSensor::enable(bool enable)
{
    enabled_ = enable;
    apply();

}

void RosRotatingTofSensor::set_scan_angle(int angle_sweep_deg)
{
    if (scan_angle_ == angle_sweep_deg) {
        return;
    }
    scan_angle_ = angle_sweep_deg;
    apply();
}

void RosRotatingTofSensor::apply()
{
    if (enabled_) {
        stepper_base_.start(scan_angle_);
    } else {
        stepper_base_.stop();
    }        
}

// Should be called at the target rate for TOF readings.
void RosRotatingTofSensor::update()
{
    if (!enabled_) {
        return;
    }
    stepper_base_.update();

    bool not_ready = false;

    if (scan_angle_ > 0) {
        if (stepper_base_.at_position()) {
            stepper_base_.publish_joint_state();
            if (sensor_.update()) {
                if (++readings_ >= READINGS_PER_POS) {
                    readings_ = 0;
                    stepper_base_.goto_next_position();
                }
            } else {
                not_ready = true;
            }
        }            
    } else {
        stepper_base_.publish_joint_state();
        if (!sensor_.update()) {
            not_ready = true;
        }
    }
    if (not_ready) {
        Logger::log_message(Logger::LogLevel::Info, "RosRotatingTofSensor::update, not ready");
        if (++not_ready_cnt_ > 15*10) {
            sensor_.sensor_init();
            Logger::log_message(Logger::LogLevel::Info, "RosRotatingTofSensor::update, re-init sensor");
            not_ready_cnt_ = 0;
        }
    } else {
        not_ready_cnt_ = 0;
    }        
}
