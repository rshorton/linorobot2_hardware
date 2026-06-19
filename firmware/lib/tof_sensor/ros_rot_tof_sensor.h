#ifndef ROS_ROT_TOF_SENSOR_H
#define ROS_ROT_TOF_SENSOR_H

#include "Arduino.h"
#include <Wire.h>
#include <rcl/rcl.h>
#include <sensor_msgs/msg/joint_state.h>

#include "ros_tof_sensor_stepper_base.h"
#include "ros_vl53l7cx_tof_sensor.h"

// This class is used when a TOF sensor is mounted on a stepper to coordinate measurements
// with stepper position.

class RosRotatingTofSensor
{
public:
    RosRotatingTofSensor(RosVl53l7cxTofSensor &sensor, RosTofSensorStepperBase &stepper_base);
    ~RosRotatingTofSensor();

    void enable(bool enable);
    void set_scan_angle(int angle_sweep_deg);
    void update();

private:
    void apply();

    RosVl53l7cxTofSensor &sensor_;
    RosTofSensorStepperBase &stepper_base_;

    bool enabled_{false};
    int scan_angle_{0};
    int readings_{0};
    int not_ready_cnt_{0};
};

#endif // ROS_ROT_TOF_SENSOR_H
