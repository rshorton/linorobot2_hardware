#ifndef ROS_RANGE_SENSOR_SEQUENCER_H
#define ROS_RANGE_SENSOR_SEQUENCER_H

#include "Arduino.h"

class RosRangeSensor;

class RosRangeSensorSequencer
{
public:
    RosRangeSensorSequencer(RosRangeSensor* (&sensors)[], unsigned int cnt);
    void update();

private:
    RosRangeSensor* (&sensors_)[];
    unsigned int cnt_{0};
    unsigned int idx_{0};
};

#endif // ROS_RANGE_SENSOR_SEQUENCER_H
