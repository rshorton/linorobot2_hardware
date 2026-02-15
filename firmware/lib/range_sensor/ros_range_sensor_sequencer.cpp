#include "Arduino.h"
#include <stdlib.h>

#include "ros_range_sensor_sequencer.h"
#include "ros_range_sensor.h"

RosRangeSensorSequencer::RosRangeSensorSequencer(RosRangeSensor* (&sensors)[], unsigned int cnt):
    sensors_(sensors),
    cnt_(cnt)
{
}

void RosRangeSensorSequencer::update()
{
    if (cnt_ == 0 && !sensors_[idx_]) {
        return;
    }

    if (!sensors_[idx_]->is_busy()) {
        if (++idx_ >= cnt_) {
            idx_ = 0;
        }        
    }
    sensors_[idx_]->update();
}
