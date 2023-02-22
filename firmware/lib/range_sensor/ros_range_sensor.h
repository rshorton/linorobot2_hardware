#ifndef ROS_RANGE_SENSOR_H
#define ROS_RANGE_SENSOR_H

#include "Arduino.h"
#include <rcl/rcl.h>
#include <sensor_msgs/msg/range.h>

#include "hc_sr04.h"

class RosRangeSensor
{
public:
    RosRangeSensor(HCSR04 &sensor, const String &frame_name, const String &topic_name);

    void create(rcl_node_t &node);
    void destroy(rcl_node_t &node);

    bool update();

private:
    void publish_range(float range);

private:
    HCSR04 &sensor_;
    const String frame_name_;
    const String topic_name_;
    bool inited_;
    bool measuring_;

    rcl_publisher_t publisher_;
    sensor_msgs__msg__Range range_msg_;
};

#endif // ROS_RANGE_SENSOR_H
