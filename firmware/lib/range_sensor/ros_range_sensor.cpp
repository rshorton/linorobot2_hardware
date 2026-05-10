#include "Arduino.h"
#include <stdlib.h>
#include <stdio.h>
#include <limits>

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/joint_state.h>

#include "ros_range_sensor.h"
#include "logger.h"

#undef DEBUG_LOG

// fix - move definition to common file
extern struct timespec getTime();

RosRangeSensor::RosRangeSensor(HCSR04 &sensor, const String &frame_name, const String &topic_name):
    sensor_(sensor),
    frame_name_(frame_name),
    topic_name_(topic_name)
{
}

void RosRangeSensor::init(rcl_node_t &node)
{
    if (!inited_)
    {
        rclc_publisher_init_default(
            &publisher_,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
            topic_name_.c_str());

        range_msg_.header.frame_id.data = (char *)frame_name_.c_str();
        range_msg_.header.frame_id.size = frame_name_.length() + 1;
        range_msg_.header.frame_id.capacity = range_msg_.header.frame_id.size;

        range_msg_.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
        range_msg_.field_of_view = sensor_.get_field_of_view();
        range_msg_.min_range = 0.04;
        range_msg_.max_range = 4.0; 

        inited_ = true;
    }
}

void RosRangeSensor::destroy(rcl_node_t &node)
{
    if (inited_)
    {
        rcl_publisher_fini(&publisher_, &node);
        inited_ = false;
    }
}

void RosRangeSensor::publish_range(float range)
{
    if (!inited_)
    {
        return;
    }

    struct timespec time_stamp = getTime();
    range_msg_.header.stamp.sec = time_stamp.tv_sec;
    range_msg_.header.stamp.nanosec = time_stamp.tv_nsec;

    range_msg_.range = range;
    rcl_publish(&publisher_, &range_msg_, NULL);
}

bool RosRangeSensor::update()
{
    if (!inited_)
    {
        return false;
    }

#if defined(DEBUG_LOG)
    Logger::log_message(Logger::LogLevel::Info, "RosRangeSensor::update state (enter): %d", measuring_);
#endif    

    if (measuring_)
    {
        float dist;
        if (sensor_.get_distance_m(dist))
        {
            publish_range(dist);
        }
        measuring_ = !sensor_.finished();
#if defined(DEBUG_LOG)
        Logger::log_message(Logger::LogLevel::Info, "RosRangeSensor::update state (measuring): %d", measuring_, dist);
#endif        
    }
    return measuring_;
}

void RosRangeSensor::start()
{
    if (measuring_) {
        Logger::log_message(Logger::LogLevel::Error, "RosRangeSensor: Error, start called but already measuring.");
    }
    
    sensor_.start();
    measuring_ = true;

#if defined(DEBUG_LOG)
    Logger::log_message(Logger::LogLevel::Info, "RosRangeSensor::start");
#endif    
}