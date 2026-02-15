#ifndef ROS_ROT_RANGE_SENSOR_H
#define ROS_ROT_RANGE_SENSOR_H

#include "Arduino.h"

#include <rcl/rcl.h>
#include <sensor_msgs/msg/joint_state.h>

#include "serial_bus_servo.h"

class RosRangeSensor;

class RosRotatingRangeSensor
{
public:
    enum class State
    {
        kInit,
        kReady,
        kMove,
        kDelay,
        kRange
    };

public:
    RosRotatingRangeSensor(const String &joint_frame_name, RosRangeSensor &range_sensor, SerialServo &servo,
                           const float (&positions)[], int position_cnt, float zero_offset_deg, int pos_delay);

    void init(rcl_node_t &node);
    void destroy(rcl_node_t &node);

    void start(bool scan = true);
    void stop();
    void update();

private:
    unsigned long move_next();
    unsigned long move_servo(float pos);

    void publish_servo_position(rcl_publisher_t &pub, float angle);

private:
    const String joint_frame_name_;
    RosRangeSensor &range_sensor_;
    SerialServo &servo_;
    const float (&positions_deg_)[];
    int position_cnt_;
    float zero_offset_deg_;
    int pos_delay_;

    float move_ms_per_degree_{10.0f};        
    State state_{State::kInit};
    unsigned long next_update_{0};

    bool scan_{true};
    int pos_idx_{0};
    float servo_pos_{0.0f};

    rcl_publisher_t servo_joint_position_publisher_;
    sensor_msgs__msg__JointState servo_joint_msg_;
};

#endif // ROS_ROT_RANGE_SENSOR_H
