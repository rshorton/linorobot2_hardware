#ifndef ROT_DIST_SENSOR_H
#define ROT_DIST_SENSOR_H

#include "Arduino.h"

#include <rcl/rcl.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/joint_state.h>

#include "hc_sr04.h"
#include "serial_bus_servo.h"

class RotatingDistSensor
{
public:
    enum class State
    {
        kInit,
        kReady,
        kMove,
        kRange
    };

public:
    RotatingDistSensor(HCSR04 &dist_sensor0, SerialServo &servo0);

    void init(rcl_node_t &node);
    void destroy(rcl_node_t &node);

    void start(bool scan = true);
    void stop();
    void update();

private:
    unsigned long move_next();
    unsigned long move_servo(int16_t pos);

    float servo_pos_to_angle();

    void publish_range(rcl_publisher_t &pub, const char* frame, float range, float angle);
    void publish_servo_position(rcl_publisher_t &pub, float angle);

private:
    HCSR04 &dist_sensor0_;
    SerialServo &servo0_;
    State state_;
    unsigned long next_update_;

    bool scan_;
    int16_t servo_pos_;

    rcl_publisher_t dist_sensor_front_publisher_;
    rcl_publisher_t dist_sensor_back_publisher_;
    rcl_publisher_t servo_joint_position_publisher_;

    sensor_msgs__msg__Range range_msg_;
    sensor_msgs__msg__JointState servo_joint_msg_;
};

#endif // ROT_DIST_SENSOR_H
