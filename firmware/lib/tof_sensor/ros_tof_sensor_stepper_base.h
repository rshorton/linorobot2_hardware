#ifndef ROS_TOF_SENSOR_BASE_H
#define ROS_TOF_SENSOR_BASE_H

#include "Arduino.h"
#include <Wire.h>
#include <rcl/rcl.h>
#include <sensor_msgs/msg/joint_state.h>

#include "stepper_28BYJ48.h"

// This class is used when a TOF sensor is mounted on a 28BYJ48 stepper motor.  It controls sweeping the
// stepper between a specified sweep angle.  (While this approach did function, the time between measurements
// for a position was somewhat slow, ~2 every seconds.)

class RosTofSensorStepperBase
{
private:
    enum class State { reset, init, homing, moving, at_pos };

public:
    RosTofSensorStepperBase(Stepper28BYJ48 &stepper, const String &frame_name, const String &topic_name);
    ~RosTofSensorStepperBase();

    void init(rcl_node_t &node);
    void destroy(rcl_node_t &node);

    void start(int angle_sweep_deg);
    void stop();

    bool update();
    bool started() const {
        return state_ == State::moving ||
               state_ == State::at_pos;
    }

    void goto_next_position();

    bool at_position() const;
    float get_current_pos_angle() const;

    void publish_joint_state();

private:
    void init_joint_msg();
    inline long angle_to_steps(int angle) const;
    void publish(float angle);

private:
    Stepper28BYJ48 &stepper_;
    const String frame_name1_;
    const String frame_name2_;

    State state_{State::reset};
    long steps_per_sweep_{0};
    float angle_sweep_deg_{0.0f};
    bool next_pos_{false};

    rcl_publisher_t publisher_;
    sensor_msgs__msg__JointState joint_msg_;
};

#endif // ROS_TOF_SENSOR_BASE_H
