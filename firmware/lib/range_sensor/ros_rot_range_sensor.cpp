#include "Arduino.h"
#include <stdlib.h>
#include <stdio.h>
#include <limits>

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>

#include "logger.h"

#include "ros_rot_range_sensor.h"
#include "ros_range_sensor.h"

// fix - move definition to common file
extern struct timespec getTime();

RosRotatingRangeSensor::RosRotatingRangeSensor(const String &joint_frame_name, RosRangeSensor &range_sensor, SerialServo &servo,
                                               const float (&positions)[], int position_cnt, float zero_offset_deg, int pos_delay = 1000) :
    joint_frame_name_(joint_frame_name),
    range_sensor_(range_sensor),
    servo_(servo),
    positions_deg_(positions),
    position_cnt_(position_cnt),
    zero_offset_deg_(zero_offset_deg),
    pos_delay_(pos_delay)
{
    // Allocate the memory that holds the structure for the joint state msg
    servo_joint_msg_.name.data = (rosidl_runtime_c__String*)malloc(sizeof(rosidl_runtime_c__String));
    servo_joint_msg_.name.size = 1;
    servo_joint_msg_.name.capacity = 1;

    // Initialize each sequence stucture with the const string for each joint
    servo_joint_msg_.name.data[0].data = (char*)joint_frame_name_.c_str();
    servo_joint_msg_.name.data[0].size = strlen(joint_frame_name_.c_str()) + 1;
    servo_joint_msg_.name.data[0].capacity = servo_joint_msg_.name.data[0].size;

    servo_joint_msg_.position.data = (double *) malloc(sizeof(double));
    servo_joint_msg_.position.size= 1;
    servo_joint_msg_.position.capacity = 1;

    servo_joint_msg_.velocity.data = NULL;
    servo_joint_msg_.velocity.size = 0;
    servo_joint_msg_.velocity.capacity = 0;

    servo_joint_msg_.effort.data = NULL;
    servo_joint_msg_.effort.size = 0;
    servo_joint_msg_.effort.capacity = 0;
}

void RosRotatingRangeSensor::init(rcl_node_t &node)
{
    if (state_ == State::kInit) {
        rclc_publisher_init_default(
            &servo_joint_position_publisher_,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "joint_states");

        range_sensor_.init(node);

        state_ = State::kReady;
    }
}

void RosRotatingRangeSensor::destroy(rcl_node_t &node)
{
    if (state_ != State::kInit) {
        rcl_publisher_fini(&servo_joint_position_publisher_, &node);
        range_sensor_.destroy(node);
        state_ = State::kInit;
    }        
}

void RosRotatingRangeSensor::publish_servo_position(rcl_publisher_t &pub, float angle)
{
    if (state_ == State::kInit) {
        return;
    }

    struct timespec time_stamp = getTime();
    servo_joint_msg_.header.stamp.sec = time_stamp.tv_sec;
    servo_joint_msg_.header.stamp.nanosec = time_stamp.tv_nsec;
    servo_joint_msg_.position.data[0] = angle*M_PI/180.0f;
    rcl_publish(&pub, &servo_joint_msg_, NULL);
}

void RosRotatingRangeSensor::start(bool scan)
{
    if (state_ != State::kInit)
    {
        if (scan == scan_ &&
            state_ != State::kReady) {
            return;
        }
        scan_ = scan;
        pos_idx_ = 0;
        next_update_ = millis() + move_next();
    }
}

void RosRotatingRangeSensor::stop()
{
    if (state_ != State::kInit)
    {
        state_ = State::kReady;
    }
}

void RosRotatingRangeSensor::update()
{
    unsigned long now = millis();
    unsigned long delay = 50;

#if defined(DEBUG_LOG)
    Logger::log_message(Logger::LogLevel::Info, "RotatingDistSensor::update state: %d", state_);
#endif    

    publish_servo_position(servo_joint_position_publisher_, positions_deg_[pos_idx_]);

    if (state_ != State::kMoving)
    {
        publish_servo_position(servo_joint_position_publisher_, positions_deg_[pos_idx_]);        
    }

    if ((long)(now - next_update_) < 0)
    {
        return;
    }

    switch (state_)
    {
        default:
        case State::kInit:
        case State::kReady:
            break;

        case State::kMoving:
            state_ = State::kPostMoveDelay;
            delay = pos_delay_;
            break;

        case State::kPostMoveDelay:
            state_ = State::kRange;
            range_sensor_.start();
            delay = 0;
            break;

        case State::kRange:
            // Returns true when measurement complete and sufficient settling time has occurred
            if (!range_sensor_.update())
            {
                state_ = State::kPostRangeDelay;
                delay = 10;
            }
            break;

        case State::kPostRangeDelay:
            delay = move_next();
            break;
    }
    next_update_ = now + delay;
}

unsigned long RosRotatingRangeSensor::move_servo(float pos)
{
    auto move_duration = (int)(abs(servo_pos_ - pos) * move_ms_per_degree_);
    servo_pos_ = servo_.move(pos, move_duration);
    state_ = State::kMoving;
    return move_duration;
}

unsigned long RosRotatingRangeSensor::move_next()
{
    if (!scan_ || ++pos_idx_ >= position_cnt_) {
        pos_idx_ = 0;
    }        
    return move_servo(positions_deg_[pos_idx_] + zero_offset_deg_);
}
