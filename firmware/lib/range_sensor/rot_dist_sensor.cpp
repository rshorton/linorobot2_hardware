#include "Arduino.h"
#include <stdlib.h>
#include <stdio.h>
#include <limits>

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/joint_state.h>

#include "rot_dist_sensor.h"

namespace
{
#if 1
    const int SCAN_POS_CENTER = 300;
    const int SCAN_POS_START = SCAN_POS_CENTER - 140;
    const int SCAN_POS_END = SCAN_POS_CENTER + 140;
    const int SCAN_POS_STEP = 70;
    const float SCAN_ANGLE_DEG = 60.0;
#else
    const int SCAN_POS_START = 100;
    const int SCAN_POS_CENTER = 300;
    const int SCAN_POS_END = 500;
    const int SCAN_POS_STEP = 100;
    const float SCAN_ANGLE_DEG = 90.0;
#endif    
    const int SERVO0_ID = 1;

    const char* SERVO_PAN_JOINT_FRONT = "hcsr04_pan_joint_front";
    const char* SERVO_PAN_JOINT_BACK = "hcsr04_pan_joint_back";
    const int NUM_SERVO_JOINTS = 2;

    const char* DIST_SENSOR_FRAME_FRONT = "front_hcsr04";
}

// fix - move definition to common file
extern struct timespec getTime();

RotatingDistSensor::RotatingDistSensor(HCSR04 &dist_sensor0, SerialServo &servo0) :
    dist_sensor0_(dist_sensor0),
    servo0_(servo0),
    state_(State::kInit),
    next_update_(0),
    scan_(true),
    servo_pos_(0)
{
    // Allocate the memory that holds the sequence structures for each name
    servo_joint_msg_.name.data = (rosidl_runtime_c__String*)malloc(sizeof(rosidl_runtime_c__String)*NUM_SERVO_JOINTS);
    servo_joint_msg_.name.size = 2;
    servo_joint_msg_.name.capacity = NUM_SERVO_JOINTS;

    // Initialize each sequence stucture with the const string for each joint
    servo_joint_msg_.name.data[0].data = (char*)SERVO_PAN_JOINT_FRONT;
    servo_joint_msg_.name.data[0].size = strlen(SERVO_PAN_JOINT_FRONT) + 1;
    servo_joint_msg_.name.data[0].capacity = servo_joint_msg_.name.data[0].size;

    servo_joint_msg_.name.data[1].data = (char*)SERVO_PAN_JOINT_BACK;
    servo_joint_msg_.name.data[1].size = strlen(SERVO_PAN_JOINT_BACK) + 1;
    servo_joint_msg_.name.data[1].capacity = servo_joint_msg_.name.data[1].size;

    servo_joint_msg_.position.data = (double *) malloc(NUM_SERVO_JOINTS * sizeof(double));
    servo_joint_msg_.position.size= 2;
    servo_joint_msg_.position.capacity = NUM_SERVO_JOINTS;

    servo_joint_msg_.velocity.data = NULL;
    servo_joint_msg_.velocity.size = 0;
    servo_joint_msg_.velocity.capacity = 0;

    servo_joint_msg_.effort.data = NULL;
    servo_joint_msg_.effort.size = 0;
    servo_joint_msg_.effort.capacity = 0;

    //range_msg_.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    range_msg_.radiation_type = 0;
    range_msg_.field_of_view = dist_sensor0_.get_field_of_view();
    range_msg_.min_range = 0.02;
    range_msg_.max_range = 2.5;
}

void RotatingDistSensor::init(rcl_node_t &node)
{
    if (state_ == State::kInit) {
        rclc_publisher_init_default(
            &dist_sensor_front_publisher_,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
            "range/front");

        rclc_publisher_init_default(
            &dist_sensor_back_publisher_,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
            "range/back");

        rclc_publisher_init_default(
            &servo_joint_position_publisher_,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "sensor_joint_states");

        state_ = State::kReady;
    }
}

void RotatingDistSensor::destroy(rcl_node_t &node)
{
    if (state_ != State::kInit) {
        rcl_publisher_fini(&dist_sensor_front_publisher_, &node);
        rcl_publisher_fini(&dist_sensor_back_publisher_, &node);
        rcl_publisher_fini(&servo_joint_position_publisher_, &node);
        state_ = State::kInit;
    }        
}

void RotatingDistSensor::publish_range(rcl_publisher_t &pub, const char* frame, float range, float angle)
{
    if (state_ == State::kInit) {
        return;
    }

    struct timespec time_stamp = getTime();
    range_msg_.header.stamp.sec = time_stamp.tv_sec;
    range_msg_.header.stamp.nanosec = time_stamp.tv_nsec;
    
    range_msg_.header.frame_id.data = (char*)frame;
    range_msg_.header.frame_id.size = strlen(frame) + 1;
    range_msg_.header.frame_id.capacity = range_msg_.header.frame_id.size;

    range_msg_.range = range;

    rcl_publish(&pub, &range_msg_, NULL);
}

void RotatingDistSensor::publish_servo_position(rcl_publisher_t &pub, float angle)
{
    if (state_ == State::kInit) {
        return;
    }

    struct timespec time_stamp = getTime();
    servo_joint_msg_.header.stamp.sec = time_stamp.tv_sec;
    servo_joint_msg_.header.stamp.nanosec = time_stamp.tv_nsec;
    servo_joint_msg_.position.data[0] = angle;
    servo_joint_msg_.position.data[1] = angle;
    rcl_publish(&pub, &servo_joint_msg_, NULL);
}

void RotatingDistSensor::start(bool scan)
{
    if (state_ != State::kInit)
    {
        scan_ = scan;
        next_update_ = millis() + move_servo(scan_? SCAN_POS_START: SCAN_POS_CENTER);
    }
}

void RotatingDistSensor::stop()
{
    if (state_ == State::kInit)
    {
        state_ = State::kReady;
    }
}

void RotatingDistSensor::update()
{
     unsigned long now = millis();
     unsigned long delay = 50;

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

        case State::kMove:
            publish_servo_position(servo_joint_position_publisher_, servo_pos_to_angle());
            dist_sensor0_.start();
            state_ = State::kRange;
            break;

        case State::kRange:
        {
            float dist;
            if (dist_sensor0_.get_distance_m(dist))
            {
                publish_range(dist_sensor_front_publisher_, DIST_SENSOR_FRAME_FRONT, dist, 0);
            }

            if (dist_sensor0_.finished())
            {
                delay = move_next();
            }
        }
    }
    next_update_ = now + delay;
}

unsigned long RotatingDistSensor::move_servo(int16_t pos)
{
    auto move_duration = abs(pos - servo_pos_) * 2;
    servo0_.move(SERVO0_ID, pos, move_duration);
    servo_pos_ = pos;
    state_ = State::kMove;
    return move_duration + 1000;
}

unsigned long RotatingDistSensor::move_next()
{
    if (!scan_) {
        return 1000;
    }

    auto new_pos = 0;
    if (servo_pos_ >= SCAN_POS_END) {
        new_pos = SCAN_POS_START;
    } else {       
        new_pos = servo_pos_ + SCAN_POS_STEP;
    }        
    return move_servo(new_pos);
}

float RotatingDistSensor::servo_pos_to_angle()
{
    return (float)(servo_pos_ - SCAN_POS_CENTER)/(SCAN_POS_END - SCAN_POS_START)*SCAN_ANGLE_DEG*M_PI/180.0;
}