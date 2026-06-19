#include "Arduino.h"
#include <stdlib.h>
#include <stdio.h>
#include <limits>

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>

#include "ros_tof_sensor_stepper_base.h"
#include "logger.h"
#include "time_util.h"

namespace {
    const int NUM_JOINTS = 2;

    const long STOP_POSITION_ANGLE_FROM_0_DEG = 35.0f;                          // Angle from forward to stop position
    const long MAX_TRAVEL_ANGLE = (STOP_POSITION_ANGLE_FROM_0_DEG + 48.0f);     // Max angular range between left and right physical limits
}

RosTofSensorStepperBase::RosTofSensorStepperBase(Stepper28BYJ48 &stepper, const String &frame_name1,  const String &frame_name2) :
    stepper_(stepper),
    frame_name1_(frame_name1),
    frame_name2_(frame_name2)
{
    joint_msg_.name.data = (rosidl_runtime_c__String*)malloc(sizeof(rosidl_runtime_c__String)*NUM_JOINTS);
    joint_msg_.name.size = 2;
    joint_msg_.name.capacity = NUM_JOINTS;

    // Initialize each sequence stucture with the const string for each joint
    joint_msg_.name.data[0].data = (char*)frame_name1_.c_str();
    joint_msg_.name.data[0].size = strlen(frame_name1_.c_str()) + 1;
    joint_msg_.name.data[0].capacity = joint_msg_.name.data[0].size;

    joint_msg_.name.data[1].data = (char*)frame_name2_.c_str();
    joint_msg_.name.data[1].size = strlen(frame_name2_.c_str()) + 1;
    joint_msg_.name.data[1].capacity = joint_msg_.name.data[1].size;

    joint_msg_.position.data = (double *) malloc(NUM_JOINTS * sizeof(double));
    joint_msg_.position.size= 2;
    joint_msg_.position.capacity = NUM_JOINTS;

    joint_msg_.velocity.data = NULL;
    joint_msg_.velocity.size = 0;
    joint_msg_.velocity.capacity = 0;

    joint_msg_.effort.data = NULL;
    joint_msg_.effort.size = 0;
    joint_msg_.effort.capacity = 0;
}

RosTofSensorStepperBase::~RosTofSensorStepperBase()
{
    stepper_.stop();
}

void RosTofSensorStepperBase::init(rcl_node_t &node)
{
    if (state_ == State::reset) {
        rclc_publisher_init_default(
            &publisher_,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "joint_states");

        state_ = State::init;
    }
}

void RosTofSensorStepperBase::destroy(rcl_node_t &node)
{
    if (state_ != State::reset) {
        rcl_publisher_fini(&publisher_, &node);
        state_ = State::reset;
    }        
    stop();
}

float RosTofSensorStepperBase::get_current_pos_angle() const
{
    float pos = stepper_.get_position();
    return 360.0f*pos/(float)stepper_.get_steps_per_rev();
}

bool RosTofSensorStepperBase::at_position() const
{
    return state_ == State::at_pos;
}

void RosTofSensorStepperBase::publish_joint_state()
{
    if (state_ == State::reset) {
        return;
    }

    publish(get_current_pos_angle());
}

void RosTofSensorStepperBase::publish(float angle)
{
    struct timespec time_stamp = TimeUtil::get_time();
    joint_msg_.header.stamp.sec = time_stamp.tv_sec;
    joint_msg_.header.stamp.nanosec = time_stamp.tv_nsec;
    auto rad = angle*M_PI/180.0f;
    joint_msg_.position.data[0] = rad;
    joint_msg_.position.data[1] = rad;
    rcl_publish(&publisher_, &joint_msg_, NULL);
}

inline long RosTofSensorStepperBase::angle_to_steps(int angle) const
{
    return stepper_.get_steps_per_rev()*angle/360;
}

void RosTofSensorStepperBase::start(int angle_sweep_deg)
{
    if (state_ == State::reset) {
        return;
    }

    steps_per_sweep_ = angle_to_steps(angle_sweep_deg);
    auto max_steps = angle_to_steps(STOP_POSITION_ANGLE_FROM_0_DEG)*2;

    // Limit sweep to available range
    if (steps_per_sweep_ > max_steps) {
        steps_per_sweep_ = max_steps - angle_to_steps(5)*2;
    }

    // Home the stepper by moving enough toward stop position to 
    // ensure it is reached
    stepper_.move_to_position(-angle_to_steps(MAX_TRAVEL_ANGLE + 2));
    state_ = State::homing;

    Logger::log_message(Logger::LogLevel::Info, "RosTofSensorStepperBase::start, angle: %d, steps_per_sweep: %d",
                        angle_sweep_deg, steps_per_sweep_);
}

void RosTofSensorStepperBase::stop()
{
    if (state_ != State::reset) {
        stepper_.move_to_position(0);
        publish(0);
        state_ = State::at_pos;
        next_pos_ = false;
    }
}

void RosTofSensorStepperBase::goto_next_position()
{
    next_pos_ = true;
    update();
}

bool RosTofSensorStepperBase::update()
{
    switch(state_) 
    {
        case State::homing:
            if (stepper_.at_position()) {
                // Reached limit.
                stepper_.set_current_pos(-angle_to_steps(STOP_POSITION_ANGLE_FROM_0_DEG));
#ifdef USE_OSC_MODE                
                stepper_.set_osc_mode(steps_per_sweep_/2);
#else
                stepper_.move_to_position(steps_per_sweep_/2);
#endif                
                state_ = State::moving;
                Logger::log_message(Logger::LogLevel::Info, "RosTofSensorStepperBase::update, homed");
            }
            break;
#ifndef USE_OSC_MODE            
        case State::moving:
            if (stepper_.at_position()) {
                Logger::log_message(Logger::LogLevel::Info, "RosTofSensorStepperBase::update, at position");
                state_ = State::at_pos;
                publish_joint_state();
            }
            break;
        case State::at_pos:
            if (next_pos_) {
                next_pos_ = false;
                state_ = State::moving;
                Logger::log_message(Logger::LogLevel::Info, "RosTofSensorStepperBase::update, move to next position");
                stepper_.move_to_position(steps_per_sweep_/2*(stepper_.get_target_position() < 0? 1: -1));
            }
            return true;
#endif            
        default:
            break;
    }
    return false;
}
