#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/float32.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "diagnostics.h"

//Fix - The default config for micro ros only supports publishing 10 topics.
// Attempts at changing the config to allow more didn't work.  Only those topics
// below that were needed for debugging were enabled.

MotorDiags::MotorDiags():
    inited_(false)
{
}

void MotorDiags::create(rcl_node_t &node, int index)
{
    if (inited_) {
        return;
    }

    index_ = index;

    String idx_str = String(index);
    String topic_base = String("motor_" + idx_str);

    rclc_publisher_init_default( 
        &motor_req_rpm_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        String(topic_base + "/req_rpm").c_str()
    );

    rclc_publisher_init_default( 
        &motor_cur_rpm_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        String(topic_base + "/cur_rpm").c_str()
    );

    rclc_publisher_init_default( 
        &motor_current_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        String(topic_base + "/current").c_str()
    );

    rclc_publisher_init_default( 
        &motor_pid_error_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        String(topic_base + "/pid_error").c_str()
    );

    rclc_publisher_init_default( 
        &motor_pid_integral_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        String(topic_base + "/pid_integral").c_str()
    );

    rclc_publisher_init_default( 
        &motor_pid_derivative_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        String(topic_base + "/pid_derivative").c_str()
    );

    rclc_publisher_init_default( 
        &motor_pid_output_raw_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        String(topic_base + "/pid_output_raw").c_str()
    );

    rclc_publisher_init_default( 
        &motor_pid_output_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        String(topic_base + "/pid_output").c_str()
    );

    inited_ = true;
}

void MotorDiags::destroy(rcl_node_t &node)
{
    if (!inited_) {
        return;
    }
    inited_ = false;

    rcl_publisher_fini(&motor_req_rpm_publisher_, &node);
    rcl_publisher_fini(&motor_cur_rpm_publisher_, &node);
    rcl_publisher_fini(&motor_current_publisher_, &node);
    rcl_publisher_fini(&motor_pid_error_publisher_, &node);
    rcl_publisher_fini(&motor_pid_integral_publisher_, &node);
    rcl_publisher_fini(&motor_pid_derivative_publisher_, &node);
    rcl_publisher_fini(&motor_pid_output_raw_publisher_, &node);
    rcl_publisher_fini(&motor_pid_output_publisher_, &node);
}

void MotorDiags::publish(float rpm_req, float rpm_cur, float current, PID const &pid)
{
    if (!inited_) {
        return;
    }

    std_msgs__msg__Float32 msg;

    msg.data = rpm_req;
    rcl_publish(&motor_req_rpm_publisher_, &msg, NULL);

    msg.data = rpm_cur;
    rcl_publish(&motor_cur_rpm_publisher_, &msg, NULL);

    msg.data = current;
    rcl_publish(&motor_current_publisher_, &msg, NULL);

    msg.data = pid.getError();
    rcl_publish(&motor_pid_error_publisher_, &msg, NULL);

    msg.data = pid.getIntegral();
    rcl_publish(&motor_pid_integral_publisher_, &msg, NULL);

    msg.data = pid.getDerivative();
    rcl_publish(&motor_pid_derivative_publisher_, &msg, NULL);

    msg.data = pid.getOutputRaw();
    rcl_publish(&motor_pid_output_raw_publisher_, &msg, NULL);

    msg.data = pid.getOutputConstrained();
    rcl_publish(&motor_pid_output_publisher_, &msg, NULL);
}

