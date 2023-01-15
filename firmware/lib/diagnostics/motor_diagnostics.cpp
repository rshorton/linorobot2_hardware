#include <Arduino.h>

#include "config.h"

#include <micro_ros_platformio.h>

#include <rclc/rclc.h>

#include <std_msgs/msg/float32.h>
#if defined(PUBLISH_MOTOR_DIAGS)
#include <elsabot_custom_messages/msg/motor_diag.h>
#endif

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "motor_diagnostics.h"

MotorDiags::MotorDiags():
    inited_(false)
{
    //motor_diag_msg_.header.frame_id = micro_ros_string_utilities_set(motor_diag_msg_.header.frame_id, "none");
}

void MotorDiags::create(rcl_node_t &node, int index)
{
#if defined(PUBLISH_MOTOR_DIAGS)
    if (inited_) {
        return;
    }

    index_ = index;

    String idx_str = String(index);
    String topic_base = String("motor_" + idx_str);

    rclc_publisher_init_default( 
        &motor_diag_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(elsabot_custom_messages, msg, MotorDiag),
        String(topic_base + "/motor_diag").c_str()
    );
    inited_ = true;
#endif    
}

void MotorDiags::destroy(rcl_node_t &node)
{
    if (!inited_) {
        return;
    }
    inited_ = false;
    rcl_publisher_fini(&motor_diag_publisher_, &node);
}

void MotorDiags::publish(struct timespec time_stamp, float rpm_req, float rpm_cur, float current, PID const &pid)
{
#if defined(PUBLISH_MOTOR_DIAGS)
    if (!inited_) {
        return;
    }

    motor_diag_msg_.header.stamp.sec = time_stamp.tv_sec;
    motor_diag_msg_.header.stamp.nanosec = time_stamp.tv_nsec;

    motor_diag_msg_.req_rpm = rpm_req;
    motor_diag_msg_.cur_rpm = rpm_cur;
    motor_diag_msg_.current = current;
    motor_diag_msg_.pid_error = pid.getError();
    motor_diag_msg_.pid_integral = pid.getIntegral();
    motor_diag_msg_.pid_derivative = pid.getDerivative();
    motor_diag_msg_.pid_output_raw = pid.getOutputRaw();
    motor_diag_msg_.pid_output = pid.getOutputConstrained();
    rcl_publish(&motor_diag_publisher_, &motor_diag_msg_, NULL);
#endif    
}

