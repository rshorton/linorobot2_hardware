#include <Arduino.h>

#include "config.h"

#include <micro_ros_platformio.h>

#include <rclc/rclc.h>

#include <std_msgs/msg/float32.h>
#if defined(PUBLISH_SERVO_DIAGS)
#include <elsabot_custom_messages/msg/servo_diag.h>
#endif

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "servo_diagnostics.h"

ServoDiags::ServoDiags():
    inited_(false)
{
}

void ServoDiags::create(rcl_node_t &node, const String &name)
{
#if defined(PUBLISH_SERVO_DIAGS)
    if (inited_) {
        return;
    }

    name_ = name;

    String topic_base = String("ebot/servo_" + name_);

    rclc_publisher_init_default( 
        &servo_diag_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(elsabot_custom_messages, msg, ServoDiag),
        String(topic_base + "/servo_diag").c_str()
    );
    inited_ = true;
#endif    
}

void ServoDiags::destroy(rcl_node_t &node)
{
#if defined(PUBLISH_SERVO_DIAGS)
    if (!inited_) {
        return;
    }
    inited_ = false;
    rcl_publisher_fini(&servo_diag_publisher_, &node);
#endif    
}

void ServoDiags::publish(struct timespec time_stamp, float req, float cur,
                         PID const &pid, EncoderInterface &encoder)
{
#if defined(PUBLISH_SERVO_DIAGS)
    if (!inited_) {
        return;
    }

    servo_diag_msg_.header.stamp.sec = time_stamp.tv_sec;
    servo_diag_msg_.header.stamp.nanosec = time_stamp.tv_nsec;

    servo_diag_msg_.req = req;
    servo_diag_msg_.cur = cur;
    servo_diag_msg_.pid_error = pid.getError();
    servo_diag_msg_.pid_integral = pid.getIntegral();
    servo_diag_msg_.pid_derivative = pid.getDerivative();
    servo_diag_msg_.pid_output_raw = pid.getOutputRaw();
    servo_diag_msg_.pid_output = pid.getOutputConstrained();
    servo_diag_msg_.encoder_tics = encoder.read();
    rcl_publish(&servo_diag_publisher_, &servo_diag_msg_, NULL);
#endif    
}

