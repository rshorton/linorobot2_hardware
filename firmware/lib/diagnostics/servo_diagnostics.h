#ifndef SERVO_DIAGNOSTICS_H
#define SERVO_DIAGNOSTICS_H

#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#if defined(PUBLISH_SERVO_DIAGS)
#include <elsabot_custom_messages/msg/servo_diag.h>
#endif

#include "pid.h"
#include "encoder_interface.h"

class ServoDiags
{
    public:
        ServoDiags();
        void create(rcl_node_t &node, const String &name);
        void destroy(rcl_node_t &node);
        void publish(struct timespec time_stamp, float cur, float req, PID const &pid,
                     EncoderInterface &encoder);

    private:
        bool inited_;
        String name_;
        rcl_publisher_t servo_diag_publisher_;
#if defined(PUBLISH_SERVO_DIAGS)        
        elsabot_custom_messages__msg__ServoDiag servo_diag_msg_;
#endif        
};

#endif // SERVO_DIAGNOSTICS_H