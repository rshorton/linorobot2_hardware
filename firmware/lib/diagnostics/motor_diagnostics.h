#ifndef MOTOR_DIAGNOSTICS_H
#define MOTOR_DIAGNOSTICS_H

#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#if defined(PUBLISH_MOTOR_DIAGS)
#include <elsabot_custom_messages/msg/motor_diag.h>
#endif

#include "pid.h"

class MotorDiags
{
    public:
        MotorDiags();
        void create(rcl_node_t &node, int index);
        void destroy(rcl_node_t &node);
        void publish(struct timespec time_stamp, float rpm_cur, float rpm_req, float current, PID const &pid);

    private:
        bool inited_;
        int index_;
        rcl_publisher_t motor_diag_publisher_;
#if defined(PUBLISH_MOTOR_DIAGS)        
        elsabot_custom_messages__msg__MotorDiag motor_diag_msg_;
#endif        
};

#endif // MOTOR_DIAGNOSTICS_H