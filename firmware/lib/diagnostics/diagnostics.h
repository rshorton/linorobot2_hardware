#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <elsabot_custom_messages/msg/motor_diag.h>

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
        elsabot_custom_messages__msg__MotorDiag motor_diag_msg_;
};

#endif