#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>

#include "pid.h"

class MotorDiags
{
    public:
        MotorDiags();
        void create(rcl_node_t &node, int index);
        void destroy(rcl_node_t &node);
        void publish(float rpm_cur, float rpm_req, float current, PID const &pid);

    private:
        bool inited_;
        int index_;
        rcl_publisher_t motor_req_rpm_publisher_;
        rcl_publisher_t motor_cur_rpm_publisher_;
        rcl_publisher_t motor_current_publisher_;
        rcl_publisher_t motor_pid_error_publisher_;
        rcl_publisher_t motor_pid_integral_publisher_;
        rcl_publisher_t motor_pid_derivative_publisher_;
        rcl_publisher_t motor_pid_output_raw_publisher_;
        rcl_publisher_t motor_pid_output_publisher_;
};

#endif