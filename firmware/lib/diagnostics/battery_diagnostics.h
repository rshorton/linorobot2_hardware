#ifndef BATTERY_DIAGNOSTICS_H
#define BATTERY_DIAGNOSTICS_H

#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <std_msgs/msg/float32.h>

#include "INA226.h"

class BatteryDiags
{
    public:
        BatteryDiags(const String &name, INA226 &pwr_mon);
        void create(rcl_node_t &node);
        void destroy(rcl_node_t &node);
        void publish();

    private:
        String name_;
        INA226 &pwr_mon_;
        
        bool inited_;

        rcl_publisher_t battery_voltage_publisher_;
        rcl_publisher_t battery_current_publisher_;
        
        std_msgs__msg__Float32 battery_voltage_msg;
        std_msgs__msg__Float32 battery_current_msg;
};

#endif // BATTERY_DIAGNOSTICS_H