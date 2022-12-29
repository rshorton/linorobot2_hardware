#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/float32.h>
#if defined(ELSABOT_MOTOR_DIAG)
#include <elsabot_custom_messages/msg/motor_diag.h>
#endif

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "battery_diagnostics.h"

BatteryDiags::BatteryDiags(const String &name, INA226 &pwr_mon):
    name_(name),
    pwr_mon_(pwr_mon),
    inited_(false)
{
}

void BatteryDiags::create(rcl_node_t &node)
{
    if (inited_) {
        return;
    }

    String topic_base = String("battery/" + name_);

    rclc_publisher_init_default( 
        &battery_voltage_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        String(topic_base + "/voltage").c_str()
    );
    
    rclc_publisher_init_default( 
        &battery_current_publisher_, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        String(topic_base + "/current").c_str()
    );

    inited_ = true;
}

void BatteryDiags::destroy(rcl_node_t &node)
{
    if (!inited_) {
        return;
    }
    inited_ = false;
    rcl_publisher_fini(&battery_voltage_publisher_, &node);
    rcl_publisher_fini(&battery_current_publisher_, &node);
}

void BatteryDiags::publish()
{
    if (!inited_) {
        return;
    }

    battery_voltage_msg.data = pwr_mon_.readBusVoltage();
    battery_current_msg.data = pwr_mon_.readShuntCurrent();

    rcl_publish(&battery_voltage_publisher_, &battery_voltage_msg, NULL);
    rcl_publish(&battery_current_publisher_, &battery_current_msg, NULL);
}

